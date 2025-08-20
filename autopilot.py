#!/usr/bin/env python3
"""
Betaflight MSP Autopilot (Altitude Hold)

High-level controller to arm, takeoff to a target altitude, hover (alt-hold), and land,
using MSP over serial. Designed to be simple and plug-and-play.

Features
- Autodetect serial port (Windows-friendly, also works on Linux).
- Read altitude (baro) and GPS via MSP. Use baro as primary, optional GPS blend.
- PI controller on throttle to hold target altitude.
- Flight states: MANUAL, TAKEOFF, HOLD, LAND.
- Yaw lock (keeps yaw centered).
- Keyboard controls (no joystick needed):
  - a: Arm/Disarm
  - t: Auto takeoff to --target-alt (then HOLD)
  - h: Enter HOLD at current altitude
  - l: Land
  - + / - (or w/s): Increase/decrease target altitude by 0.1 m
  - z: Zero altitude bias now
  - q: Quit (disarm & exit)
- Optional joystick override (pygame): any stick/button latches MANUAL; yaw locked unless --no-yaw-lock.
- Logging with verbosity controls (-v for verbose, -q for quiet)
- Safety: clamps, disarms on exit, handles missing sensors.

IMPORTANT SAFETY NOTICE
- Test with props off first. Ensure Betaflight is configured to ARM via AUX1 (or adapt below).
- Altitude from MSP is relative; controller assumes ANGLE mode and a properly tuned quad.
- You are responsible for safe operation.
"""

import sys
import time
import struct
import logging
import argparse
import threading
import platform
from dataclasses import dataclass

from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except Exception as e:
    print("pyserial is required (pip install pyserial)")
    raise

# Optional joystick (pygame)
PYGAME_AVAILABLE = False
try:
    import pygame  # type: ignore
    PYGAME_AVAILABLE = True
except Exception:
    PYGAME_AVAILABLE = False

# Non-blocking keyboard (Windows-first)
WINDOWS = platform.system().lower().startswith("win")
if WINDOWS:
    try:
        import msvcrt  # type: ignore
    except Exception:
        msvcrt = None
else:
    import select
    import termios
    import tty

# MSP Constants
BAUD_RATE = 115200
MSP_SET_RAW_RC = 200
MSP_ALTITUDE = 109
MSP_RAW_GPS = 106

# RC Channels (Betaflight order: Roll, Pitch, Throttle, Yaw, AUX1..)
RC_CHANNELS_COUNT = 8
ROLL_IDX = 0
PITCH_IDX = 1
THROTTLE_IDX = 2
YAW_IDX = 3
AUX1_IDX = 4  # Arm channel (assumes Betaflight ARM is mapped to AUX1)

ARM_VALUE = 1800
DISARM_VALUE = 1000
RC_MIN = 1000
RC_MAX = 2000
RC_MID = 1500

# Default control gains (PWM units per meter)
DEFAULT_KP = 40.0
DEFAULT_KI = 15.0
DEFAULT_IMAX = 250.0  # Max absolute integral contribution (PWM units)

# Default behavior
DEFAULT_TARGET_ALT = 1.5  # meters
DEFAULT_HOVER_PWM = 1550  # starting guess; adjust via --hover-pwm
ALT_REQ_INTERVAL = 0.05   # seconds
GPS_REQ_INTERVAL = 0.5    # seconds
RC_SEND_INTERVAL = 0.02   # 50 Hz
UI_PRINT_INTERVAL = 0.5
SENSOR_TIMEOUT = 1.5      # seconds until altitude considered stale


def calculate_checksum(payload: bytes) -> int:
    chk = 0
    for b in payload:
        chk ^= b
    return chk


def clamp(val, vmin, vmax):
    return vmin if val < vmin else vmax if val > vmax else val


class MSPClient:
    def __init__(self, port: Optional[str], baud: int = BAUD_RATE, auto_detect: bool = True, logger: Optional[logging.Logger] = None):
        self.log = logger or logging.getLogger(__name__)
        self.port = port
        self.baud = baud
        if self.port is None and auto_detect:
            self.port = self._auto_detect_port()
        if self.port is None:
            raise RuntimeError("No serial port provided and auto-detect failed.")
        self.log.info(f"Opening serial port {self.port} @ {self.baud}")
        # Non-blocking reads (timeout=0)
        self.ser = serial.Serial(self.port, self.baud, timeout=0)
        self.buf = b""
        # Telemetry
        self.altitude_cm: Optional[int] = None
        self.alt_timestamp: float = 0.0
        self.gps_fix: bool = False
        self.gps_num_sat: int = 0
        self.gps_lat: Optional[float] = None
        self.gps_lon: Optional[float] = None
        self.gps_alt_cm: Optional[int] = None
        self.gps_speed_cms: Optional[int] = None
        self.gps_course_deg: Optional[float] = None

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _auto_detect_port(self) -> Optional[str]:
        candidates = []
        for p in list_ports.comports():
            desc = f"{p.device} {p.description} {p.manufacturer} {p.hwid}".lower()
            # Prefer likely USB UARTs
            score = 0
            if "usb" in desc or "uart" in desc or "acm" in desc or "cp210" in desc or "ch340" in desc or "ftdi" in desc or "stm" in desc:
                score += 2
            if WINDOWS and p.device.upper().startswith("COM"):
                score += 1
            if not WINDOWS and ("/dev/ttyUSB" in p.device or "/dev/ttyACM" in p.device):
                score += 1
            candidates.append((score, p.device))
        candidates.sort(reverse=True)
        return candidates[0][1] if candidates else None

    def send_msp(self, cmd: int, data: Optional[bytes] = None):
        if data:
            size = len(data)
        else:
            size = 0
        header = b"$M<"
        payload_header = struct.pack("<BB", size, cmd)
        full_payload = payload_header + (data if data else b"")
        checksum = calculate_checksum(full_payload)
        pkt = header + full_payload + struct.pack("<B", checksum)
        try:
            self.ser.write(pkt)
        except Exception as e:
            self.log.error(f"Serial write error: {e}")

    def request(self, cmd: int):
        self.send_msp(cmd, None)

    def read_and_parse(self):
        try:
            waiting = self.ser.in_waiting
        except Exception:
            waiting = 0
        if waiting > 0:
            try:
                self.buf += self.ser.read(waiting)
            except Exception as e:
                self.log.error(f"Serial read error: {e}")
                return
        self.buf = self._parse_buffer(self.buf)

    def _parse_buffer(self, buf: bytes) -> bytes:
        # Parse all complete packets in buffer
        while True:
            idx = buf.find(b"$M>")
            if idx == -1:
                return buf
            if len(buf) < idx + 5:
                return buf[idx:]
            payload_size = buf[idx + 3]
            cmd = buf[idx + 4]
            if len(buf) < idx + 5 + payload_size + 1:
                return buf[idx:]
            full_payload = buf[idx + 3 : idx + 5 + payload_size]
            recv_chk = buf[idx + 5 + payload_size]
            calc_chk = calculate_checksum(full_payload)
            if recv_chk == calc_chk:
                payload = buf[idx + 5 : idx + 5 + payload_size]
                self._handle_packet(cmd, payload)
                buf = buf[idx + 6 + payload_size :]
            else:
                # skip a byte and retry
                buf = buf[idx + 1 :]
        # unreachable

    def _handle_packet(self, cmd: int, payload: bytes):
        if cmd == MSP_ALTITUDE and len(payload) >= 4:
            altitude_cm = struct.unpack("<i", payload[0:4])[0]
            self.altitude_cm = altitude_cm
            self.alt_timestamp = time.time()
        elif cmd == MSP_RAW_GPS and len(payload) >= 16:
            self.gps_fix = payload[0] != 0
            self.gps_num_sat = payload[1]
            self.gps_lat = struct.unpack("<i", payload[2:6])[0] / 10_000_000.0
            self.gps_lon = struct.unpack("<i", payload[6:10])[0] / 10_000_000.0
            self.gps_alt_cm = struct.unpack("<h", payload[10:12])[0]
            self.gps_speed_cms = struct.unpack("<h", payload[12:14])[0]
            self.gps_course_deg = struct.unpack("<h", payload[14:16])[0] / 10.0


class PIController:
    def __init__(self, kp: float, ki: float, imax: float):
        self.kp = kp
        self.ki = ki
        self.imax = abs(imax)
        self.i_term = 0.0

    def reset(self):
        self.i_term = 0.0

    def update(self, error: float, dt: float) -> float:
        # Proportional
        p = self.kp * error
        # Integral (with clamping)
        self.i_term += self.ki * error * dt
        self.i_term = clamp(self.i_term, -self.imax, self.imax)
        return p + self.i_term


@dataclass
class Params:
    port: Optional[str]
    baud: int
    target_alt: float
    hover_pwm: int
    kp: float
    ki: float
    imax: float
    gps_weight: float
    yaw_lock: bool
    verbose: bool
    quiet: bool
    dry_run: bool
    joystick: bool
    joy_deadzone: float
    bias_on: str
    alt_lpf: float


class Autopilot:
    MANUAL = 0
    TAKEOFF = 1
    HOLD = 2
    LAND = 3

    def __init__(self, msp: Optional[MSPClient], params: Params, logger: logging.Logger):
        self.msp = msp
        self.params = params
        self.log = logger
        self.state = Autopilot.MANUAL
        self.armed = False
        self.last_rc_send = 0.0
        self.last_alt_req = 0.0
        self.last_gps_req = 0.0
        self.last_ui_print = 0.0
        self.last_time = time.time()
        self.bias_done = False

        # RC values
        self.rc = [RC_MID] * RC_CHANNELS_COUNT
        self.rc[THROTTLE_IDX] = RC_MIN
        self.rc[AUX1_IDX] = DISARM_VALUE
        if params.yaw_lock:
            self.rc[YAW_IDX] = RC_MID

        # Altitude fusion
        self.baro_alt0_cm: Optional[float] = None
        self.gps_alt0_cm: Optional[float] = None
        self.alt_filt_m: Optional[float] = None
        self.manual_override: bool = False  # latched full-manual control via joystick
        self.altitude_assist: bool = False  # joystick-triggered altitude hold (autopilot controls throttle only)
        self.controller = PIController(params.kp, params.ki, params.imax)
        self.target_alt_m = params.target_alt

    # --- Utilities ---
    def _now(self) -> float:
        return time.time()

    def set_bias(self, reason: str = "startup"):
        if not self.msp:
            return
        if self.msp.altitude_cm is not None:
            self.baro_alt0_cm = float(self.msp.altitude_cm)
        if self.msp.gps_alt_cm is not None:
            self.gps_alt0_cm = float(self.msp.gps_alt_cm)
        self.alt_filt_m = 0.0
        self.controller.reset()
        self.log.info(f"Zeroed altitude bias ({reason}).")

    def _alt_m(self) -> Optional[float]:
        # Relative altitude from baro and GPS baselines
        baro_m = None
        gps_m = None
        if self.msp and self.msp.altitude_cm is not None:
            if self.baro_alt0_cm is None:
                # If no bias captured yet, treat current as zero (will be set explicitly by set_bias)
                self.baro_alt0_cm = float(self.msp.altitude_cm)
            baro_m = (self.msp.altitude_cm - self.baro_alt0_cm) / 100.0
        if self.msp and self.msp.gps_alt_cm is not None:
            if self.gps_alt0_cm is None:
                self.gps_alt0_cm = float(self.msp.gps_alt_cm)
            gps_m = (self.msp.gps_alt_cm - self.gps_alt0_cm) / 100.0
        w = clamp(self.params.gps_weight, 0.0, 1.0)
        fused = None
        if baro_m is None and gps_m is None:
            fused = None
        elif baro_m is None:
            fused = gps_m
        elif gps_m is None:
            fused = baro_m
        else:
            fused = (1.0 - w) * baro_m + w * gps_m
        # Smoothing
        if fused is None:
            return None
        alpha = clamp(self.params.alt_lpf, 0.0, 1.0)
        if self.alt_filt_m is None or alpha <= 0.0:
            self.alt_filt_m = fused
        else:
            self.alt_filt_m = self.alt_filt_m + alpha * (fused - self.alt_filt_m)
        return self.alt_filt_m

    def _alt_valid(self) -> bool:
        if not self.msp:
            return False
        return (self._now() - self.msp.alt_timestamp) < SENSOR_TIMEOUT and self.msp.altitude_cm is not None

    def _send_rc(self):
        if self.params.dry_run or not self.msp:
            return
        payload = b"".join(struct.pack("<H", int(clamp(v, RC_MIN, RC_MAX))) for v in self.rc[:RC_CHANNELS_COUNT])
        self.msp.send_msp(MSP_SET_RAW_RC, payload)

    # --- Public Controls ---
    def arm(self):
        if self.armed:
            return
        if self.rc[THROTTLE_IDX] > RC_MIN + 50:
            self.log.warning("Arm blocked: throttle must be low.")
            return
        self.rc[AUX1_IDX] = ARM_VALUE
        self.armed = True
        if self.params.bias_on == "arm" and self._alt_valid():
            self.set_bias("arm")
        self.log.info("ARMED")

    def disarm(self):
        if not self.armed:
            return
        self.rc[AUX1_IDX] = DISARM_VALUE
        self.rc[THROTTLE_IDX] = RC_MIN
        self.armed = False
        self.state = Autopilot.MANUAL
        self.controller.reset()
        self.log.info("DISARMED")

    def takeoff(self, target_alt_m: Optional[float] = None):
        if target_alt_m is not None:
            self.target_alt_m = target_alt_m
        if not self.armed:
            self.log.warning("Cannot TAKEOFF: not armed")
            return
        if not self._alt_valid():
            self.log.warning("Cannot TAKEOFF: no valid altitude")
            return
        if self.params.bias_on == "takeoff" and self._alt_valid():
            self.set_bias("takeoff")
        self.state = Autopilot.TAKEOFF
        self.controller.reset()
        self.log.info(f"TAKEOFF to {self.target_alt_m:.2f} m")

    def hold_here(self):
        if not self.armed:
            self.log.warning("Cannot HOLD: not armed")
            return
        alt = self._alt_m()
        if alt is None:
            self.log.warning("Cannot HOLD: no valid altitude")
            return
        self.target_alt_m = alt
        self.state = Autopilot.HOLD
        self.controller.reset()
        self.log.info(f"HOLD @ {self.target_alt_m:.2f} m")

    def land(self):
        if not self.armed:
            self.log.warning("Cannot LAND: not armed")
            return
        self.state = Autopilot.LAND
        self.log.info("LAND")

    def nudge_target(self, delta_m: float):
        self.target_alt_m = clamp(self.target_alt_m + delta_m, -1.0, 30.0)
        if self.state in (Autopilot.HOLD, Autopilot.TAKEOFF):
            self.log.info(f"Target altitude: {self.target_alt_m:.2f} m")

    # --- Core loop ---
    def step(self):
        now = self._now()
        dt = now - self.last_time
        if dt < 0:
            dt = 0
        self.last_time = now

        # Periodic MSP requests
        if self.msp:
            if now - self.last_alt_req >= ALT_REQ_INTERVAL:
                self.msp.request(MSP_ALTITUDE)
                self.last_alt_req = now
            if now - self.last_gps_req >= GPS_REQ_INTERVAL:
                self.msp.request(MSP_RAW_GPS)
                self.last_gps_req = now
            self.msp.read_and_parse()

        # Smart bias at startup when data becomes valid
        if (not self.bias_done) and self.params.bias_on == "startup" and self._alt_valid():
            self.set_bias("startup")
            self.bias_done = True

        # Yaw lock and neutral attitude (skip when joystick/manual or altitude assist)
        if self.params.yaw_lock and not (self.manual_override or self.altitude_assist):
            self.rc[YAW_IDX] = RC_MID
        if not (self.manual_override or self.altitude_assist):
            self.rc[ROLL_IDX] = RC_MID
            self.rc[PITCH_IDX] = RC_MID

        # State machine
        if not self.armed:
            self.state = Autopilot.MANUAL
            self.rc[THROTTLE_IDX] = RC_MIN
        else:
            alt_m = self._alt_m()
            # Handle missing altitude mid-flight
            if alt_m is None:
                # Conservative behavior: hold near hover PWM but do not ascend aggressively
                self.rc[THROTTLE_IDX] = clamp(self.params.hover_pwm - 50, RC_MIN, RC_MAX)
            else:
                if self.altitude_assist:
                    # Autopilot controls throttle to maintain target altitude; pilot controls R/P/Y
                    error = (self.target_alt_m - alt_m)
                    correction = self.controller.update(error, dt)
                    commanded = self.params.hover_pwm + correction
                    self.rc[THROTTLE_IDX] = int(clamp(commanded, RC_MIN, RC_MAX))
                    # Reflect state as HOLD for UI
                    if self.state != Autopilot.HOLD:
                        self.state = Autopilot.HOLD
                elif self.manual_override:
                    # In manual override, reflect MANUAL state and keep joystick throttle
                    if self.state != Autopilot.MANUAL:
                        self.state = Autopilot.MANUAL
                        self.controller.reset()
                    # Do not modify RC here; joystick already set throttle/attitude
                    pass
                elif self.state == Autopilot.TAKEOFF:
                    # PI control towards target; allow slight extra ceiling during takeoff
                    error = (self.target_alt_m - alt_m)
                    correction = self.controller.update(error, dt)
                    commanded = self.params.hover_pwm + correction
                    # Allow a modest ceiling to ensure liftoff
                    commanded = clamp(commanded, RC_MIN, min(RC_MAX, self.params.hover_pwm + 200))
                    self.rc[THROTTLE_IDX] = int(commanded)
                    if error <= 0.15:
                        # Reached target -> HOLD
                        self.state = Autopilot.HOLD
                        self.controller.reset()
                        self.log.info("Reached target; HOLD")
                elif self.state == Autopilot.HOLD:
                    error = (self.target_alt_m - alt_m)
                    correction = self.controller.update(error, dt)
                    commanded = self.params.hover_pwm + correction
                    self.rc[THROTTLE_IDX] = int(clamp(commanded, RC_MIN, RC_MAX))
                elif self.state == Autopilot.LAND:
                    # Target is 0 m; reduce throttle with gentle descent
                    landing_target = 0.0
                    error = (landing_target - alt_m)
                    correction = self.controller.update(error, dt) * 0.5  # softer integral during land
                    base = self.params.hover_pwm - 80  # slight below-hover bias
                    commanded = base + correction
                    # As we get very low, reduce further
                    if alt_m < 0.25:
                        commanded = min(commanded, RC_MIN + 50)
                    self.rc[THROTTLE_IDX] = int(clamp(commanded, RC_MIN, RC_MAX))
                    if alt_m < 0.12:
                        # Cut and disarm when very close to ground
                        self.rc[THROTTLE_IDX] = RC_MIN
                        self.disarm()
                else:
                    # MANUAL (but we still keep neutral attitude)
                    if not self.manual_override:
                        self.rc[THROTTLE_IDX] = RC_MIN

        # Send RC at fixed rate
        if now - self.last_rc_send >= RC_SEND_INTERVAL:
            self._send_rc()
            self.last_rc_send = now

        # Status UI
        if (not self.params.quiet) and (now - self.last_ui_print >= UI_PRINT_INTERVAL):
            alt = self._alt_m()
            alt_str = f"{alt:.2f}m" if alt is not None else "N/A"
            state_names = {0: "MANUAL", 1: "TAKEOFF", 2: "HOLD", 3: "LAND"}
            print(
                f"State={state_names.get(self.state,'?'):<7} | Armed={'Y' if self.armed else 'N'} | Alt={alt_str:<6} | Target={self.target_alt_m:.2f}m | Thr={self.rc[THROTTLE_IDX]} | OVR={'Y' if self.manual_override else 'N'} | ALT_ASS={'Y' if self.altitude_assist else 'N'}",
                end="\r",
                flush=True,
            )
            self.last_ui_print = now


class KeyHandler:
    def __init__(self, on_key):
        self.on_key = on_key
        self.stop_evt = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_evt.set()
        try:
            self.thread.join(timeout=1.0)
        except Exception:
            pass

    def _run(self):
        if WINDOWS and msvcrt is not None:
            while not self.stop_evt.is_set():
                if msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    self.on_key(ch)
                else:
                    time.sleep(0.01)
        else:
            # POSIX fallback (raw mode, non-blocking)
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                while not self.stop_evt.is_set():
                    r, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if r:
                        ch = sys.stdin.read(1)
                        self.on_key(ch)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class JoystickInput:
    AXIS_YAW = 0
    AXIS_THROTTLE = 1
    AXIS_ROLL = 3
    AXIS_PITCH = 4
    BUTTON_ARM_DISARM = 4
    BUTTON_ALT_HOLD = 5

    def __init__(self, deadzone: float, logger: logging.Logger):
        self.deadzone = max(0.0, min(0.49, deadzone))
        self.log = logger
        self.enabled = False
        self.joy = None

    def start(self):
        if not PYGAME_AVAILABLE:
            self.log.warning("--joystick requested but pygame is not installed. Disabling joystick.")
            return
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joy = pygame.joystick.Joystick(0)
                self.joy.init()
                self.enabled = True
                self.log.info(f"Joystick '{self.joy.get_name()}' connected.")
            else:
                self.log.error("No joystick detected. Run without --joystick or connect a controller.")
                self.enabled = False
        except Exception as e:
            self.log.error(f"Joystick init failed: {e}")
            self.enabled = False

    def stop(self):
        try:
            if PYGAME_AVAILABLE:
                pygame.quit()
        except Exception:
            pass

    def _map_axis(self, v: float, inverted: bool = False) -> int:
        if abs(v) < self.deadzone:
            v = 0.0
        if inverted:
            v = -v
        normalized = (v + 1.0) / 2.0
        return int(clamp(RC_MIN + normalized * (RC_MAX - RC_MIN), RC_MIN, RC_MAX))

    def poll(self, ap: "Autopilot"):
        if not self.enabled or self.joy is None:
            return
        # Process events (buttons, etc.)
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                # Any button press indicates pilot interaction
                if event.button == self.BUTTON_ARM_DISARM:
                    ap.manual_override = True
                    if ap.armed:
                        ap.disarm()
                    else:
                        ap.arm()
                elif event.button == self.BUTTON_ALT_HOLD:
                    if not ap.armed:
                        self.log.warning("ALT-ASSIST request ignored: not armed")
                    elif not ap._alt_valid():
                        self.log.warning("ALT-ASSIST request ignored: no valid altitude")
                    else:
                        ap.manual_override = False
                        ap.altitude_assist = not ap.altitude_assist
                        if ap.altitude_assist:
                            alt = ap._alt_m()
                            if alt is not None:
                                ap.target_alt_m = alt
                            ap.state = Autopilot.HOLD
                            ap.controller.reset()
                            self.log.info("ALT-ASSIST ON (holding current altitude)")
                        else:
                            ap.manual_override = True  # fall back to manual throttle
                            self.log.info("ALT-ASSIST OFF (manual)")
        # Read axes continuously
        try:
            yaw_v = self.joy.get_axis(self.AXIS_YAW)
            thr_v = self.joy.get_axis(self.AXIS_THROTTLE)
            roll_v = self.joy.get_axis(self.AXIS_ROLL)
            pitch_v = self.joy.get_axis(self.AXIS_PITCH)
        except Exception:
            return
        # Detect pilot stick activity
        if any(abs(v) >= self.deadzone for v in (yaw_v, roll_v, pitch_v)):
            ap.manual_override = True

        # Throttle stick: if in ALT-ASSIST and pilot moves throttle, exit assist and enter manual
        if abs(thr_v) >= self.deadzone:
            if ap.altitude_assist:
                ap.altitude_assist = False
                ap.manual_override = True
                self.log.info("ALT-ASSIST cancelled by throttle movement -> MANUAL")
            ap.manual_override = True

        # Apply mappings
        if ap.altitude_assist:
            # Pilot controls attitude and yaw; autopilot keeps throttle
            ap.rc[ROLL_IDX] = self._map_axis(roll_v)
            ap.rc[PITCH_IDX] = self._map_axis(pitch_v, inverted=True)
            ap.rc[YAW_IDX] = self._map_axis(yaw_v)
        elif ap.manual_override:
            # Full manual mapping including throttle
            ap.rc[THROTTLE_IDX] = self._map_axis(thr_v, inverted=True)
            ap.rc[ROLL_IDX] = self._map_axis(roll_v)
            ap.rc[PITCH_IDX] = self._map_axis(pitch_v, inverted=True)
            ap.rc[YAW_IDX] = self._map_axis(yaw_v)


def build_logger(verbose: bool, quiet: bool) -> logging.Logger:
    lvl = logging.INFO
    if verbose and not quiet:
        lvl = logging.DEBUG
    if quiet:
        lvl = logging.WARNING
    logger = logging.getLogger("autopilot")
    logger.setLevel(lvl)
    h = logging.StreamHandler(sys.stdout)
    h.setLevel(lvl)
    fmt = logging.Formatter("[%(levelname)s] %(message)s")
    h.setFormatter(fmt)
    logger.handlers.clear()
    logger.addHandler(h)
    return logger


def parse_args() -> Params:
    p = argparse.ArgumentParser(description="Betaflight MSP Autopilot (Altitude Hold)")
    p.add_argument("--port", type=str, default=None, help="Serial port (auto-detect if omitted)")
    p.add_argument("--baud", type=int, default=BAUD_RATE, help="Baud rate (default 115200)")
    p.add_argument("--target-alt", type=float, default=DEFAULT_TARGET_ALT, help="Default target altitude for takeoff (m)")
    p.add_argument("--hover-pwm", type=int, default=DEFAULT_HOVER_PWM, help="Estimated hover PWM (1000-2000)")
    p.add_argument("--kp", type=float, default=DEFAULT_KP, help="PI Kp (PWM/m)")
    p.add_argument("--ki", type=float, default=DEFAULT_KI, help="PI Ki (PWM/(m*s))")
    p.add_argument("--imax", type=float, default=DEFAULT_IMAX, help="Integral clamp (abs PWM)")
    p.add_argument("--gps-weight", type=float, default=0.0, help="Weight of GPS altitude in fusion [0..1] (baro primary)")
    p.add_argument("--no-yaw-lock", action="store_true", help="Do not lock yaw to center")
    p.add_argument("--joystick", action="store_true", default=True, help="Enable joystick manual override (pygame)")
    p.add_argument("--joy-deadzone", type=float, default=0.08, help="Joystick deadzone (0..0.49)")
    p.add_argument("--bias-on", type=str, choices=["startup", "arm", "takeoff", "never"], default="startup", help="When to zero altitude bias")
    p.add_argument("--alt-lpf", type=float, default=0.2, help="Altitude smoothing factor (alpha 0..1)")
    p.add_argument("-v", "--verbose", action="store_true", help="Verbose logs")
    p.add_argument("-q", "--quiet", action="store_true", help="Quiet logs (warnings only)")
    p.add_argument("--dry-run", action="store_true", help="Run without serial (no RC output)")
    args = p.parse_args()

    return Params(
        port=args.port,
        baud=args.baud,
        target_alt=args.target_alt,
        hover_pwm=int(clamp(args.hover_pwm, RC_MIN, RC_MAX)),
        kp=args.kp,
        ki=args.ki,
        imax=args.imax,
        gps_weight=args.gps_weight,
        yaw_lock=not args.no_yaw_lock,
        verbose=args.verbose,
        quiet=args.quiet,
        dry_run=args.dry_run,
        joystick=args.joystick,
        joy_deadzone=args.joy_deadzone,
        bias_on=args.bias_on,
        alt_lpf=args.alt_lpf,
    )


def main():
    params = parse_args()
    logger = build_logger(params.verbose, params.quiet)

    logger.info("Autopilot starting...")
    if params.dry_run:
        logger.warning("DRY RUN: not opening serial, not sending RC")
        msp = None
    else:
        try:
            msp = MSPClient(params.port, params.baud, auto_detect=True, logger=logger)
        except Exception as e:
            logger.error(f"Serial init failed: {e}")
            sys.exit(1)

    ap = Autopilot(msp, params, logger)

    # Graceful quit event
    quit_evt = threading.Event()

    # Key bindings help
    print()
    print("Controls: a=Arm/Disarm, t=Takeoff, h=Hold here, l=Land, +/- or w/s=Alt setpoint, z=Zero bias, q=Quit")
    if params.joystick:
        print("Joystick: any stick/button latches MANUAL override; Button L1/LB toggles Arm/Disarm")

    def on_key(ch: str):
        ch = ch.lower()
        if ch == "a":
            if ap.armed:
                ap.disarm()
            else:
                ap.arm()
        elif ch == "t":
            ap.manual_override = False
            ap.takeoff(params.target_alt)
        elif ch == "h":
            ap.manual_override = False
            ap.hold_here()
        elif ch == "l":
            ap.manual_override = False
            ap.land()
        elif ch in ["+", "="] or ch == "w":
            ap.nudge_target(+0.1)
        elif ch in ["-", "_"] or ch == "s":
            ap.nudge_target(-0.1)
        elif ch == "z":
            if ap._alt_valid():
                ap.set_bias("manual")
            else:
                logger.warning("Cannot zero bias: no valid altitude yet")
        elif ch == "q":
            logger.info("Quit requested")
            quit_evt.set()
        # ignore others

    kh = KeyHandler(on_key)
    kh.start()

    js = None
    if params.joystick:
        js = JoystickInput(params.joy_deadzone, logger)
        js.start()

    try:
        while not quit_evt.is_set():
            if js is not None:
                js.poll(ap)
            ap.step()
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        kh.stop()
        if js is not None:
            js.stop()
        # Try to shutdown safely
        try:
            ap.rc[THROTTLE_IDX] = RC_MIN
            ap.rc[AUX1_IDX] = DISARM_VALUE
            ap._send_rc()
            time.sleep(0.2)
        except Exception:
            pass
        if msp is not None:
            msp.close()
        print("\nAutopilot stopped.")


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
Autonomous Drone Controller for Raspberry Pi + Betaflight (MSP)

- Connects to Betaflight via MSP over serial (SpeedyBee F405 AIO)
- Provides a high-level API: arm/disarm, takeoff, hold (alt/pos), goto with secure itinerary, land
- Implements controllers for altitude, heading, and GPS position (Angle mode assumed)

Assumptions:
- FC runs Betaflight in Angle mode (stick commands map to target angles around 1500µs)
- RC channels: 0=Roll, 1=Pitch, 2=Throttle, 3=Yaw, 4=AUX1 (Arm), 5=AUX2
- Altitude via MSP_ALTITUDE (cm), GPS via MSP_RAW_GPS, Heading via MSP_ATTITUDE

Safety:
- Geofence (radius from home)
- Refuse goto without GPS fix and sufficient satellites
- Graceful degradation if sensors are missing

CLI examples:
  python3 autonomous_drone.py --arm
  python3 autonomous_drone.py --takeoff 1.5
  python3 autonomous_drone.py --goto 48.85837 2.29448 --alt 5 --safe-alt 8 --geofence 200
  python3 autonomous_drone.py --land

Dependency: pip install pyserial
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial. Install with 'pip install pyserial'")
    raise

# ------------------------
# MSP Constants
# ------------------------
BAUD_RATE = 115200
MSP_SET_RAW_RC = 200
MSP_ALTITUDE = 109
MSP_RAW_GPS = 106
MSP_ATTITUDE = 108

RC_CHANNELS_COUNT = 8
PWM_MIN = 1000
PWM_MAX = 2000
PWM_MID = 1500

AUX_ARM_CH = 4          # AUX1 index
AUX_ARM_HIGH = 1800
AUX_ARM_LOW = 1000

THROTTLE_SAFETY_ARM_OFFSET = 50  # throttle must be <= (min + offset) to arm

# ------------------------
# Ultrasonic Fusion Config
# ------------------------
# Enable/disable ultrasonic fusion by default (can be overridden by CLI)
ULTRA_DEFAULT_ENABLED = True
# Valid measurement range (meters)
ULTRA_MIN_VALID_M = 0.02
ULTRA_MAX_VALID_M = 1.00
# Blending region around the top end of ultrasonic range for smooth transition
# w_ultra = 1 below BLEND_LOW, 0 above BLEND_HIGH, linearly in between
ULTRA_BLEND_LOW_M = 0.80
ULTRA_BLEND_HIGH_M = 1.20
# Ground calibration: when ultrasonic < this, adapt baro bias toward ultrasonic
ULTRA_GROUND_CALIB_M = 0.15
# Smoothing factors
FUSED_ALT_LPF_ALPHA = 0.10      # low-pass on fused altitude
BARO_BIAS_ALPHA = 0.02          # slow adaptation of baro bias near ground

# ------------------------
# Data Models
# ------------------------
@dataclass
class DroneState:
    # Sensors
    altitude_m: Optional[float] = None            # fused altitude (meters)
    # Raw/aux signals
    baro_alt_m_raw: Optional[float] = None        # raw from MSP_ALTITUDE (m)
    baro_bias_m: float = 0.0                      # bias subtracted from raw
    ultrasonic_alt_m: Optional[float] = None      # from ultrasonic (m)
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    gps_fix: bool = False
    gps_num_sat: int = 0
    gps_speed_mps: float = 0.0
    gps_course_deg: float = 0.0
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    yaw_deg: Optional[float] = None

    # Home
    home_lat_deg: Optional[float] = None
    home_lon_deg: Optional[float] = None
    home_set: bool = False

    # Timestamps
    last_alt_update: float = 0.0
    last_ultra_update: float = 0.0
    last_gps_update: float = 0.0
    last_att_update: float = 0.0

    # Internals
    _altitude_m_prev: Optional[float] = None      # fused previous (for vspeed)
    _altitude_vspeed_mps: float = 0.0
    _altitude_lpf: Optional[float] = None         # fused LPF store

    def update_altitude(self, alt_m: float):
        """Backward-compatible: treat as barometric update from MSP."""
        self.update_baro_alt(alt_m)

    def update_baro_alt(self, alt_m: float):
        now = time.time()
        # store raw and corrected
        self.baro_alt_m_raw = alt_m
        baro_corr = alt_m - self.baro_bias_m
        # fuse with ultrasonic if available
        fused = self._fuse_altitude(baro_corr, self.ultrasonic_alt_m)
        self._update_fused(fused, now)

    def update_ultrasonic(self, ultra_m: Optional[float]):
        now = time.time()
        # sanitize ultrasonic reading
        if ultra_m is not None and (ULTRA_MIN_VALID_M <= ultra_m <= ULTRA_MAX_VALID_M):
            self.ultrasonic_alt_m = float(ultra_m)
            self.last_ultra_update = now
            # Opportunistic baro bias calibration near ground
            if self.baro_alt_m_raw is not None and self.ultrasonic_alt_m < ULTRA_GROUND_CALIB_M:
                target_bias = self.baro_alt_m_raw - self.ultrasonic_alt_m
                self.baro_bias_m = (1.0 - BARO_BIAS_ALPHA) * self.baro_bias_m + BARO_BIAS_ALPHA * target_bias
        else:
            # out of range -> ignore but still fuse using baro
            self.ultrasonic_alt_m = None
        # recompute fused using latest values
        baro_corr = None if self.baro_alt_m_raw is None else (self.baro_alt_m_raw - self.baro_bias_m)
        fused = self._fuse_altitude(baro_corr, self.ultrasonic_alt_m)
        self._update_fused(fused, now)

    def _fuse_altitude(self, baro_corr: Optional[float], ultra: Optional[float]) -> Optional[float]:
        # Choose sensor or blend smoothly in top-end of ultrasonic range
        if baro_corr is None and ultra is None:
            return None
        if ultra is None:
            return baro_corr
        if baro_corr is None:
            return ultra
        # blending weight based on ultrasonic reading
        # 1.0 below BLEND_LOW, 0.0 above BLEND_HIGH
        u = float(ultra)
        if u <= ULTRA_BLEND_LOW_M:
            w_u = 1.0
        elif u >= ULTRA_BLEND_HIGH_M:
            w_u = 0.0
        else:
            w_u = (ULTRA_BLEND_HIGH_M - u) / (ULTRA_BLEND_HIGH_M - ULTRA_BLEND_LOW_M)
        return w_u * u + (1.0 - w_u) * float(baro_corr)

    def _update_fused(self, fused: Optional[float], now: float):
        if fused is None:
            # do not change altitude_m but update timestamp if we had any sensor update
            self.last_alt_update = now
            return
        # vertical speed from fused
        if self._altitude_m_prev is not None:
            dt = max(1e-3, now - self.last_alt_update)
            vs = (fused - self._altitude_m_prev) / dt
            alpha_vs = 0.3
            self._altitude_vspeed_mps = (1 - alpha_vs) * self._altitude_vspeed_mps + alpha_vs * vs
        self._altitude_m_prev = fused
        # low-pass fused altitude for stability
        if self._altitude_lpf is None:
            self._altitude_lpf = fused
        else:
            self._altitude_lpf = (1.0 - FUSED_ALT_LPF_ALPHA) * self._altitude_lpf + FUSED_ALT_LPF_ALPHA * fused
        self.altitude_m = self._altitude_lpf
        self.last_alt_update = now

    def set_home_if_needed(self):
        if not self.home_set and self.gps_fix and self.lat_deg is not None and self.lon_deg is not None:
            self.home_lat_deg = self.lat_deg
            self.home_lon_deg = self.lon_deg
            self.home_set = True

# ------------------------
# Helpers
# ------------------------

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def meters_per_deg_lat(lat_deg: float) -> float:
    return 111132.92 - 559.82 * math.cos(2 * math.radians(lat_deg)) + 1.175 * math.cos(4 * math.radians(lat_deg))


def meters_per_deg_lon(lat_deg: float) -> float:
    return 111412.84 * math.cos(math.radians(lat_deg)) - 93.5 * math.cos(3 * math.radians(lat_deg))


def bearing_deg(from_lat: float, from_lon: float, to_lat: float, to_lon: float) -> float:
    # Bearing 0..360
    y = math.sin(math.radians(to_lon - from_lon)) * math.cos(math.radians(to_lat))
    x = math.cos(math.radians(from_lat)) * math.sin(math.radians(to_lat)) - math.sin(math.radians(from_lat)) * math.cos(math.radians(to_lat)) * math.cos(math.radians(to_lon - from_lon))
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0


# ------------------------
# MSP Client
# ------------------------
class MspClient:
    def __init__(self, port: Optional[str] = None, baud: int = BAUD_RATE,
                 use_ultrasonic: bool = ULTRA_DEFAULT_ENABLED,
                 ultra_trigger_pin: int = 23,
                 ultra_echo_pin: int = 24,
                 ultra_max_distance_m: float = 1.2):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self._rx_buf = bytearray()
        self.state = DroneState()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        # polling intervals (s)
        self.alt_interval = 0.05
        self.gps_interval = 0.5
        self.att_interval = 0.05
        self.ultra_interval = 0.05
        self._last_alt_req = 0.0
        self._last_gps_req = 0.0
        self._last_att_req = 0.0
        self._last_ultra_poll = 0.0
        # RC state
        self.rc_values: List[int] = [PWM_MID] * RC_CHANNELS_COUNT
        self.rc_values[2] = PWM_MIN  # throttle low
        self.rc_values[AUX_ARM_CH] = AUX_ARM_LOW  # disarmed
        # lock for rc/state
        self._lock = threading.Lock()
        # Ultrasonic
        self._ultra: Optional[UltrasonicReader] = None
        self._ultra_enabled = use_ultrasonic
        self._ultra_trigger_pin = ultra_trigger_pin
        self._ultra_echo_pin = ultra_echo_pin
        self._ultra_max_distance_m = ultra_max_distance_m

    # ----- Serial -----
    def _open_serial(self) -> serial.Serial:
        if self.port:
            return serial.Serial(self.port, self.baud, timeout=0)
        # Try common ports
        ports = [
            '/dev/ttyAMA0',
            '/dev/ttyS0',
            '/dev/ttyACM0',
            '/dev/ttyUSB0',
        ]
        last_err = None
        for p in ports:
            try:
                print(f"[MSP] Trying port {p}...")
                s = serial.Serial(p, self.baud, timeout=0)
                print(f"[MSP] Connected on {p}")
                return s
            except Exception as e:
                last_err = e
        raise RuntimeError(f"Unable to open serial port. Last error: {last_err}")

    # ----- MSP framing -----
    @staticmethod
    def _checksum(payload: bytes) -> int:
        chk = 0
        for b in payload:
            chk ^= b
        return chk

    def _send_msp(self, cmd: int, data: Optional[bytes] = None):
        if data is None:
            data = b''
        header = b'$M<'
        payload_header = struct.pack('<BB', len(data), cmd)
        payload = payload_header + data
        chk = self._checksum(payload)
        pkt = header + payload + struct.pack('<B', chk)
        try:
            self.ser.write(pkt)
        except Exception as e:
            print(f"[MSP] Serial write error: {e}")

    def _request(self, cmd: int):
        self._send_msp(cmd, None)

    def _parse_rx(self):
        # parse packets from _rx_buf
        buf = self._rx_buf
        while True:
            idx = buf.find(b'$M>')
            if idx == -1:
                # keep tail
                self._rx_buf = buf[-3:]
                return
            if len(buf) < idx + 5:
                self._rx_buf = buf[idx:]
                return
            size = buf[idx + 3]
            cmd = buf[idx + 4]
            if len(buf) < idx + 5 + size + 1:
                self._rx_buf = buf[idx:]
                return
            payload = buf[idx + 5: idx + 5 + size]
            recv_chk = buf[idx + 5 + size]
            calc_chk = self._checksum(buf[idx + 3: idx + 5 + size])
            # advance
            buf = buf[idx + 6 + size:]
            if recv_chk != calc_chk:
                continue
            # decode
            self._handle_cmd(cmd, payload)
        # not reached

    def _handle_cmd(self, cmd: int, payload: bytes):
        with self._lock:
            if cmd == MSP_ALTITUDE and len(payload) >= 4:
                alt_cm = struct.unpack('<i', payload[0:4])[0]
                self.state.update_baro_alt(float(alt_cm) / 100.0)
            elif cmd == MSP_RAW_GPS and len(payload) >= 16:
                fix = payload[0] != 0
                sats = payload[1]
                lat_e7 = struct.unpack('<i', payload[2:6])[0]
                lon_e7 = struct.unpack('<i', payload[6:10])[0]
                alt_m = struct.unpack('<h', payload[10:12])[0]
                speed_cms = struct.unpack('<h', payload[12:14])[0]
                course_deg10 = struct.unpack('<h', payload[14:16])[0]
                self.state.gps_fix = fix
                self.state.gps_num_sat = int(sats)
                if lat_e7 != 0 and lon_e7 != 0:
                    self.state.lat_deg = lat_e7 / 1e7
                    self.state.lon_deg = lon_e7 / 1e7
                    self.state.set_home_if_needed()
                # altitude from GPS (fallback only)
                if self.state.altitude_m is None and alt_m != 0:
                    self.state.update_altitude(float(alt_m))
                self.state.gps_speed_mps = max(0.0, speed_cms) / 100.0
                self.state.gps_course_deg = (course_deg10 / 10.0) % 360.0
                self.state.last_gps_update = time.time()
            elif cmd == MSP_ATTITUDE and len(payload) >= 6:
                roll = struct.unpack('<h', payload[0:2])[0] / 10.0
                pitch = struct.unpack('<h', payload[2:4])[0] / 10.0
                yaw = struct.unpack('<h', payload[4:6])[0] / 10.0
                self.state.roll_deg = roll
                self.state.pitch_deg = pitch
                # Betaflight yaw often [-180,180]. Normalize 0..360 for convenience
                self.state.yaw_deg = (yaw + 360.0) % 360.0
                self.state.last_att_update = time.time()

    # ----- Lifecycle -----
    def start(self):
        self.ser = self._open_serial()
        self._stop_evt.clear()
        # Try to initialize ultrasonic reader (optional)
        if self._ultra_enabled and self._ultra is None:
            try:
                self._ultra = UltrasonicReader(trigger_pin=self._ultra_trigger_pin,
                                               echo_pin=self._ultra_echo_pin,
                                               max_distance_m=self._ultra_max_distance_m)
                print("[MSP] Ultrasonic: enabled")
            except Exception as e:
                print(f"[MSP] Ultrasonic disabled (init failed): {e}")
                self._ultra = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def _loop(self):
        while not self._stop_evt.is_set():
            now = time.time()
            # Periodic requests
            if now - self._last_alt_req > self.alt_interval:
                self._request(MSP_ALTITUDE)
                self._last_alt_req = now
            if now - self._last_gps_req > self.gps_interval:
                self._request(MSP_RAW_GPS)
                self._last_gps_req = now
            if now - self._last_att_req > self.att_interval:
                self._request(MSP_ATTITUDE)
                self._last_att_req = now
            # Poll ultrasonic if available
            if self._ultra is not None and (now - self._last_ultra_poll > self.ultra_interval):
                try:
                    d = self._ultra.read_m()
                except Exception:
                    d = None
                self.state.update_ultrasonic(d)
                self._last_ultra_poll = now
            # Read
            try:
                n = self.ser.in_waiting
                if n > 0:
                    data = self.ser.read(n)
                    if data:
                        self._rx_buf.extend(data)
                        self._parse_rx()
            except Exception:
                pass
            time.sleep(0.005)

    # ----- RC -----
    def send_rc(self, rc_values: List[int]):
        # copy & clamp then send
        with self._lock:
            vals = [int(clamp(v, PWM_MIN, PWM_MAX)) for v in rc_values[:RC_CHANNELS_COUNT]]
            # keep array length consistent
            if len(vals) < RC_CHANNELS_COUNT:
                vals += [PWM_MIN] * (RC_CHANNELS_COUNT - len(vals))
            self.rc_values = list(vals)
        payload = b''.join(struct.pack('<H', v) for v in self.rc_values)
        self._send_msp(MSP_SET_RAW_RC, payload)


# ------------------------
# Ultrasonic Reader (optional)
# ------------------------
class UltrasonicReader:
    """Lightweight wrapper for gpiozero.DistanceSensor.

    Returns distance in meters via read_m(). If gpiozero is not available,
    instantiation will raise so that caller can disable ultrasonic gracefully.
    """
    def __init__(self, trigger_pin: int, echo_pin: int, max_distance_m: float = 1.2):
        try:
            from gpiozero import DistanceSensor  # type: ignore
        except Exception as e:
            raise RuntimeError("gpiozero not available") from e
        self._max_m = max(ULTRA_MAX_VALID_M, float(max_distance_m))
        # gpiozero DistanceSensor.distance -> 0..1 proportion of max_distance
        self._sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin, max_distance=self._max_m)

    def read_m(self) -> Optional[float]:
        try:
            d_ratio = float(self._sensor.distance)  # 0..1 relative to max_distance
        except Exception:
            return None
        d_m = d_ratio * self._max_m
        # sanitize
        if d_m <= 0.0 or d_m > (self._max_m + 0.2):
            return None
        return d_m
    # (UltrasonicReader is polled by MspClient; no background thread here)


# ------------------------
# Manual Controller (Gamepad override)
# ------------------------
class ManualController:
    """Gamepad-driven manual RC override, similar to remote_control.py.

    - Uses pygame joystick to read axes/buttons in a background thread
    - Provides has_control() to indicate when manual should override autopilot
    - Provides get_rc() to return current 8-channel RC values (1000..2000)
    - Gracefully disables itself if pygame or a gamepad is unavailable
    """

    # Axis mapping (matches remote_control.py defaults)
    AXIS_YAW = 0        # left stick X
    AXIS_THROTTLE = 1   # left stick Y (inverted)
    AXIS_ROLL = 3       # right stick X
    AXIS_PITCH = 4      # right stick Y (inverted)

    # Button mapping (common Xbox layout)
    BUTTON_ARM_DISARM = 4  # LB / L1
    BUTTON_ALTHOLD = 3     # Y / Triangle (hold)

    def __init__(self, deadzone: float = 0.08, yaw_locked: bool = True):
        self._dz = float(deadzone)
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._connected = False
        self._active = False
        self.yaw_locked = yaw_locked

        # RC state (8 channels)
        self._rc: List[int] = [PWM_MID] * RC_CHANNELS_COUNT
        self._rc[2] = PWM_MIN  # throttle low
        self._rc[AUX_ARM_CH] = AUX_ARM_LOW
        for i in range(5, RC_CHANNELS_COUNT):
            self._rc[i] = PWM_MIN  # AUX low by default

        # pygame setup (lazy import to keep dependency optional)
        try:
            import pygame  # type: ignore
        except Exception as e:
            raise RuntimeError("pygame not available; manual controller disabled") from e
        self._pg = pygame
        self._pg.init()
        self._pg.joystick.init()
        if self._pg.joystick.get_count() <= 0:
            raise RuntimeError("No gamepad detected; manual controller disabled")
        self._js = self._pg.joystick.Joystick(0)
        self._js.init()
        self._connected = True

        # background thread
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ----- Public API -----
    def has_control(self) -> bool:
        """True when sticks are deflected (or throttle raised) and gamepad connected."""
        return self._connected and self._active

    def get_rc(self) -> List[int]:
        with self._lock:
            return list(self._rc)

    def stop(self):
        self._stop_evt.set()
        try:
            self._thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            # Quit pygame subsystems if we started them
            self._pg.joystick.quit()
            self._pg.quit()
        except Exception:
            pass

    # ----- Internals -----
    def _map_axis_to_rc(self, axis_value: float, min_rc: int = PWM_MIN, max_rc: int = PWM_MAX, inverted: bool = False) -> int:
        v = float(axis_value)
        if abs(v) < self._dz:
            v = 0.0
        if inverted:
            v = -v
        normalized = (v + 1.0) / 2.0
        return int(clamp(min_rc + normalized * (max_rc - min_rc), PWM_MIN, PWM_MAX))

    def _loop(self):
        try:
            while not self._stop_evt.is_set():
                # process events
                for event in self._pg.event.get():
                    if event.type == self._pg.JOYAXISMOTION:
                        if event.axis == self.AXIS_YAW:
                            if not self.yaw_locked:
                                with self._lock:
                                    self._rc[3] = self._map_axis_to_rc(event.value)
                        elif event.axis == self.AXIS_THROTTLE:
                            with self._lock:
                                self._rc[2] = self._map_axis_to_rc(event.value, PWM_MIN, PWM_MAX, inverted=True)
                        elif event.axis == self.AXIS_ROLL:
                            with self._lock:
                                self._rc[0] = self._map_axis_to_rc(event.value)
                        elif event.axis == self.AXIS_PITCH:
                            with self._lock:
                                self._rc[1] = self._map_axis_to_rc(event.value, PWM_MIN, PWM_MAX, inverted=True)

                    elif event.type == self._pg.JOYBUTTONDOWN:
                        if event.button == self.BUTTON_ARM_DISARM:
                            with self._lock:
                                # arm only if throttle is low for safety
                                if self._rc[2] <= PWM_MIN + THROTTLE_SAFETY_ARM_OFFSET:
                                    self._rc[AUX_ARM_CH] = AUX_ARM_HIGH if self._rc[AUX_ARM_CH] == AUX_ARM_LOW else AUX_ARM_LOW
                                else:
                                    print("[MANUAL] Refuse arm: throttle too high")
                        elif event.button == self.BUTTON_ALTHOLD:
                            with self._lock:
                                # AUX2 high while held
                                if RC_CHANNELS_COUNT > 5:
                                    self._rc[5] = 1800

                    elif event.type == self._pg.JOYBUTTONUP:
                        if event.button == self.BUTTON_ALTHOLD:
                            with self._lock:
                                if RC_CHANNELS_COUNT > 5:
                                    self._rc[5] = 1000

                    elif event.type == self._pg.JOYDEVICEREMOVED:
                        self._connected = False
                        self._active = False

                    elif event.type == self._pg.JOYDEVICEADDED:
                        # try to reinit
                        try:
                            if self._pg.joystick.get_count() > 0:
                                self._js = self._pg.joystick.Joystick(0)
                                self._js.init()
                                self._connected = True
                        except Exception:
                            pass

                # enforce yaw lock if enabled
                if self.yaw_locked:
                    with self._lock:
                        self._rc[3] = PWM_MID

                # update active flag based on stick deflection
                with self._lock:
                    roll_dev = abs(self._rc[0] - PWM_MID) > 20
                    pitch_dev = abs(self._rc[1] - PWM_MID) > 20
                    yaw_dev = (not self.yaw_locked) and (abs(self._rc[3] - PWM_MID) > 20)
                    thr_active = self._rc[2] > PWM_MIN + 5
                self._active = self._connected and (roll_dev or pitch_dev or yaw_dev or thr_active)

                time.sleep(0.01)
        except Exception as e:
            print(f"[MANUAL] Stopped due to error: {e}")
            self._connected = False
            self._active = False


# ------------------------
# Autopilot
# ------------------------
class Autopilot:
    MODE_IDLE = 0
    MODE_TAKEOFF = 1
    MODE_HOLD = 2
    MODE_GOTO = 3
    MODE_LAND = 4

    def __init__(self, msp: MspClient,
                 angle_limit_deg: float = 45.0,
                 max_tilt_deg: float = 25.0,
                 max_speed_mps: float = 5.0,
                 invert_roll: bool = False,
                 invert_pitch: bool = True,  # match remote_control default feel
                 manual: Optional[ManualController] = None):
        self.msp = msp
        self.angle_limit_deg = max(5.0, min(85.0, angle_limit_deg))
        self.max_tilt_deg = max(5.0, min(self.angle_limit_deg, max_tilt_deg))
        self.max_speed_mps = max(0.5, min(20.0, max_speed_mps))
        self.invert_roll = invert_roll
        self.invert_pitch = invert_pitch
        self.manual = manual

        # RC state
        self.rc: List[int] = [PWM_MID] * RC_CHANNELS_COUNT
        self.rc[2] = PWM_MIN
        self.rc[AUX_ARM_CH] = AUX_ARM_LOW
        for i in range(6, RC_CHANNELS_COUNT):
            self.rc[i] = PWM_MIN

        # control targets
        self.mode = self.MODE_IDLE
        self.target_alt_m: Optional[float] = None
        self.target_yaw_deg: Optional[float] = None
        self.target_latlon: Optional[Tuple[float, float]] = None
        self.safe_alt_m: float = 10.0
        self.geofence_m: Optional[float] = None

        # goto phase
        self._goto_phase: int = 0  # 0=idle, 1=climb, 2=cruise, 3=descend, 4=done

        # altitude control
        self.hover_pwm = 1500
        self.alt_kp = 20.0
        self.alt_ki = 2.0
        self.alt_i = 0.0
        self.alt_pwm_max = 150  # +/- around hover

        # yaw control
        self.yaw_kp = 2.0  # pwm per deg error (via 500/angle_limit scaling)

        # position control
        self.pos_kp_deg_per_m = 0.8  # deg tilt per meter error
        self.vel_kd_deg_per_mps = 0.2  # deg per m/s damping

        # execution
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ----- Public API -----
    def arm(self) -> bool:
        # safety: throttle low
        self.rc[2] = PWM_MIN
        self.msp.send_rc(self.rc)
        time.sleep(0.1)
        # require safety threshold
        if self.rc[2] > PWM_MIN + THROTTLE_SAFETY_ARM_OFFSET:
            print("[AP] Refusing to arm: throttle too high")
            return False
        self.rc[AUX_ARM_CH] = AUX_ARM_HIGH
        self.msp.send_rc(self.rc)
        print("[AP] Arm command sent (AUX1 high)")
        return True

    def disarm(self):
        self.rc[2] = PWM_MIN
        self.rc[AUX_ARM_CH] = AUX_ARM_LOW
        self.msp.send_rc(self.rc)
        print("[AP] Disarm command sent")

    def takeoff(self, alt_m: float):
        self.target_alt_m = max(0.5, alt_m)
        self.mode = self.MODE_TAKEOFF
        print(f"[AP] Takeoff to {self.target_alt_m:.2f} m")

    def hold(self, use_pos: bool = True):
        # lock current altitude and position
        st = self.msp.state
        if st.altitude_m is not None:
            self.target_alt_m = st.altitude_m
        if use_pos and st.gps_fix and st.lat_deg is not None and st.lon_deg is not None:
            self.target_latlon = (st.lat_deg, st.lon_deg)
        else:
            self.target_latlon = None
        if st.yaw_deg is not None:
            self.target_yaw_deg = st.yaw_deg
        self.mode = self.MODE_HOLD
        print("[AP] Hold engaged")

    def goto(self, lat: float, lon: float, target_alt_m: float,
             safe_alt_m: float = 10.0,
             geofence_m: Optional[float] = 200.0):
        self.safe_alt_m = max(1.0, safe_alt_m)
        self.target_alt_m = max(0.5, target_alt_m)
        self.target_latlon = (lat, lon)
        self.geofence_m = geofence_m
        self._goto_phase = 1
        self.mode = self.MODE_GOTO
        print(f"[AP] Goto lat={lat:.6f}, lon={lon:.6f}, target_alt={self.target_alt_m:.1f} m via safe_alt={self.safe_alt_m:.1f} m")

    def land(self):
        self.mode = self.MODE_LAND
        print("[AP] Land engaged")

    # ----- Internals -----
    def _loop(self):
        # RC send rate
        last_send = 0.0
        send_interval = 1.0 / 50.0
        while not self._stop_evt.is_set():
            st = self.msp.state
            # Manual override: if gamepad active, merge sticks but preserve AUX and (by default) throttle
            if self.manual is not None and self.manual.has_control():
                man = self.manual.get_rc()
                # start from current autopilot RC (preserve AUX channels)
                merged = list(self.rc)
                # sticks
                merged[0] = int(clamp(man[0], PWM_MIN, PWM_MAX))  # roll
                merged[1] = int(clamp(man[1], PWM_MIN, PWM_MAX))  # pitch
                # throttle: only override if manual throttle is raised above near-min
                if man[2] > PWM_MIN + 20:
                    merged[2] = int(clamp(man[2], PWM_MIN, PWM_MAX))
                # yaw
                merged[3] = int(clamp(man[3], PWM_MIN, PWM_MAX))
                # Do NOT touch AUX here to avoid accidental disarm on takeover
                self.rc = merged
                now = time.time()
                if now - last_send >= send_interval:
                    self.msp.send_rc(self.rc)
                    last_send = now
                time.sleep(0.01)
                continue
            # default neutral
            roll_cmd = PWM_MID
            pitch_cmd = PWM_MID
            yaw_cmd = PWM_MID
            thr_cmd = self.rc[2]

            # heading target default
            if self.target_yaw_deg is None and st.yaw_deg is not None:
                self.target_yaw_deg = st.yaw_deg

            # mode logic
            if self.mode == self.MODE_IDLE:
                # keep throttle low, neutral sticks
                thr_cmd = PWM_MIN
                roll_cmd = PWM_MID
                pitch_cmd = PWM_MID
                yaw_cmd = PWM_MID

            elif self.mode == self.MODE_TAKEOFF:
                # hold yaw, neutral roll/pitch
                yaw_cmd = self._yaw_hold_cmd(st)
                roll_cmd = PWM_MID
                pitch_cmd = PWM_MID
                # altitude control to climb to target
                thr_cmd = self._altitude_cmd(st)
                # transition when reached
                if st.altitude_m is not None and self.target_alt_m is not None:
                    if st.altitude_m >= self.target_alt_m - 0.1:
                        self.hold(use_pos=False)

            elif self.mode == self.MODE_HOLD:
                yaw_cmd = self._yaw_hold_cmd(st)
                # altitude hold
                thr_cmd = self._altitude_cmd(st)
                # optional position hold
                if self.target_latlon and st.gps_fix and st.lat_deg is not None and st.lon_deg is not None:
                    roll_cmd, pitch_cmd = self._position_cmd(st, self.target_latlon)
                else:
                    roll_cmd = PWM_MID
                    pitch_cmd = PWM_MID

            elif self.mode == self.MODE_GOTO:
                yaw_cmd = self._yaw_hold_cmd(st)
                # enforce geofence
                if self._violates_geofence(st):
                    print("[AP] Geofence breached, switching to HOLD")
                    self.hold(use_pos=True)
                elif not (st.gps_fix and st.lat_deg is not None and st.lon_deg is not None and self.target_latlon):
                    print("[AP] No GPS fix/pos, switching to HOLD")
                    self.hold(use_pos=False)
                else:
                    # phases
                    cur_alt = st.altitude_m if st.altitude_m is not None else 0.0
                    if self._goto_phase == 1:
                        # climb to safe altitude
                        self.target_alt_m = self.safe_alt_m
                        thr_cmd = self._altitude_cmd(st)
                        roll_cmd = PWM_MID
                        pitch_cmd = PWM_MID
                        if cur_alt >= self.safe_alt_m - 0.2:
                            self._goto_phase = 2
                    elif self._goto_phase == 2:
                        # cruise horizontally, hold at safe_alt
                        self.target_alt_m = self.safe_alt_m
                        thr_cmd = self._altitude_cmd(st)
                        roll_cmd, pitch_cmd = self._position_cmd(st, self.target_latlon)
                        # check proximity to target
                        if self._distance_to_target_m(st, self.target_latlon) < 2.0:
                            self._goto_phase = 3
                    elif self._goto_phase == 3:
                        # descend to target_alt and hold position
                        self.target_alt_m = max(0.5, self.target_alt_m or 1.0)
                        thr_cmd = self._altitude_cmd(st)
                        roll_cmd, pitch_cmd = self._position_cmd(st, self.target_latlon)
                        if abs((st.altitude_m or 0.0) - self.target_alt_m) < 0.15 and self._distance_to_target_m(st, self.target_latlon) < 1.0:
                            self._goto_phase = 4
                            print("[AP] Goto reached target")
                            self.hold(use_pos=True)
                    else:
                        self.hold(use_pos=True)

            elif self.mode == self.MODE_LAND:
                yaw_cmd = self._yaw_hold_cmd(st)
                # gentle descend
                if st.altitude_m is not None:
                    self.target_alt_m = 0.0
                    thr_cmd = self._altitude_cmd(st, landing=True)
                    roll_cmd = PWM_MID
                    pitch_cmd = PWM_MID
                    if st.altitude_m < 0.15:
                        self.rc[2] = PWM_MIN
                        self.msp.send_rc(self._compose_rc(PWM_MID, PWM_MID, PWM_MIN, PWM_MID))
                        time.sleep(0.2)
                        self.disarm()
                        self.mode = self.MODE_IDLE
                        print("[AP] Landed and disarmed")
                else:
                    # no altitude, just cut throttle carefully
                    thr_cmd = max(PWM_MIN, self.rc[2] - 5)

            # update rc & send
            self.rc = self._compose_rc(roll_cmd, pitch_cmd, thr_cmd, yaw_cmd)
            now = time.time()
            if now - last_send >= send_interval:
                self.msp.send_rc(self.rc)
                last_send = now

            time.sleep(0.01)

    def stop(self):
        self._stop_evt.set()
        self._thread.join(timeout=1.0)

    # ----- Controllers -----
    def _compose_rc(self, roll: int, pitch: int, thr: int, yaw: int) -> List[int]:
        rc = list(self.rc)
        rc[0] = int(clamp(roll, PWM_MIN, PWM_MAX))
        rc[1] = int(clamp(pitch, PWM_MIN, PWM_MAX))
        rc[2] = int(clamp(thr, PWM_MIN, PWM_MAX))
        rc[3] = int(clamp(yaw, PWM_MIN, PWM_MAX))
        # keep AUX the same
        return rc

    def _altitude_cmd(self, st: DroneState, landing: bool = False) -> int:
        # If no altitude, hold last throttle (or hover)
        if st.altitude_m is None or self.target_alt_m is None:
            return int(clamp(self.hover_pwm, PWM_MIN, PWM_MAX))
        err = self.target_alt_m - st.altitude_m
        # integral with anti-windup
        self.alt_i = clamp(self.alt_i + err * 0.01, -100.0, 100.0)
        adjust = self.alt_kp * err + self.alt_ki * self.alt_i
        adjust = clamp(adjust, -self.alt_pwm_max, self.alt_pwm_max)
        # landing bias to ensure descent
        if landing:
            adjust = clamp(adjust - 50, -self.alt_pwm_max, self.alt_pwm_max)
        cmd = int(clamp(self.hover_pwm + adjust, PWM_MIN, PWM_MAX))
        # hover estimator: slowly adapt so that on average adjust -> 0
        self.hover_pwm = int(clamp(0.999 * self.hover_pwm + 0.001 * cmd, PWM_MIN + 200, PWM_MAX - 200))
        return cmd

    def _yaw_hold_cmd(self, st: DroneState) -> int:
        if st.yaw_deg is None:
            return PWM_MID
        if self.target_yaw_deg is None:
            self.target_yaw_deg = st.yaw_deg
        # shortest error (-180..180)
        err = (self.target_yaw_deg - st.yaw_deg + 540.0) % 360.0 - 180.0
        # map error deg -> angle fraction -> rc
        # We use fraction of angle_limit for proportional yaw; small gain
        frac = clamp(err / self.angle_limit_deg, -1.0, 1.0)
        pwm = int(PWM_MID + 500.0 * self.yaw_kp * frac / 10.0)  # divide to keep modest
        return int(clamp(pwm, PWM_MIN, PWM_MAX))

    def _position_cmd(self, st: DroneState, tgt: Tuple[float, float]) -> Tuple[int, int]:
        lat, lon = st.lat_deg, st.lon_deg
        tlat, tlon = tgt
        if lat is None or lon is None:
            return PWM_MID, PWM_MID
        # position error in N/E meters
        m_per_lat = meters_per_deg_lat(lat)
        m_per_lon = meters_per_deg_lon(lat)
        dN = (tlat - lat) * m_per_lat
        dE = (tlon - lon) * m_per_lon
        # velocity estimate from GPS
        v = st.gps_speed_mps
        bearing = math.radians(st.gps_course_deg if st.gps_course_deg is not None else 0.0)
        vN = v * math.cos(bearing)
        vE = v * math.sin(bearing)
        # desired tilt in deg (north->pitch, east->roll)
        pitch_deg = clamp(-self.pos_kp_deg_per_m * dN - self.vel_kd_deg_per_mps * vN, -self.max_tilt_deg, self.max_tilt_deg)
        roll_deg = clamp(+self.pos_kp_deg_per_m * dE + self.vel_kd_deg_per_mps * vE, -self.max_tilt_deg, self.max_tilt_deg)
        # map to RC
        roll_rc = self._angle_to_rc(roll_deg, invert=self.invert_roll)
        pitch_rc = self._angle_to_rc(pitch_deg, invert=self.invert_pitch)
        return roll_rc, pitch_rc

    def _angle_to_rc(self, angle_deg: float, invert: bool = False) -> int:
        a = -angle_deg if invert else angle_deg
        frac = clamp(a / self.angle_limit_deg, -1.0, 1.0)
        return int(clamp(PWM_MID + 500.0 * frac, PWM_MIN, PWM_MAX))

    def _distance_to_target_m(self, st: DroneState, tgt: Tuple[float, float]) -> float:
        lat, lon = st.lat_deg, st.lon_deg
        tlat, tlon = tgt
        if lat is None or lon is None:
            return 1e9
        m_per_lat = meters_per_deg_lat(lat)
        m_per_lon = meters_per_deg_lon(lat)
        dN = (tlat - lat) * m_per_lat
        dE = (tlon - lon) * m_per_lon
        return math.hypot(dN, dE)

    def _violates_geofence(self, st: DroneState) -> bool:
        if self.geofence_m is None or self.geofence_m <= 0:
            return False
        if not (st.home_set and st.lat_deg is not None and st.lon_deg is not None and st.home_lat_deg is not None and st.home_lon_deg is not None):
            return False
        dist = self._distance_to_target_m(st, (st.home_lat_deg, st.home_lon_deg))
        return dist > self.geofence_m


# ------------------------
# CLI
# ------------------------

def main():
    parser = argparse.ArgumentParser(description="Autonomous controller over MSP (Betaflight Angle mode)")
    parser.add_argument('--port', type=str, default=None, help='Serial port (auto if not set)')
    parser.add_argument('--arm', action='store_true', help='Arm (AUX1 high)')
    parser.add_argument('--disarm', action='store_true', help='Disarm (AUX1 low)')
    parser.add_argument('--takeoff', type=float, default=None, help='Takeoff to altitude (meters)')
    parser.add_argument('--hold', action='store_true', help='Hold current altitude (and position if GPS)')
    parser.add_argument('--goto', nargs=2, type=float, default=None, metavar=('LAT', 'LON'), help='Goto target lat lon')
    parser.add_argument('--alt', type=float, default=None, help='Target altitude for goto (meters)')
    parser.add_argument('--safe-alt', type=float, default=10.0, help='Safe altitude for goto (meters)')
    parser.add_argument('--geofence', type=float, default=20.0, help='Geofence radius from home (meters); set 0 to disable')
    parser.add_argument('--land', action='store_true', help='Land and disarm')
    parser.add_argument('--angle-limit', type=float, default=45.0, help='Angle mode limit (deg) used for RC mapping')
    parser.add_argument('--max-tilt', type=float, default=25.0, help='Max commanded tilt (deg) for position control')
    parser.add_argument('--max-speed', type=float, default=5.0, help='Max ground speed target (m/s) for position control (used in damping)')
    parser.add_argument('--invert-roll', action='store_true', help='Invert roll command mapping')
    parser.add_argument('--invert-pitch', action='store_true', help='Invert pitch command mapping')
    # Ultrasonic options
    parser.add_argument('--no-ultrasonic', action='store_true', help='Disable ultrasonic altitude fusion')
    parser.add_argument('--ultra-trigger', type=int, default=23, help='Ultrasonic trigger pin (BCM)')
    parser.add_argument('--ultra-echo', type=int, default=24, help='Ultrasonic echo pin (BCM)')
    parser.add_argument('--ultra-max', type=float, default=1.2, help='Ultrasonic max distance (m)')
    # Manual control options
    parser.add_argument('--no-manual', action='store_true', help='Disable gamepad manual override')

    args = parser.parse_args()

    msp = MspClient(
        port=args.port,
        use_ultrasonic=not args.no_ultrasonic,
        ultra_trigger_pin=args.ultra_trigger,
        ultra_echo_pin=args.ultra_echo,
        ultra_max_distance_m=args.ultra_max,
    )
    try:
        msp.start()
    except Exception as e:
        print(f"[MSP] Failed to start: {e}")
        sys.exit(1)

    manual = None
    if not args.no_manual:
        try:
            manual = ManualController()
            print("[MAIN] Manual controller enabled (gamepad)")
        except Exception as e:
            print(f"[MAIN] Manual controller disabled: {e}")

    ap = Autopilot(
        msp,
        angle_limit_deg=args.angle_limit,
        max_tilt_deg=args.max_tilt,
        max_speed_mps=args.max_speed,
        invert_roll=args.invert_roll,
        invert_pitch=args.invert_pitch or True,  # default True to match remote_control feel
        manual=manual,
    )

    try:
        # allow some sensor warm-up
        time.sleep(0.5)

        # Commands
        if args.arm:
            ap.arm()
            time.sleep(0.2)
        # Auto-arm for flight-related commands
        if (args.takeoff is not None) or args.hold or (args.goto is not None):
            ap.arm()
            time.sleep(0.2)
        if args.takeoff is not None:
            ap.takeoff(args.takeoff)
        if args.hold:
            ap.hold(use_pos=True)
        if args.goto is not None:
            lat, lon = args.goto
            tgt_alt = args.alt if args.alt is not None else (msp.state.altitude_m or 2.0)
            ap.goto(lat, lon, tgt_alt, safe_alt_m=args.safe_alt, geofence_m=args.geofence)
        if args.land:
            ap.land()
        if args.disarm:
            ap.disarm()

        # Keep running until user interrupts
        print("[MAIN] Running. Press Ctrl+C to exit.")
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[MAIN] EMERGENCY DISARM (Ctrl+C)...")
        try:
            # Send multiple disarm frames to ensure FC receives it
            ap.disarm()
            time.sleep(0.1)
            ap.disarm()
            time.sleep(0.1)
            ap.disarm()
        except Exception as e:
            print(f"[MAIN] Disarm error: {e}")
    finally:
        try:
            ap.stop()
        except Exception:
            pass
        try:
            msp.stop()
        except Exception:
            pass
        try:
            if 'manual' in locals() and manual is not None:
                manual.stop()
        except Exception:
            pass


if __name__ == '__main__':
    main()
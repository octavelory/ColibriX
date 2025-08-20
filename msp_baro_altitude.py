#!/usr/bin/env python3
"""
MSP barometer altitude reader with adaptive, outlier-robust Kalman filtering.

- Connects to a flight controller over MSP (v1) via serial using pyserial
- Requests MSP_ALTITUDE (ID=109) periodically
- Parses altitude (cm) and converts to meters
- Zeros altitude to an initial median baseline (drone unarmed, on table)
- Filters using a 1D constant-velocity Kalman filter with:
  * gating on normalized innovation to reject outliers
  * online measurement-noise (R) adaptation via EMA of innovation^2
- Prints raw vs filtered altitude in the terminal; optional CSV logging

Usage examples:
  python msp_baro_altitude.py --port COM7 --baud 115200 --rate 10
  python msp_baro_altitude.py --port /dev/ttyUSB0 --rate 20 --log alt.csv
  python msp_baro_altitude.py --simulate --rate 20

Notes:
- Ensure the FC serial port you connect to has MSP enabled and is not in use by other apps.
- MSP_ALTITUDE is defined as returning altitude in centimeters (int32) and vario (int16). We only use altitude.
- If your firmware always reports zero altitude when disarmed, try arming or use another MSP message that exposes raw pressure;
  this script currently focuses on MSP_ALTITUDE.
"""

import argparse
import csv
import math
import random
import statistics
import struct
import sys
import time
from collections import deque
from typing import Optional, Tuple

# Optional pyserial import: simulation mode should work without it.
try:
    import serial  # type: ignore
    from serial.tools import list_ports as _serial_list_ports  # type: ignore
except Exception:  # pyserial not installed
    serial = None  # type: ignore
    _serial_list_ports = None  # type: ignore

MSP_ALTITUDE = 109  # MSPv1 command id
_HEADER_OK = b"$M>"
_HEADER_ERR = b"$M!"
_HEADER_REQ = b"$M<"


def list_serial_ports() -> list[str]:
    ports = []
    try:
        if _serial_list_ports is None:
            return ports
        for p in _serial_list_ports.comports():
            ports.append(p.device)
    except Exception:
        pass
    return ports


def msp_send_request(ser, cmd: int, payload: bytes = b"") -> None:
    size = len(payload)
    if size > 255:
        raise ValueError("MSP v1 payload too large")
    chk = size ^ cmd
    for b in payload:
        chk ^= b
    frame = bytearray()
    frame += _HEADER_REQ
    frame.append(size)
    frame.append(cmd)
    frame += payload
    frame.append(chk & 0xFF)
    ser.write(frame)


def _read_exact(ser, n: int, deadline: float) -> bytes:
    buf = bytearray()
    while len(buf) < n and time.time() < deadline:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
    return bytes(buf)


def msp_read_response(ser, expect_cmd: Optional[int] = None, timeout: float = 1.0) -> Tuple[int, bytes]:
    deadline = time.time() + timeout
    # find header
    hdr = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        hdr += b
        if len(hdr) > 3:
            del hdr[0 : len(hdr) - 3]
        if bytes(hdr) in (_HEADER_OK, _HEADER_ERR):
            break
    else:
        raise TimeoutError("MSP response timeout waiting for header")

    size_b = _read_exact(ser, 1, deadline)
    if not size_b:
        raise TimeoutError("MSP response timeout waiting for size")
    size = size_b[0]
    cmd_b = _read_exact(ser, 1, deadline)
    if not cmd_b:
        raise TimeoutError("MSP response timeout waiting for command")
    cmd = cmd_b[0]
    data = _read_exact(ser, size, deadline)
    if len(data) != size:
        raise TimeoutError("MSP response timeout waiting for payload")
    chk_b = _read_exact(ser, 1, deadline)
    if not chk_b:
        raise TimeoutError("MSP response timeout waiting for checksum")
    chk = chk_b[0]

    calc = size ^ cmd
    for b in data:
        calc ^= b
    if (calc & 0xFF) != chk:
        raise ValueError("MSP checksum mismatch")

    if expect_cmd is not None and cmd != expect_cmd:
        raise ValueError(f"Unexpected MSP cmd {cmd}, expected {expect_cmd}")

    return cmd, data


class AdaptiveKalman1D:
    """1D constant-velocity Kalman filter with adaptive measurement noise (R) and gating.

    State: x = [z, v]^T
    z: altitude (m), v: vertical speed (m/s)

    Q is derived from a white-acceleration model with variance sigma_a^2.
    R adapts via EMA of innovation^2 when gated measurements are accepted.
    """

    def __init__(
        self,
        dt: float,
        sigma_a: float = 1.0,  # m/s^2 process acceleration std
        r_var_init: float = 4.0,  # m^2 initial measurement variance (2 m std)
        r_alpha: float = 0.02,  # EMA factor for R adaptation
        gate_sigma: float = 5.0,
        r_min: float = 1e-4,
        r_max: float = 400.0,  # up to 20 m std
    ) -> None:
        self.dt = max(1e-3, float(dt))
        self.sigma_a = float(sigma_a)
        self.r_var = float(r_var_init)
        self.r_alpha = float(r_alpha)
        self.gate_sigma = float(gate_sigma)
        self.r_min = float(r_min)
        self.r_max = float(r_max)

        # State and covariance
        self.xz = 0.0
        self.xv = 0.0
        self.Pzz = 100.0
        self.Pzv = 0.0
        self.Pvv = 10.0

    def _q_matrix(self, dt: float) -> Tuple[float, float, float]:
        # Q = G * sigma_a^2 * G^T with G = [[0.5*dt^2], [dt]]
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        s2 = self.sigma_a * self.sigma_a
        Qzz = 0.25 * dt4 * s2
        Qzv = 0.5 * dt3 * s2
        Qvv = dt2 * s2
        return Qzz, Qzv, Qvv

    def predict(self, dt: Optional[float] = None) -> None:
        if dt is None:
            dt = self.dt
        else:
            dt = max(1e-3, float(dt))
            self.dt = dt
        # x = A x, with A = [[1, dt],[0,1]]
        self.xz = self.xz + dt * self.xv
        # self.xv unchanged
        # P = A P A^T + Q
        Pzz, Pzv, Pvv = self.Pzz, self.Pzv, self.Pvv
        # A P A^T
        Pzz_new = Pzz + dt * (Pzv + Pzv) + dt * dt * Pvv
        Pzv_new = Pzv + dt * Pvv
        Pvv_new = Pvv
        # + Q
        Qzz, Qzv, Qvv = self._q_matrix(dt)
        self.Pzz = Pzz_new + Qzz
        self.Pzv = Pzv_new + Qzv
        self.Pvv = Pvv_new + Qvv

    def update(self, z_meas: float) -> Tuple[bool, float, float]:
        # Innovation: y = z_meas - H x, with H = [1, 0]
        y = z_meas - self.xz
        S = self.Pzz + self.r_var
        if S <= 0.0:
            S = 1e-6
        # Gating on normalized innovation
        nu = y / math.sqrt(S)
        accepted = abs(nu) <= self.gate_sigma
        if accepted:
            # K = P H^T S^-1 => K = [Pzz/S, Pzv/S]^T
            Kz = self.Pzz / S
            Kv = self.Pzv / S
            # x = x + K y
            self.xz += Kz * y
            self.xv += Kv * y
            # P = (I - K H) P
            # (I - K H) = [[1 - Kz, 0], [-Kv, 1]]
            Pzz = (1.0 - Kz) * self.Pzz
            Pzv = (1.0 - Kz) * self.Pzv
            Pvv = self.Pvv - Kv * self.Pzv
            self.Pzz, self.Pzv, self.Pvv = Pzz, Pzv, Pvv
            # Adapt R using EMA of innovation^2
            r_new = (1.0 - self.r_alpha) * self.r_var + self.r_alpha * (y * y)
            self.r_var = min(self.r_max, max(self.r_min, r_new))
        # else: reject update; keep predicted state
        return accepted, y, S

    @property
    def state(self) -> Tuple[float, float]:
        return self.xz, self.xv


def parse_msp_altitude(payload: bytes) -> Optional[Tuple[float, Optional[float]]]:
    """Parse MSP_ALTITUDE payload.

    Returns altitude in meters and vario in m/s if available.
    """
    if len(payload) >= 6:
        alt_cm, vario_cms = struct.unpack_from("<ih", payload, 0)
        alt_m = alt_cm / 100.0
        vario_ms = vario_cms / 100.0
        return alt_m, vario_ms
    elif len(payload) >= 4:
        (alt_cm,) = struct.unpack_from("<i", payload, 0)
        return alt_cm / 100.0, None
    else:
        return None


def run_simulated(args: argparse.Namespace) -> None:
    """Simulate a baro altitude around 0 m with random walk, noise and outliers."""
    print("[SIM] Running simulated altitude stream. Ctrl-C to stop.")
    dt = 1.0 / max(1, args.rate)
    filt = AdaptiveKalman1D(
        dt=dt,
        sigma_a=args.sigma_a,
        r_var_init=args.r_init * args.r_init,
        r_alpha=args.r_alpha,
        gate_sigma=args.gate,
        r_min=args.r_min * args.r_min,
        r_max=args.r_max * args.r_max,
    )

    baseline_window = max(1, int(args.warmup * args.rate))
    baseline_buf: deque[float] = deque(maxlen=baseline_window)
    baseline: Optional[float] = None

    t0 = time.time()
    last = t0

    csv_writer = None
    csv_file = None
    if args.log:
        csv_file = open(args.log, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["t", "raw_m", "filt_m", "vel_mps", "acc", "R_m2", "accepted", "nu"])

    try:
        z_true = 0.0
        v_true = 0.0
        while True:
            now = time.time()
            # Maintain loop timing
            if now - last < dt:
                time.sleep(max(0.0, dt - (now - last)))
                now = time.time()
            last = now
            t = now - t0

            # Simulate physics
            # small drift acceleration in [-0.02, 0.02] m/s^2
            a_true = 0.02 * (2.0 * random.random() - 1.0)
            v_true += a_true * dt
            z_true += v_true * dt

            # Measurement noise with occasional outliers
            rnd = random.random()
            noise = random.gauss(0.0, args.r_init)
            if rnd < 0.02:
                # 2% chance of a big outlier
                noise += random.choice([-1, 1]) * random.uniform(5.0, 20.0)
            z_meas = z_true + noise

            # Baseline learning
            if baseline is None:
                baseline_buf.append(z_meas)
                if len(baseline_buf) >= baseline_window:
                    baseline = statistics.median(baseline_buf)
                raw_rel = z_meas  # will adjust after baseline ready
            else:
                raw_rel = z_meas - baseline

            filt.predict(dt)
            accepted, y, S = filt.update(raw_rel)
            z_hat, v_hat = filt.state
            nu = y / math.sqrt(S)
            if baseline is None:
                disp = f"t={t:6.2f}s warmup {len(baseline_buf)}/{baseline_window} raw=-- filt=-- R=--"
            else:
                disp = (
                    f"t={t:6.2f}s raw={raw_rel:+7.3f} m filt={z_hat:+7.3f} m v={v_hat:+6.3f} m/s "
                    f"R={math.sqrt(filt.r_var):.3f} m acc={'Y' if accepted else 'N'} nu={nu:+5.2f}"
                )
            print(disp)
            if csv_writer and baseline is not None:
                csv_writer.writerow([t, raw_rel, z_hat, v_hat, a_true, filt.r_var, int(accepted), nu])
                csv_file.flush()
    except KeyboardInterrupt:
        print("\n[SIM] Stopped.")
    finally:
        if csv_file:
            csv_file.close()


def run_msp(args: argparse.Namespace) -> None:
    if serial is None:
        print("pyserial is required for MSP mode. Install with: pip install pyserial", file=sys.stderr)
        return
    ports = list_serial_ports()
    port = args.port
    if not port:
        if len(ports) == 1:
            port = ports[0]
            print(f"Using detected port: {port}")
        elif len(ports) == 0:
            print("No serial ports detected. Specify --port (e.g., COM7 or /dev/ttyUSB0).", file=sys.stderr)
            return
        else:
            print("Multiple ports detected; specify one with --port:")
            for p in ports:
                print(f"  - {p}")
            return

    try:
        ser = serial.Serial(port=port, baudrate=args.baud, timeout=0.05)
    except Exception as e:
        print(f"Failed to open {port}: {e}", file=sys.stderr)
        return

    dt = 1.0 / max(1, args.rate)
    filt = AdaptiveKalman1D(
        dt=dt,
        sigma_a=args.sigma_a,
        r_var_init=args.r_init * args.r_init,
        r_alpha=args.r_alpha,
        gate_sigma=args.gate,
        r_min=args.r_min * args.r_min,
        r_max=args.r_max * args.r_max,
    )

    baseline_window = max(1, int(args.warmup * args.rate))
    baseline_buf: deque[float] = deque(maxlen=baseline_window)
    baseline: Optional[float] = None

    csv_writer = None
    csv_file = None
    if args.log:
        try:
            csv_file = open(args.log, "w", newline="")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["t", "raw_m", "filt_m", "vel_mps", "R_m2", "accepted", "nu"])
        except Exception as e:
            print(f"Failed to open log file {args.log}: {e}")
            csv_writer = None
            csv_file = None

    print(
        f"Reading MSP_ALTITUDE from {port} @ {args.baud} baud, {args.rate} Hz. Ctrl-C to stop.\n"
        f"Warm-up {args.warmup}s to learn baseline (drone disarmed, stationary)."
    )

    t0 = time.time()
    last = t0
    try:
        while True:
            now = time.time()
            # Maintain loop timing
            if now - last < dt:
                time.sleep(max(0.0, dt - (now - last)))
                now = time.time()
            t = now - t0
            last = now

            try:
                msp_send_request(ser, MSP_ALTITUDE)
                _, payload = msp_read_response(ser, expect_cmd=MSP_ALTITUDE, timeout=0.5)
            except TimeoutError:
                print("Timeout: No MSP response. Check port/baud and MSP enabled on FC.")
                continue
            except Exception as e:
                print(f"MSP error: {e}")
                continue

            parsed = parse_msp_altitude(payload)
            if not parsed:
                print("Received malformed MSP_ALTITUDE payload.")
                continue
            raw_alt_m, _vario = parsed

            # Baseline learning and zeroing
            if baseline is None:
                baseline_buf.append(raw_alt_m)
                if len(baseline_buf) >= baseline_window:
                    baseline = statistics.median(baseline_buf)
                raw_rel = raw_alt_m
            else:
                raw_rel = raw_alt_m - baseline

            filt.predict(dt)
            accepted, y, S = filt.update(raw_rel)
            z_hat, v_hat = filt.state
            nu = y / math.sqrt(S) if S > 0 else 0.0

            if baseline is None:
                print(
                    f"t={t:6.2f}s warmup {len(baseline_buf)}/{baseline_window} raw=-- filt=-- R=--"
                )
            else:
                print(
                    f"t={t:6.2f}s raw={raw_rel:+7.3f} m filt={z_hat:+7.3f} m v={v_hat:+6.3f} m/s "
                    f"R={math.sqrt(filt.r_var):.3f} m acc={'Y' if accepted else 'N'} nu={nu:+5.2f}"
                )
                if csv_writer:
                    csv_writer.writerow([t, raw_rel, z_hat, v_hat, filt.r_var, int(accepted), nu])
                    csv_file.flush()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        if csv_file:
            try:
                csv_file.close()
            except Exception:
                pass


def main():
    p = argparse.ArgumentParser(description="MSP barometer altitude with adaptive filtering")
    p.add_argument("--port", help="Serial port (e.g., COM7 or /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument("--rate", type=int, default=10, help="Polling rate in Hz (default: 10)")
    p.add_argument("--warmup", type=float, default=5.0, help="Warm-up seconds to learn zero baseline (default: 5)")
    p.add_argument("--gate", type=float, default=5.0, help="Gating threshold in sigma for outlier rejection (default: 5.0)")
    p.add_argument("--r-alpha", dest="r_alpha", type=float, default=0.02, help="EMA factor for R adaptation (default: 0.02)")
    p.add_argument("--r-init", dest="r_init", type=float, default=2.0, help="Initial measurement std dev in m (default: 2.0)")
    p.add_argument("--r-min", dest="r_min", type=float, default=0.02, help="Min measurement std dev in m (default: 0.02)")
    p.add_argument("--r-max", dest="r_max", type=float, default=20.0, help="Max measurement std dev in m (default: 20.0)")
    p.add_argument("--sigma-a", dest="sigma_a", type=float, default=0.5, help="Process acceleration std in m/s^2 (default: 0.5)")
    p.add_argument("--log", help="CSV log file path")
    p.add_argument("--simulate", action="store_true", help="Run in simulation mode without a FC")

    args = p.parse_args()

    if args.simulate:
        run_simulated(args)
    else:
        run_msp(args)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Demo mission planner/executor for autonomous_drone.Autopilot

Plan:
- Arm
- Takeoff to target altitude
- If GPS fix: fly two short legs (N then E) around home and return near start
- Land and disarm

By default, prints the plan only. Use --execute to actually run it.
"""
import argparse
import sys
import time
import math
from typing import Optional, Tuple

from autonomous_drone import (
    MspClient,
    Autopilot,
    meters_per_deg_lat,
    meters_per_deg_lon,
)


def has_gps_pos(msp: MspClient) -> bool:
    st = msp.state
    return bool(st.gps_fix and st.lat_deg is not None and st.lon_deg is not None)


def wait_for_altitude(msp: MspClient, target_alt: float, tol: float = 0.15, timeout: float = 20.0) -> bool:
    """Wait until altitude within tolerance of target."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        alt = msp.state.altitude_m
        if alt is not None and abs(alt - target_alt) <= tol:
            return True
        time.sleep(0.1)
    return False


def distance_to(msp: MspClient, tgt: Tuple[float, float]) -> float:
    st = msp.state
    if st.lat_deg is None or st.lon_deg is None:
        return float("inf")
    m_per_lat = meters_per_deg_lat(st.lat_deg)
    m_per_lon = meters_per_deg_lon(st.lat_deg)
    dN = (tgt[0] - st.lat_deg) * m_per_lat
    dE = (tgt[1] - st.lon_deg) * m_per_lon
    return math.hypot(dN, dE)


def wait_until_close_to(msp: MspClient, tgt: Tuple[float, float], tol_m: float = 1.5, timeout: float = 45.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = distance_to(msp, tgt)
        if d < tol_m:
            return True
        time.sleep(0.2)
    return False


def compute_waypoints_around_home(msp: MspClient, leg_m: float) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    st = msp.state
    if not (st.gps_fix and st.lat_deg is not None and st.lon_deg is not None):
        return None
    lat = st.lat_deg
    lon = st.lon_deg
    m_per_lat = meters_per_deg_lat(lat)
    m_per_lon = meters_per_deg_lon(lat)
    dlat_N = leg_m / m_per_lat
    dlon_E = leg_m / m_per_lon
    wpt1 = (lat + dlat_N, lon)           # go North
    wpt2 = (lat + dlat_N, lon + dlon_E)  # then East
    return wpt1, wpt2


def print_plan(exec_: bool, takeoff_alt: float, wpts: Optional[Tuple[Tuple[float, float], Tuple[float, float]]]):
    print("\n=== Demo Mission Plan ===")
    print(f"- Arm")
    print(f"- Takeoff to {takeoff_alt:.2f} m")
    if wpts is None:
        print("- No GPS fix -> skip waypoints; hold briefly")
    else:
        (w1, w2) = wpts
        print(f"- Goto WPT1: lat={w1[0]:.6f}, lon={w1[1]:.6f}")
        print(f"- Goto WPT2: lat={w2[0]:.6f}, lon={w2[1]:.6f}")
    print("- Land and disarm")
    print(f"Mode: {'EXECUTE' if exec_ else 'DRY-RUN'}\n")


def main():
    parser = argparse.ArgumentParser(description="Demo mission using autonomous_drone.Autopilot")
    parser.add_argument('--port', type=str, default=None, help='Serial port (e.g., COM3 on Windows, /dev/ttyAMA0 on Pi)')
    parser.add_argument('--execute', action='store_true', help='Actually send RC commands to fly the mission')
    parser.add_argument('--takeoff', type=float, default=1.5, help='Takeoff altitude (m)')
    parser.add_argument('--hover-sec', type=float, default=5.0, help='Hover time when GPS not available (s)')
    parser.add_argument('--leg-dist', type=float, default=10.0, help='Leg distance for waypoints (m)')
    parser.add_argument('--safe-alt', type=float, default=10.0, help='Safe altitude for waypoint travel (m)')
    parser.add_argument('--geofence', type=float, default=200.0, help='Geofence radius (m); set 0 to disable')
    parser.add_argument('--angle-limit', type=float, default=45.0, help='Angle mode limit (deg)')
    parser.add_argument('--max-tilt', type=float, default=25.0, help='Max tilt command (deg)')
    parser.add_argument('--max-speed', type=float, default=5.0, help='Max ground speed target (m/s)')
    parser.add_argument('--invert-roll', action='store_true', help='Invert roll mapping')
    parser.add_argument('--invert-pitch', action='store_true', help='Invert pitch mapping')

    args = parser.parse_args()

    # Plan-only mode: do not connect to flight controller
    if not args.execute:
        print_plan(False, args.takeoff, None)
        print("Dry-run only. Re-run with --execute to fly. Example:")
        print("  python demo_mission.py --port COM3 --execute")
        return

    msp = MspClient(port=args.port)
    try:
        msp.start()
    except Exception as e:
        print(f"[MSP] Failed to start: {e}")
        sys.exit(1)

    ap = Autopilot(
        msp,
        angle_limit_deg=args.angle_limit,
        max_tilt_deg=args.max_tilt,
        max_speed_mps=args.max_speed,
        invert_roll=args.invert_roll,
        invert_pitch=args.invert_pitch or True,  # keep default feel
    )

    try:
        # sensor warm-up
        time.sleep(1.0)
        st = msp.state
        print("[INFO] GPS fix:", st.gps_fix, "sats:", st.gps_num_sat)
        if st.lat_deg is not None and st.lon_deg is not None:
            print(f"[INFO] Current pos: lat={st.lat_deg:.6f}, lon={st.lon_deg:.6f}")
        if st.altitude_m is not None:
            print(f"[INFO] Altitude: {st.altitude_m:.2f} m")

        wpts = compute_waypoints_around_home(msp, args.leg_dist)
        print_plan(args.execute, args.takeoff, wpts)

        # Execute mission
        print("[MISSION] Arming...")
        if not ap.arm():
            print("[MISSION] Arm refused (throttle safety). Aborting.")
            return
        time.sleep(0.3)

        print(f"[MISSION] Takeoff to {args.takeoff:.2f} m ...")
        ap.takeoff(args.takeoff)
        if not wait_for_altitude(msp, args.takeoff, tol=0.2, timeout=30.0):
            print("[MISSION] Takeoff timeout. Proceeding to HOLD.")
        ap.hold(use_pos=True)

        if wpts is not None:
            # Leg 1
            w1, w2 = wpts
            tgt_alt = max(args.takeoff, 1.0)
            print(f"[MISSION] Goto WPT1 @ alt {tgt_alt:.1f} m -> {w1}")
            ap.goto(w1[0], w1[1], tgt_alt, safe_alt_m=args.safe_alt, geofence_m=args.geofence)
            wait_until_close_to(msp, w1, tol_m=2.0, timeout=60.0)
            ap.hold(use_pos=True)
            time.sleep(1.0)

            # Leg 2
            print(f"[MISSION] Goto WPT2 @ alt {tgt_alt:.1f} m -> {w2}")
            ap.goto(w2[0], w2[1], tgt_alt, safe_alt_m=args.safe_alt, geofence_m=args.geofence)
            wait_until_close_to(msp, w2, tol_m=2.0, timeout=60.0)
            ap.hold(use_pos=True)
            time.sleep(1.0)
        else:
            # No GPS: just hover a bit
            print(f"[MISSION] No GPS fix, hover {args.hover_sec:.1f}s...")
            time.sleep(max(0.0, args.hover_sec))

        # Land
        print("[MISSION] Landing...")
        ap.land()
        # wait until close to ground if we get altitude
        if msp.state.altitude_m is not None:
            t0 = time.time()
            while time.time() - t0 < 40.0:
                alt = msp.state.altitude_m
                if alt is not None and alt < 0.2:
                    break
                time.sleep(0.2)
        # Ensure disarm command sent in case
        ap.disarm()
        print("[MISSION] Completed.")

    except KeyboardInterrupt:
        print("\n[MISSION] Interrupted by user. Disarming...")
        try:
            ap.disarm()
        except Exception:
            pass
    finally:
        try:
            ap.stop()
        except Exception:
            pass
        try:
            msp.stop()
        except Exception:
            pass


if __name__ == '__main__':
    main()

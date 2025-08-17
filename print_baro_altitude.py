#!/usr/bin/env python3
"""
Print barometric altitude from Betaflight via MSP.

Usage examples (Windows):
  python print_baro_altitude.py --port COM5

Requires: pyserial
"""
from __future__ import annotations

import argparse
import time
import sys

from autonomous_drone import MspClient


def main():
    parser = argparse.ArgumentParser(description="Print barometric altitude (meters) via MSP")
    parser.add_argument("--port", required=False, default="/dev/ttyAMA0", help="Serial port of the FC (e.g., COM5 on Windows)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--rate", type=float, default=5.0, help="Print rate in Hz (default: 5.0)")
    args = parser.parse_args()

    msp = MspClient(port=args.port, baud=args.baud, use_ultrasonic=False)

    try:
        msp.start()
        print("Connected. Printing barometric altitude (m). Press Ctrl+C to stop.")
        interval = 1.0 / max(0.5, float(args.rate))
        while True:
            alt = msp.state.baro_alt_m_raw
            if alt is not None:
                print(f"Altitude (baro): {alt:.2f} m")
            else:
                print("Altitude (baro): -- waiting for data --")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        try:
            msp.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()

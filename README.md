# ColibriX Betaflight Autopilot (Altitude Hold)

A simple Python autopilot for Betaflight that can take off to a target altitude, hold it steadily, and land on command. It reads the drone’s altitude and GPS via MSP and sends RC commands back to Betaflight. You can override at any time with a joystick or by disarming.


## Plain-English Overview

- You plug in your Betaflight quad via USB.
- Run `autopilot.py` and press:
  - `a` to arm
  - `t` to take off to a set height and hover
  - `h` to hold your current height
  - `l` to land
  - `z` to re-zero altitude reference
  - `q` to quit (disarms first)
- If you move a joystick (optional), the script immediately gives control back to you and stays in manual until you explicitly choose an auto mode again.
- It’s designed to be safe: it won’t rise aggressively when it can’t read altitude, clamps throttle, and disarms on exit.

You don’t need to understand MSP or control theory to use it—just the keys above. Start with props off to get familiar and tune one parameter (hover power) so it knows roughly how much throttle to hover.


## Features (What it does exactly)

- High-level altitude control
  - Auto takeoff to a target altitude, hover (alt-hold), and land.
  - Flight states: `MANUAL`, `TAKEOFF`, `HOLD`, `LAND`.
- Sensor fusion and relative altitude
  - Reads barometer altitude (`MSP_ALTITUDE`) and GPS (`MSP_RAW_GPS`).
  - Smart relative zeroing (baseline/bias) for both baro and GPS so altitude is treated as “relative to now”.
  - Optional blending of GPS altitude with baro (configurable weight) and smoothing filter.
- Throttle PI controller
  - Simple PI controller adjusts throttle around a configurable hover PWM.
  - Gains and integral clamp are configurable.
- Immediate manual override
  - Optional joystick input (pygame). Any stick or button latches manual mode.
  - While latched, the script does not fight your inputs; You are fully in control.
  - Press auto keys again (e.g., `t` or `h`) to resume autonomous modes.
- Yaw lock (optional)
  - Keeps yaw centered unless disabled.
- Robust MSP/RC output
  - Uses `MSP_SET_RAW_RC` to send RC commands at ~50 Hz.
  - Graceful disarm and throttle cut on exit.
- Windows-friendly serial autodetect
  - Auto-detects likely COM ports; also works on Linux.
- Safety behavior
  - If altitude is missing mid-flight, holds near-hover throttle conservatively.
  - Clamps throttle range. Disarms on exit.


## Requirements

- Python 3.8+
- pyserial (required)
- pygame (optional, only for joystick override)

Install:

```bash
pip install pyserial pygame
```


## Betaflight Setup Notes

- ARM on `AUX1` (or adjust in code if you use a different channel).
- Angle/Level mode recommended for simple hovering.
- Barometer enabled/working. GPS optional (used for altitude blend if present).
- Connect via USB or serial that exposes MSP.


## Usage

- Dry run (no serial, safe to try):

```bash
python autopilot.py --dry-run -v
```

- Typical run (auto-detect port):

```bash
python autopilot.py --target-alt 1.5 --hover-pwm 1550 -v
```

- With joystick manual override:

```bash
python autopilot.py --joystick --joy-deadzone 0.08
```

- Useful options:
  - `--port COM5` or `/dev/ttyUSB0` to force a port.
  - `--gps-weight 0.0..1.0` blend GPS altitude with baro (baro primary by default).
  - `--bias-on startup|arm|takeoff|never` when to zero the altitude reference.
  - `--alt-lpf 0.0..1.0` smoothing factor for altitude (0 disables smoothing).
  - `--no-yaw-lock` to allow yaw control in auto modes.
  - `--kp`, `--ki`, `--imax` PI controller gains/clamp.

Keyboard controls shown at startup:

- `a` Arm/Disarm
- `t` Auto takeoff to `--target-alt` then HOLD
- `h` Hold current altitude
- `l` Land
- `+` / `-` or `w` / `s` Adjust target altitude by ±0.1 m
- `z` Zero altitude reference now
- `q` Quit (disarm & exit)


## Joystick (optional)

- Enable with `--joystick` (requires pygame).
- Any stick motion or button press latches MANUAL override (the script stops commanding throttle/attitude).
- Button LB/L1 toggles Arm/Disarm (configurable in code: `BUTTON_ARM_DISARM = 4`).
- Axes mapping (mirrors `remote_control.py`):
  - Yaw: Axis 0
  - Throttle: Axis 1 (inverted)
  - Roll: Axis 3
  - Pitch: Axis 4 (inverted)
- Deadzone: `--joy-deadzone` (default 0.08).


## How it works (short)

- The script reads altitude from Betaflight via MSP. On startup (or arm/takeoff), it saves a “zero” reference so all heights are relative to that point.
- A PI controller adds/subtracts throttle around `--hover-pwm` to keep the drone at the target height.
- GPS altitude (if available) can be blended in to improve robustness; a simple smoothing filter reduces noise.
- A small state machine drives takeoff, hold, and land. Manual joystick input always wins and latches until you opt back into auto.


## Tuning tips

- Start with props off. Watch the UI line for altitude and throttle.
- `--hover-pwm`: set near the PWM your quad needs to hover.
- Increase `--kp` for faster response; add a little `--ki` to eliminate steady-state error. Use `--imax` to limit integral windup.


## Troubleshooting

- No port found: pass `--port` explicitly, e.g., `--port COM7`.
- No altitude: check barometer in Betaflight; GPS is optional.
- Joystick not working: install `pygame` and verify Windows sees your controller.
- Won’t arm: throttle must be at minimum; confirm ARM is mapped to AUX1 in Betaflight.


## MSP details (for reference)

- Reads: `MSP_ALTITUDE (109)`, `MSP_RAW_GPS (106)`
- Writes: `MSP_SET_RAW_RC (200)`


## Safety

- Test with props off first.
- Use in a safe environment at your own risk.
- Always be ready to disarm.

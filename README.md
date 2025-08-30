# ColibriX Autonomous Drone

An MSP-based autopilot for Betaflight that can arm, take off to a target altitude, hold, fly to a GPS waypoint, and land. It exposes a small local Web API and updates the included web UI in real time.

This repository contains three main scripts:
- `autonomous_drone.py` — autonomous flight controller with ultrasonic/baro fusion, hover learning, manual gamepad override, and an embedded Web API/telemetry publisher.
- `remote_control.py` — manual controller used for testing MSP I/O, joystick mapping, servos, and the same Web API wiring; ideal for bench tests without the autonomous logic.
- `test_buttons.py` — tiny gamepad test utility to verify button/axis indexes; handy before flying.


## Features

- Modes: IDLE, TAKEOFF, HOLD (alt/pos), GOTO (waypoint), LAND.
- Sensor fusion: baro + optional ultrasonic with smart blending and low-pass filter.
- Hover learning: persists hover PWM and compensates for tilt and per-cell voltage.
- Battery safety: low/critical thresholds per cell; saver mode and forced land.
- Manual override: any stick/button latches full RC pass-through until restart.
- Web API + UI: starts the Flask API at launch and publishes telemetry for the web console in `index.html` + `assets/`.


## Quick start

1) Install dependencies (Python 3.8+):

```powershell
pip install pyserial pygame flask
```

2) Run the autonomous controller. It will start the local API on port 5000 automatically:

```powershell
python autonomous_drone.py --takeoff 1.5 --hold --telemetry
```

3) Open the UI in your browser:

- http://127.0.0.1:5000

Click on the map to set a destination (≤ 1 km), Verify, then Launch. Telemetry updates flow via SSE.


## Common options (excerpt)

- `--port COM5` Force serial port (auto tries common Linux ports by default).
- `--no-ultrasonic` Disable ultrasonic fusion.
- `--alt-kp/--alt-ki/--alt-pwm-max` Altitude hold tuning.
- `--hover-*` Control hover learning and persistence (see `--help`).
- `--no-manual` Disable gamepad override.


## Why the extra scripts?

- `remote_control.py`: I created this for quick, focused testing of MSP, joystick axes, arm/disarm logic, servo control, and the API without the complexity of the autopilot. It’s ideal for bench tests and validating hardware wiring.
- `test_buttons.py`: a simple helper to print button/axis events so you can confirm your controller mapping before flying.


## Safety

- Test with props off first. Confirm arming, disarming, and stick directions.
- Ensure AUX1 is configured for ARM in Betaflight.
- Use in a safe environment and be ready to disarm at all times.

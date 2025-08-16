# ColibriX

A low-cost, autonomous micro‑delivery drone prototype built on Raspberry Pi + Betaflight (MSP), designed to make point‑to‑point deliveries over short distances with a simple, user‑friendly workflow. The system emphasizes accessibility, openness, and pragmatic reliability over camera-first features or proprietary infrastructure.

Author: Octave
Site + 3D viewer: https://colibrix.vercel.app/ — 3D: https://colibrix.vercel.app/viewer

---

## TL;DR
- Purpose: autonomous last‑meter courier drone delivering small payloads quickly in hard‑to‑reach areas.
- Cost target: ~200 € prototype using commodity parts and open-source firmware.
- Stack: Raspberry Pi, Betaflight FC (MSP over serial), GPS, optional ultrasonic, Python controllers.
- Modes: arm/disarm, takeoff, altitude/position hold, goto(lat/lon), land, geofence.
- Current limitations: no return‑to‑home yet, weather limits, prototype frame without fairing.

---

## Why ColibriX is different
- Not a camera drone (DJI‑style): built for transport, not filming. UI goal: pick a point B and go.
- Not FPV: removes the pilot; favors autonomy, predictability, and ease of use.
- Not an industrial black box: open, modular, affordable; suitable for small shops, makers, schools.

---

## Repository structure
- `autonomous_drone.py`: MSP client + sensor fusion + autonomous `Autopilot` (takeoff/hold/goto/land) and safety (geofence, checks).
- `demo_mission.py`: example mission runner using `Autopilot` (dry‑run by default; `--execute` to fly).
- `remote_control.py`: joystick‑driven manual control over MSP with a rich terminal UI; basic auto helpers, camera servo control.
- `control_camera_tower.py`: standalone camera gimbal control with joystick and smooth servo motion.
- `test_buttons.py`: quick joystick event visualizer for mapping buttons/axes.

---

## Hardware (prototype)
- Flight controller: Betaflight (tested with SpeedyBee F405 AIO; MSP @ 115200 baud).
- Compute: Raspberry Pi (runs Python + MSP + optional ultrasonic via GPIO).
- Sensors: GPS (MSP_RAW_GPS), barometer (MSP_ALTITUDE), attitude (MSP_ATTITUDE), optional ultrasonic (HC‑SR04 class).
- Frame: quadcopter prototype, no fairing yet; payload sling underframe.

Notes
- FC mounting had to be rotated ~45° due to mismatched holes (compensated in software/firmware config).
- Barometer can benefit from a fairing to reduce prop wash pressure errors.

---

## Software prerequisites
- Python 3.9+
- Packages: `pyserial`, `pygame`, `gpiozero` (Pi only)

Example installation
```bash
pip install pyserial pygame gpiozero
```

Platform notes
- `gpiozero` and ultrasonic/servo features require Raspberry Pi GPIO. On desktop (Windows/macOS/Linux), use only MSP features.
- Serial ports: Pi: `/dev/ttyAMA0`, `/dev/ttyS0` (autodetected). Windows: typically `COM3`.

---

## Usage

### 1) Autonomous control (low-level CLI)
`autonomous_drone.py` offers direct commands over MSP.

Examples
```bash
python autonomous_drone.py --arm
python autonomous_drone.py --takeoff 1.5
python autonomous_drone.py --goto 48.85837 2.29448 --alt 5 --safe-alt 8 --geofence 200
python autonomous_drone.py --land
```

Key options (selection)
- `--port`: serial port (auto on Pi; e.g. `COM3` on Windows)
- `--alt`, `--safe-alt`, `--geofence`: altitude target, cruise safety altitude, radius geofence
- `--ultra/--no-ultra`: enable/disable ultrasonic fusion (if supported)

### 2) Demo mission
`demo_mission.py` shows a mini‑mission using `Autopilot`.

Dry‑run (prints plan only)
```bash
python demo_mission.py
```
Execute (send RC over MSP)
```bash
python demo_mission.py --port COM3 --execute
```
Options include: `--takeoff`, `--safe-alt`, `--geofence`, `--angle-limit`, `--max-tilt`, `--max-speed`.

### 3) Manual joystick control + camera servos
`remote_control.py` provides a terminal UI with joystick input, arm/disarm, ALTHOLD toggle, yaw lock, and camera servo mode.

```bash
python remote_control.py
```
Highlights
- Arm/disarm safety (throttle must be low)
- Altitude readout via MSP, GPS status, channel visualization
- Camera mode: right stick controls two servos with smoothing (requires Pi + GPIO)

### 4) Camera tower only
```bash
python control_camera_tower.py
```
- Smooth joystick control for two servos (Pi + GPIO).

### 5) Joystick tester
```bash
python test_buttons.py
```
- Prints live button/axis/hat events to help map your device.

---

## How it works (high level)

- MSP transport: `MspClient` encapsulates MSP framing, periodic polling of `MSP_ALTITUDE`, `MSP_RAW_GPS`, `MSP_ATTITUDE`, and RC outputs via `MSP_SET_RAW_RC`.
- Sensor fusion: `DroneState` fuses baro altitude with optional ultrasonic using a smooth blend near the ultrasonic’s range limit, plus a slow baro bias calibration near ground.
- Autopilot: `Autopilot` runs in a control loop with modes `IDLE/TAKEOFF/HOLD/GOTO/LAND`. It implements:
  - Altitude PI around a slowly adapting hover PWM estimator
  - Heading hold (proportional yaw)
  - Position control (meters error -> tilt angles -> RC) with velocity damping from GPS ground speed
  - Geofence check vs home if available

Key classes/functions
- `autonomous_drone.py`: `MspClient`, `Autopilot`, `DroneState`, helpers like `meters_per_deg_lat()`, `meters_per_deg_lon()`
- `demo_mission.py`: `compute_waypoints_around_home()`, `wait_for_altitude()`, `wait_until_close_to()`

---

## Current limitations and safety
- No automatic Return‑to‑Home yet.
- Battery policy (prototype):
  - Low (<20%): continue and try to get close to destination.
  - Critical (<10%): attempt nearest safe landing (no obstacle sensing if Pi stack fails).
- Weather: not water‑proof; wind not yet stress‑tested.
- Failures: if the Pi stack fails, FC can still stabilize but without obstacle avoidance. FC failure = crash (never observed during tests).

Always follow local regulations and no‑fly zones. Fly only in safe, legal environments.

---

## Roadmap
- Add RTH and mission abort logic.
- Improve GPS precision and landing accuracy (better GNSS module; RTK maybe).
- Add vision‑based precision landing and obstacle avoidance.
- 3D‑printed fairing to shield baro and protect electronics (optimize weight).
- Web UI for mission selection over onboard Wi‑Fi AP.

---

## Somes pictures
- ![PLACEHOLDER](https://i.ibb.co/bjTHBjmK/IMG-20250517-151836.jpg) Overall drone photo on bench (show frame, FC, Pi, GPS, payload hook).
- ![PLACEHOLDER](https://i.ibb.co/pvfQtQ2Z/IMG-20250517-151832.jpg) Close‑up of FC rotated mounting with caption about 25.5×25.5 pattern.
- ![PLACEHOLDER](https://i.ibb.co/hxq5JY8t/Capture-d-cran-2025-08-15-143926.png) Screenshot of demo mission CLI plan and in‑flight status.
- ![PLACEHOLDER](https://i.ibb.co/xthchGw8/image.png) 3D printed shell concept render (from your CAD).

---

## Credits
- Firmware FC: Betaflight
- Libraries: pyserial, pygame, gpiozero
- Author: Octave Lory

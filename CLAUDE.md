# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PantherLunabotics 2025-2026 is a robotics competition project (NASA Lunabotics) for Florida Institute of Technology. It is a distributed control system: Python runs on both the operator's laptop (mission control) and the robot's onboard Jetson computer, while a C++ layer (compiled via CMake/pybind11) handles low-level motor controller communication over CAN bus.

## Feature Workflow

1. Understand the prompt
2. Display a brief summary of the action you will take
3. Proceed with implementation
4. Code review

## Prompts

Prompts will often contain misspellings. Before taking action, read the prompt carefully and correct any misspellings to determine the intended meaning. If a word or phrase is ambiguous and the intent is unclear, ask for clarification before proceeding. If requirements are missing, ask for them as well.

## Archive

The repository contains an `archive/` directory of old, unused code. **Do not read, reference, or code review anything in `archive/` unless specifically asked.** It is kept for historical reference only and is not part of the active codebase.

## Debugging

When investigating a bug, **prove the root cause before implementing a fix.** Do not guess at solutions — use telemetry logs, print statements, CAN bus data, or any other available evidence to confirm what is actually going wrong. Only once you have verified the problem should you implement the fix. A fix without proof is just a guess.

## Code Review

Every time you write code, review the implementation to ensure it is efficient, clean, and correct. Make the smallest viable change — do not modify what does not need to be changed.

## Build & Run

Code is developed on a Windows laptop or via SSH into the Jetson computer running Ubuntu Linux. Many files and commands are Linux-specific and will only run correctly on the Jetson.
Note: all files exist on both devices as long as the GitHub repo is synced.

**Robot (on Jetson):**
```bash
cd onboard_software
python robot.py
```

**Mission Control (on laptop):**
```bash
cd mission_control
python3 control.py
```

**Build C++ motor controller module (on Jetson):**
```bash
cd library/motor_controller
mkdir build && cd build
cmake ..
make
```
The C++ module uses pybind11 to expose `MotorController` to Python.

**Warning:** `library/motor_controller/CMakeLists.txt` has hardcoded paths (e.g. `/home/luna01/...`). These must be updated if the Jetson username or Python version changes.

**Python dependencies:** `pip install -r requirements.txt`

**Important:** Scripts must be run from their own directory (`cd onboard_software` or `cd mission_control`) because they manipulate `sys.path` relative to `__file__` to find the `library/` package.

**Cross-boundary imports (intentional):** `mission_control/client.py` and `mission_control/control.py` add `onboard_software/` to `sys.path` so they can import `robot_params`. This is intentional — `robot_params.py` is the single source of truth for all robot configuration and is shared across both sides this way. Similarly, `Lidar.run_viewer()` in `onboard_software/subsystems/perception.py` is a static method that executes on the mission control side; it is co-located with the `Lidar` class intentionally to keep all lidar code together. **Do not refactor these patterns** — they are deliberate decisions, not architectural defects.

## Competition Rules (Software-Relevant)

Full guidebook: `lunabotics-guidebook-2025-2026.txt` — **do not read the full guidebook unless specifically asked.** The software-relevant rules are summarized below. The guidebook is mostly administrative (applications, papers, eligibility, awards) and will waste context window.

### Arena Layout

**Coordinate system:** Origin (0,0) at arena bottom-left. +X = east (right), +Y = north (up). Heading: 0° = north, positive = clockwise (90° = east). All robot poses, targets, and arena coordinates use this convention.

```
Top-down view.  Origin (0,0) at bottom-left.  X → right (east), Y → up (north).
Outer dimensions: 6.88 m × 5.0 m.  Interior ≈ 6.8 m × 5.0 m.

           ◄── 2.5 m ──►◄──────────── 4.38 m ────────────►
           ◄──────────────── 6.88 m ──────────────────────►
    ┌──────────────┬──────────────────────────────────────┐  ▲
    │              │                                      │  │
    │  EXCAVATION  │          OBSTACLE ZONE               │  │
    │    ZONE      │                                      │  │
    │              │     ☼ boulders (30-40cm dia, min 3)   │  │
    │  (dig here,  │     ○ craters (40-50cm w/d, min 3)   │ 5.0 m
    │   rocks may  │     ▌ central column (permanent)     │  │
    │   be moved)  │                                      │  │
    │              │     Randomly placed before each round.│  │
    │  ┌─────────┐ │         ┌──────────────────┐         │  │
    │  │ STARTING│ │         │  CONSTRUCTION    │         │  │
    │  │  ZONE   │ │         │  ┌──BERM───┐     │         │  │
    │  │ [Robot] │ │         │  │1.7×0.8m │     │         │  │
    │  └─────────┘ │         │  └─────────┘     │         │  │
    │  (no rocks)  │         │  Red box 2.2×0.9m│         │  │
    └──────────────┴─────────┴──────────────────┴─────────┘  ▼
    (0,0)                     Berm center ≈ (5.38, 0.6)
```

- Regolith: ~45 cm deep BP-1 crushed basalt simulant (may contain ~2 cm gravel)
- Regolith must be carried through obstacle zone to construction zone (no bulldozing)
- Only berm volume inside the red box counts toward scoring

### Obstacles
- **Boulders:** Min 3, randomly placed each round, ~30-40cm diameter with varying heights
- **Craters:** Min 3, varying depths/widths up to ~40-50cm
- **Central support column:** Permanent fixture, must be avoided
- **Penalty:** 30 pts per rock contact or crater crossing during autonomous operation (max 90 pts)

### Robot Constraints
- Max mass: 80 kg, stowed volume: 150cm x 75cm x 75cm
- E-stop button required (40mm min, highest practical location, must stop motion AND disable power)

### Timing
- 10 min setup, 30 min competition run, 5 min removal
- Robot must move within 5 min of timer start or attempt is terminated
- Loss of locomotion for 5 min = attempt terminated
- Two attempts allowed per team

### Communications
- IEEE 802.11 WiFi only, assigned SSID "Team_##", encryption required
- Max average bandwidth: 4 Mbps (+ 500 Kbps per NASA situational awareness camera used)
- At arena: all comms through NASA-provided WAP to MCC only — **no backchannel wireless connections** (disqualification)
- Bluetooth: Class 2 & 3 only (max 2.5 mW). Class 1, Zigbee, power amplifiers all prohibited
- External WiFi antenna required

### Autonomy Rules
**Allowed sensors:** IMUs, cameras, fiducial targets/beacons on arena frame

**Prohibited for autonomy:** GPS, compasses (analog/digital), touch sensors, ultrasonic proximity sensors, infrared sensors, **using walls for navigation/mapping** (disqualification)

**Autonomy scoring tiers:**
- Excavation only: 75 pts
- Excavation + dump (traversal via remote control): 125 pts
- Excavation + dump + travel: 375 pts
- Full autonomy (one cycle): 450 pts
- Full autonomy (entire run): 600 pts

**During autonomous operation:** telemetry allowed for health monitoring only, no control input, all team members hands-free, cannot update autonomy program between runs to account for obstacle locations

### Berm Scoring
- Scored by volume within target area (volumetric scan before/after)
- Productivity by mass: cm³ berm / min / kg × 4.4 coefficient
- Productivity by energy: cm³ berm / min / Wh × 1.5 coefficient
- Regolith must come from excavation zone, must be carried through obstacle zone (no bulldozing)

## Hardware & Sensors

The robot has an auger for intaking and outtaking regolith (sand/dirt). The drive base uses four Archimedean screws. All motors are REV NEO motors controlled via SparkCAN devices over CAN bus through a USB-to-CAN adapter. The onboard processor (and server side) is a Jetson Orin Nano Super Developer Kit. RPLIDAR is used for object detection.

## Architecture

```
Laptop (Mission Control)                        Jetson (Onboard)
  mission_control/                                onboard_software/
  ├── control.py  (pygame event loop)             ├── robot.py  (50Hz orchestrator)
  └── client.py  ────────TCP/IP:8080────────>     ├── server.py
      (sends commands)                            ├── teleOp.py
      (receives telemetry)        <────────────   ├── auto.py
                                                  ├── robot_params.py
                                                  ├── subsystems/
                                                  │   ├── drivetrain.py
                                                  │   ├── auger.py
                                                  │   └── perception.py
                                                  └── logs/

                        library/  (shared by both)
                        ├── protocol.py
                        ├── streaming.py
                        ├── controller.py
                        ├── telemetry_logger.py
                        ├── pid_controller.py
                        ├── util.py
                        └── motor_controller/  (C++ pybind11 / SparkCAN)
```

### Command Flow

```
control.py (pygame input) → client.send_command() → TCP/IP → server.py
→ robot.py main loop calls server.get_command()
→ controller.process_controller_inputs(cmd)
  ├── Axis data → controller.process_axes() → drivetrain drive task (TELEOP only)
  └── Button press → teleop.on_button_event() or auto.on_button_event()
      → subsystem methods (drivetrain, auger)
```

Commands are tuples: either `(mode, axis_data)` or `(mode, button, action)` or simple enums (`Command.READY`, `Command.SHUTDOWN`).

### Library

Lives at the project root. Contains season-agnostic shared infrastructure: networking protocol, gamepad abstraction, and low-level hardware wrappers used by both sides of the system. Code belongs here when it is used by both mission control and onboard software — it will generally take the form of support utilities or shared abstractions that both sides build on top of.

#### Client/Server Protocol

The Jetson and mission control laptop communicate over TCP/IP using a custom ACK-based protocol defined in `library/protocol.py`. Message types: COMMAND, TELEMETRY, ACK. JSON-encoded with message ID deduplication and 500ms ACK timeout with automatic resend.

**Protocol enums:**
- `Command`: READY, SHUTDOWN
- `Mode`: TELEOP, AUTO
- `Button`: A, B, X, Y, LB, RB, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT
- `ButtonAction`: PRESSED, RELEASED

### Onboard Software

Jetson/robot side of the repository.

#### Subsystems

Define each hardware device on the robot and set up functionality and utilities for them. This is where all sensors and motors are declared.

- **drivetrain.py**: Archimedean screw drive (4 motors: FL=7, FR=4, BL=1, BR=2). The screws are mechanically arranged to behave like mecanum wheels, so the code uses mecanum-style math (strafe + rotation). Supports ARCADE and TANK drive modes.
- **auger.py**: Sample collection motor (ID=3). Methods: intake(), outtake(), stop().
- **perception.py**: LiDAR (RPLidar A1M8 on /dev/ttyUSB0) + camera streams. StreamMode: REMOTE (port 5000), LOCAL, or NONE.

#### Robot.py

The heart of the robot — all subsystem init methods are called here to set up subsystems. CAN bus and WiFi settings are configured here. Commands received from control.py are decided and dispatched from here.

**Initialization order:**
1. `setup_network()` (nmcli WiFi connection)
2. `init_can_bus()` (brings up CAN interface — **requires sudo**, exits on failure)
3. Motor controller singleton initialized
4. Drivetrain, Auger subsystems init (configure motors, reset positions)
5. Perception starts (daemon thread)
6. Server starts (daemon thread)
7. Controller, TeleOp, Auto init
8. Waits for `Command.READY` from mission control (60-second timeout)
9. Main loop begins

#### Auto & TeleOp

Where the robot is told to move. They run using a button press collection process, a fast-running loop, and a slower 50Hz periodic loop.

**Important:** Both modes track `_last_update_time` and only call `periodic_loop()` when the 50Hz period (20ms) has elapsed. `motor_controller.update()` is called inside `periodic_loop()` — this is required every cycle to send heartbeat and refresh cached motor feedback. Without it, motors stop responding.

### Mission Control

Command center / laptop side.

#### Control.py

The control side where the user provides inputs to control the robot. Also sets up the viewer for the LiDAR stream.

## Xbox Controller Mapping

```
Axes (pygame index):
  0: Left Stick X   (-1=Left, +1=Right)
  1: Left Stick Y   (-1=Up, +1=Down)
  2: Right Stick X  (-1=Left, +1=Right)
  3: Right Stick Y  (-1=Up, +1=Down)
  4: LT trigger     (-1=released, +1=pressed)
  5: RT trigger     (-1=released, +1=pressed)

Buttons (pygame index):
  0: A    1: B    2: X    3: Y
  4: LB   5: RB
  6: Select/Menu — toggles TELEOP ↔ AUTO mode
  7: Start — triggers SHUTDOWN

D-Pad (hat):
  (0,1): UP   (0,-1): DOWN   (-1,0): LEFT   (1,0): RIGHT
```

**TeleOp button actions:**
- D-Pad UP/DOWN/LEFT/RIGHT: drive forward / backward / strafe left / strafe right
- LB/RB: turn left / turn right
- X: fold out
- Y: auger intake
- A: auger outtake
- B: auger stop

**Auto mode:** The structure and loop are set up, but action methods for autonomous programs have not been added yet.

## Drivetrain Drive Modes

**Arcade drive** (default): Takes 3 axes (forward/backward, strafe, rotation). Uses a non-linear response curve and deadzone.

**Tank drive**: Left/right forward axes are independent, strafe is averaged. Same response curve as arcade.

## Key Configuration

All in `onboard_software/robot_params.py`:

- **CAN bus**: `can0` at 1Mbps
- **Robot IP**: 100.87.109.7, port 8080
- **LiDAR stream**: port 5000, Camera stream: port 5001
- **Update rate**: 50Hz control loop (20ms period)
- **Motor controller is a singleton** per CAN bus — one instance shared across subsystems
- **Feature toggles**: `useDrivetrain`, `useAuger`, `useLidar` — booleans to enable/disable subsystems for partial testing
- **Drive mode**: `drivetrainMode` — `DriveMode.ARCADE` or `DriveMode.TANK`
- **Telemetry toggles**: `useTelemetry`, `logDrivetrainTelemetry`, `logAugerTelemetry`, `logLiDarTelemetry`

## Adding a New Subsystem

1. Create a file in `onboard_software/subsystems/`
2. Constructor takes `mc` (motor controller singleton) as parameter
3. Implement `shutdown()` and `log_data()` methods
4. Use `TelemetryLogger` from `library/telemetry_logger.py` for CSV logging
5. Register and initialize in `robot.py.__init__()`
6. Add enable/telemetry toggles in `robot_params.RobotConfig`

## Code Conventions

- Python 3.10+
- Type hints throughout
- Daemon threads for server, perception, and motor heartbeat (no thread synchronization — be careful adding shared state)
- Telemetry logs written as CSV to `onboard_software/logs/`
- `TYPE_CHECKING` guards used to avoid circular imports

### When devloping 

While the architecture is divided into two sides, `/onboard_software` and `/mission_control`, both computers have access to the full software base. Although each primarily runs its respective side of the code, it is better to keep software implementations simple and implement everything within a single class, rather than creating two classes—one for each side—running the same software implementation. Doing so would only add unnecessary complexity where it is not needed.

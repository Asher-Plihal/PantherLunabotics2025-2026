# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PantherLunabotics 2025-2026 is a robotics competition project (NASA Lunabotics) for Florida Institute of Technology. It is a distributed control system: Python runs on both the operator's laptop (mission control) and the robot's onboard Jetson computer, while a C++ layer (compiled via CMake/pybind11) handles low-level motor controller communication over CAN bus.

## Known Issues

### CAN Bus Interface
The Jetson has two CAN interfaces: `can0` (USB-to-CAN adapter — OpenMoko/Geschwister Schneider, `gs_usb`) and `can1` (onboard Tegra CAN controller, `mttcan`). The USB adapter is `can0`. `robot_params.RobotConfig.Robot_CAN_Interface` is set to `"can0"`. The interface names are pinned permanently via systemd `.link` files in `/etc/systemd/network/` (`10-can-usb.link` and `11-can-onboard.link`). If the CAN bus stops working after a reboot, bring it up manually:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

Verify with `ip link show can0` — look for `state UP` and `LOWER_UP`. Use `candump can0` to verify frames are arriving from the SPARK MAXes.

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

Full guidebook: `ucf-lunabotics-guidebook-2026.txt` — **do not read the full guidebook unless specifically asked.** The software-relevant rules are summarized below. The guidebook is mostly administrative (applications, papers, eligibility, awards) and will waste context window.

### Arena Layout

**Coordinate system:** Origin (0,0) at INGRESS corner (top-right, where the arena divider meets the right wall). +X = west (left across arena), +Y = south (down). Berm center is at X=6.80 m, Y=3.57 m from INGRESS. The fiducials bar is on the right wall at the INGRESS end.

```
Top-down view. Origin at INGRESS (top-right corner). X → left, Y → down.
Bin width: 8.10 m. Bin height: 4.57 m. Judge platform: 1.00 m (left, not drivable).

  INGRESS (0,0)
  ┌──────────────────────────────────────────┬──────────────┐◄── Fiducials bar
  │         OBSTACLE ZONE  (4.1 m wide)      │ STARTING     │ (right wall, 40 cm
  │  ○ brown boulders (30-40 cm, min 3)      │ ZONE  2 m    │  above regolith,
  │  ○ blue craters  (40-50 cm, min 3)       │ [Robot here] │  0.5–2.0 m from top)
  │  Randomly placed before each run.        │              │
  │                    ┌─────────────────────┘              │
  │  CONSTRUCTION      │                                    │
  │  ZONE              │    OBSTACLE ZONE (continued)       │
  │  ┌──────────────┐  │                                    │
  │  │  BERM ZONE   │  │                                    │
  │  │  1.5 × 0.9 m │  │                                    │
  │  │  (red box)   │  │                                    │
  │  └──────────────┘  │         EXCAVATION ZONE  (4 m)     │
  │    2.6 m from left │                                    │
  └────────────────────┴────────────────────────────────────┘
  ◄── 1.00 m ──►◄──────────────── 8.10 m ─────────────────►
  (judge platform)
```

- Regolith: LHS-2E (Lunar Highlands Simulant), ~90 cm deep
- **Excavation Zone overlaps with Starting Zone** — robot may excavate anywhere in either zone
- Regolith for the berm must come from the Excavation Zone or Starting Zone only (not Obstacle or Construction zones)
- Bulldozing (pushing material with a blade from excavation zone through to berm) is explicitly permitted
- Only berm volume inside the red box (1.5 m × 0.9 m) counts toward scoring
- Robot must not push or move obstacles in the Obstacle Zone (only permitted in Construction Zone)
- Robot must avoid craters in Obstacle Zone (no filling them in)

### Obstacles
- **Boulders:** Min 3, randomly placed each run, ~30–40 cm diameter, varying heights. May also appear in Excavation Zone (no larger than Obstacle Zone boulders).
- **Craters:** Min 3, varying depth/width, up to 40–50 cm wide/deep, in Obstacle Zone only
- **No permanent central column** (that was the old NASA arena)
- **Penalty:** 30 pts per rock contact or crater crossing in Obstacle Zone during autonomous operation (max 90 pts)

### Robot Constraints
- Max mass: 80 kg (includes all onboard comms/video equipment and navigational aid system)
- Stowed volume: 150 cm × 75 cm × 75 cm (orientation team's choice; may expand after run starts)
- E-stop button required: min 40 mm diameter COTS red button, highest practical location, one push stops motion AND disconnects batteries from all controllers
- Resetting E-stop alone does NOT resume operation — a second deliberate action is required
- Power logger must be wired between battery and kill switch (30-pt BCP Energy penalty if not)
- Min 4 lifting points, clearly marked

### Timing
- 10 min setup, **15 min competition run**, 5 min removal
- Robot must move within 5 min of timer start or run is terminated
- Loss of locomotion for 5 min = run terminated
- Two attempts allowed per team; scores from both runs are **added together** (cumulative)

### Communications
- IEEE 802.11 WiFi only, assigned SSID "Team_##", encryption required
- Bandwidth measurements are **not performed at UCF** (only at KSC)
- Each team provides their own WAP router placed on a shelf next to the EXOLITH network drop
- All comms to robot via team WAP + EXOLITH ethernet cable to MCC only — **no backchannel wireless connections** (disqualification)
- Bluetooth: Class 2 & 3 only (max 2.5 mW EIRP). Class 1, Zigbee/802.15.4 at 2.4 GHz, and power amplifiers all prohibited
- External WiFi antenna required on robot
- Competition runs on WiFi Channels 1 and 11; RoboPits use 5 GHz only (2.4 GHz off by default)

### Autonomy Rules
**Allowed sensors:** IMUs (compass feature must be disabled), cameras, fiducial targets/beacons (starting zone only), infrared sensors, Hall Effect sensors, proximity detectors

**Prohibited:** GPS, IMU-enabled GPS, compasses (analog/digital), touch sensors, ultrasonic proximity sensors

**Walls:** May be used for localization when sensed naturally, provided **no a priori information** about wall dimensions or location is used. Simple wall-tracking offsets (e.g. "follow wall at 0.5 m") are not allowed. Teams must be able to prove to judges that their autonomy does not inappropriately use wall information.

**Beacons/fiducials:** May only be placed in the Starting Zone — attached to the designated 80/20 fiducial bar (matte black, 1.00" × 1.00" T-slot, mounted ~40 cm above regolith on the right wall, starting 0.5 m from INGRESS and extending 1.5 m) or anywhere in the regolith within the starting zone. Tape, clamps, and rods are allowed; screws or fasteners requiring holes are not.

**During autonomous operation:** telemetry allowed for health monitoring only, all team members hands-free (no touching laptops, controllers, etc.), must announce start and completion of every autonomy attempt to MCJ before initiating, cannot update autonomy program between runs to account for obstacle locations.

**Autonomy scoring tiers (allowable combinations):**
- Excavation only: 75 pts
- Dump only: 50 pts
- Travel only: 250 pts
- Excavation + Dump: 125 pts
- Dump + Travel: 300 pts
- Excavation + Dump + Travel: 375 pts
- Full autonomy (one complete cycle): 450 pts
- Full autonomy (entire run, min 2 cycles): 600 pts

Excavation/Dump/Travel points cannot be combined with Full Autonomy scores.
Travel attempt must be made at the **start of the run** (first time leaving Starting Zone) for maximum points; a 50-pt penalty applies if attempted after traversing the Obstacle Zone in remote control.

### Berm Scoring
- Scored by volume within the 1.5 m × 0.9 m target area (volumetric LiDAR scan before/after)
- Productivity by mass (BCP Mass): cm³ berm / min / kg × **4.4 coefficient**
- Productivity by energy (BCP Energy): cm³ berm / min / Wh × **1.5 coefficient**
- Camera bandwidth score: 0 cameras used = 120 pts, 1 camera = 60 pts, 2 cameras = 0 pts
- Dust Tolerant Design: up to 60 pts (drivetrain enclosed 20 pts, active dust control 20 pts, custom sealing 20 pts)
- Dust Free Operation: up to 30 pts (driving 5 pts, digging 20 pts, transfer without spillage 5 pts)

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

#### Subsystem Ownership

Subsystems (drivetrain, auger) and controllers (PIDDrive) use ownership locks to prevent TeleOp and AutoTasks from simultaneously driving the same hardware. AutoTasks claim ownership when they start and release when they finish, blocking conflicting commands. Telemetry prints all ownership events (CLAIMED, DENIED, RELEASED, FORCE_RELEASED, MOTOR_CALL_DENIED) for debugging.

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
- All classes, methods, and functions must have docstrings.

### When devloping 

While the architecture is divided into two sides, `/onboard_software` and `/mission_control`, both computers have access to the full software base. Although each primarily runs its respective side of the code, it is better to keep software implementations simple and implement everything within a single class, rather than creating two classes—one for each side—running the same software implementation. Doing so would only add unnecessary complexity where it is not needed.

**UWB:** The `uwbs/` folder contains UWB-related code and documentation. When working on anything UWB-related, read `uwbs/UWB.md` first.

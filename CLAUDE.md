# CLAUDE.md

## Project Overview

PantherLunabotics 2025-2026 is a robotics competition project (NASA Lunabotics) for Florida Institute of Technology. It is a distributed control system: Python runs on both the operator's laptop (mission control) and the robot's onboard Jetson computer, while a C++ layer (compiled via CMake/pybind11) handles low-level motor controller communication over CAN bus.

The robot's job is to drive across a regolith arena, excavate ~45 cm of crushed-basalt simulant, carry it through an obstacle field, and deposit it in a scoring berm — autonomously where possible.

## Hardware Overview

- **Onboard processor:** Jetson Orin Nano Super Developer Kit (Ubuntu 22.04, aarch64)
- **Drivetrain:** four NEO motors driving four Archimedean screws. The screws are mechanically arranged to behave like mecanum wheels, so the code uses mecanum-style math (strafe + rotation).
- **Auger:** one NEO motor for intake/outtake of regolith, plus a linear actuator (driven by an Arduino Nano over USB serial) that pivots the auger between transport, intake, and dump angles.
- **Motor controllers:** REV SPARK MAX over CAN bus (1 Mbps), accessed through a USB-to-CAN adapter using the SparkCAN C++ library wrapped with pybind11.
- **Localization (primary):** UWB (Ultra-Wideband) trilateration — two Haorutech ULA1 anchors mounted on arena walls, two ULA1 tags on the robot. Provides absolute (x, y, heading) without GPS. See `uwbs/UWB.md` for details.
- **Vision:** TBD. RPLidar A1 was used previously but is being removed; the team is currently evaluating a camera-based replacement. Treat any remaining lidar code as legacy.
- **Operator interface:** Xbox-style gamepad on the laptop running `mission_control/control.py`.

## Known Issues

### CH340 Auto-Reset on UWB Tags

Opening a serial port to a ULA1 tag can drop the ESP32 into download mode and hang it. The fix is implemented in `library/uwb_localizer.py` and the test scripts in `tests/`. See `uwbs/UWB.md` "Known Issues" for the full explanation.

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

When investigating a bug, **prove the root cause before implementing a fix.** Do not guess at solutions — use telemetry logs, print statements, CAN bus data, UWB serial dumps, or any other available evidence to confirm what is actually going wrong. Only once you have verified the problem should you implement the fix. A fix without proof is just a guess.

## Code Review

Every time you write code, review the implementation to ensure it is efficient, clean, and correct. Make the smallest viable change — do not modify what does not need to be changed.

## Build & Run

Code is developed on a Windows laptop or via SSH into the Jetson computer running Ubuntu Linux. Many files and commands are Linux-specific and will only run correctly on the Jetson. All files exist on both machines as long as the GitHub repo is synced.

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

**Cross-boundary imports (intentional):** `mission_control/client.py` and `mission_control/control.py` add `onboard_software/` to `sys.path` so they can import `robot_params`. This is intentional — `robot_params.py` is the single source of truth for all robot configuration and is shared across both sides this way. **Do not refactor this** — it is a deliberate decision, not an architectural defect.

## Competition Rules (Software-Relevant)

Full guidebook: `ucf-lunabotics-guidebook-2026.txt` — **do not read the full guidebook unless specifically asked.** The software-relevant rules are summarized below. The guidebook is mostly administrative (applications, papers, eligibility, awards) and will waste context window.

### Arena Layout

**Reference images:** `field/full_field.png` is an image of the complete field with all zones, obstacles, and dimensions labeled. `field/field.png` is an additional field diagram used for `library/Panther_dashboard.py`.

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
                                                  │   ├── perception.py   (camera + legacy lidar)
                                                  │   └── dashboard.py    (telemetry sender)
                                                  ├── autoTasks/
                                                  │   ├── excavation.py
                                                  │   └── dump.py
                                                  ├── external_controllers/
                                                  │   └── nano_linear_actuator.ino
                                                  └── logs/

                        library/  (shared by both)
                        ├── protocol.py             (TCP message protocol + enums)
                        ├── streaming.py            (TCP byte stream for video frames)
                        ├── controller.py           (gamepad command dispatch)
                        ├── subsystem.py            (Subsystem base class + ownership)
                        ├── auto_task.py            (AutoTask base class + state machine)
                        ├── pid_controller.py       (single-axis PID)
                        ├── pid_drive.py            (field-centric drive-to-point)
                        ├── uwb_localizer.py        (UWB trilateration + serial reader)
                        ├── Panther_dashboard.py    (laptop-side field viewer)
                        ├── telemetry_logger.py     (CSV writer)
                        ├── util.py
                        └── motor_controller/       (C++ pybind11 / SparkCAN)

                        uwbs/                    (UWB hardware + research)
                        ├── UWB.md                  (read this for anything UWB)
                        ├── ULA1/                   (datasheets, sample firmware)
                        └── DW1000-Arduino/         (Arduino library for tag firmware)

                        tests/                   (used for development)
```

### Command Flow

```
control.py (pygame input) → client.send_command() → TCP/IP → server.py
→ robot.py main loop calls server.get_command()
→ controller.process_controller_inputs(cmd)
  ├── Axis data → controller.process_axes() → drivetrain drive task (TELEOP only)
  └── Button press → teleop.on_button_event() or auto.on_button_event()
      → subsystem methods (drivetrain, auger) or AutoTask start/stop
```

Commands are tuples: either `(mode, axis_data)` or `(mode, button, action)` or simple enums (`Command.READY`, `Command.SHUTDOWN`).

### Library

Lives at the project root. Contains season-agnostic shared infrastructure: networking protocol, gamepad abstraction, UWB localization, control primitives (PID, drive-to-point), and low-level hardware wrappers used by both sides of the system. Code belongs here when it is used by both mission control and onboard software, or when it is reusable across seasons.

#### Client/Server Protocol

The Jetson and mission control laptop communicate over TCP/IP using a custom ACK-based protocol defined in `library/protocol.py`. Message types: COMMAND, TELEMETRY, ACK. JSON-encoded with message ID deduplication and 500 ms ACK timeout with automatic resend.

**Protocol enums:**
- `Command`: READY, SHUTDOWN
- `Mode`: TELEOP, AUTO
- `Button`: A, B, X, Y, LB, RB, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT
- `ButtonAction`: PRESSED, RELEASED

#### Subsystem Base Class

`library/subsystem.py` provides:

- **Ownership locks** — only one caller (TeleOp, an AutoTask, etc.) can drive a subsystem at a time. AutoTasks claim ownership in `start_auto_task()` and release in `stop_auto_task()`. Telemetry prints all ownership events (CLAIMED, DENIED, RELEASED, FORCE_RELEASED, MOTOR_CALL_DENIED) for debugging.
- **Telemetry helpers** — shared logging utilities used by every subsystem.

Mode transitions in the main loop call `force_release()` on every subsystem so the incoming mode starts with a clean slate.

#### AutoTask Base Class

`library/auto_task.py` is the abstract base for all autonomous tasks. It provides:

- A simple state machine: `transition_to(state)` resets a per-state timer, and `wait_for_event(next_state, condition, timeout=...)` advances when a condition is true or a timeout fires.
- The required interface: `start_auto_task()`, `stop_auto_task()`, `claim_subsystem_ownership()`, `release_subsystem_ownership()`, `run_task_states()`, and `is_finished`.

Concrete tasks live in `onboard_software/autoTasks/` (currently `excavation.py` and `dump.py`).

#### PID Drive

`library/pid_drive.py` implements a field-centric PID drive-to-point for the holonomic drivetrain. Given a target `Position(x, y, heading)`, it computes per-axis PID outputs in the field frame, rotates them into the robot frame using the current heading, and mixes them through `Drivetrain.calculate_arcade_powers()`. It pulls the current pose from a localizer (currently `UWBLocalizer.get_pose()`).

Tolerances default to 5 cm XY / 3° heading. Coefficients live in `RobotConfig.pidXCoeffs / pidYCoeffs / pidHCoeffs` and need on-hardware tuning.

#### UWB Localizer

`library/uwb_localizer.py` reads two ULA1 tag serial streams, parses the `mc ...` packets into per-anchor distances, and trilaterates each tag independently. The tag-left-to-tag-right vector gives heading directly — **no IMU is required for absolute heading.** Outputs `Position(x, y, heading)` for `PIDDrive`.

Anchor positions, tag separation, forward offset, and serial port paths are configured in `RobotConfig` (`uwbAnchorAX/AY/BX/BY`, `uwbTagSep`, `uwbForwardOffset`, `uwbLeftPort`, `uwbRightPort`).

**For any UWB-related work, read `uwbs/UWB.md` first.** It is the authoritative reference for hardware setup, DIP switch addresses, LED status meanings, the CH340 serial-reset workaround, packet format, the trilateration math, and tested noise-mitigation strategies.

### Onboard Software

Jetson/robot side of the repository.

#### Subsystems

Each hardware device on the robot is wrapped in a subsystem class. All subsystems extend `library.subsystem.Subsystem` (ownership + telemetry).

- **drivetrain.py** — Archimedean screw drive (4 NEOs: FL=3, FR=7, BL=2, BR=1). Mecanum-style mixing. Drive modes: ARCADE (default) and TANK. Includes a `calculate_arcade_powers()` helper reused by `PIDDrive`.
- **auger.py** — Sample collection NEO (CAN ID 4) plus a linear actuator that pivots the auger between preset angles (transport / intake / dump). The actuator is driven by an Arduino Nano running `external_controllers/nano_linear_actuator.ino`, communicating over USB serial (`/dev/ttyTHS1`, 9600 baud, `MOVE <inches>` / `POS` / `DONE` line protocol). Includes an `is_full` property that uses a moving-average current threshold to detect a full hopper.
- **perception.py** — Camera + lidar streaming over TCP. **Lidar is being phased out (`useLidar=False` by default)**; the team is evaluating a replacement vision system. The `Lidar` and `CameraStream` classes still exist but should be considered legacy until replaced.
- **dashboard.py** — Sends robot pose / target / UWB overlay data to mission control at 10 Hz so `library/Panther_dashboard.py` can render it. Gated by `RobotConfig.fieldDashboard`.

#### Robot.py

The heart of the robot — all subsystem init methods are called here to set up subsystems. CAN bus and WiFi settings are configured here. Commands received from `control.py` are decided and dispatched from here.

**Initialization order:**
1. `setup_network()` — connects to the WiFi profile selected by `NetworkConfig.SELECTED_NETWORK` via `nmcli` (skips if already connected).
2. `init_can_bus()` — brings up the CAN interface. **Requires sudo**, exits on failure.
3. Motor controller singleton initialized.
4. Drivetrain, Auger subsystems init (configure motors, reset positions).
5. Perception starts (daemon thread).
6. Server starts (daemon thread).
7. Dashboard sender (if enabled).
8. PIDDrive + UWBLocalizer (if `usePIDDrive` is enabled).
9. Controller, TeleOp, Auto, ExcavationTask, DumpTask init.
10. Waits for `Command.READY` from mission control (60-second timeout).
11. Main loop begins.

#### Auto & TeleOp

Where the robot is told to move. They run using a button-press dispatch and a 50 Hz periodic loop.

**Important:** Both modes track `_last_update_time` and only call `periodic_loop()` when the 50 Hz period (20 ms) has elapsed. `motor_controller.update()` is called inside `periodic_loop()` — this is required every cycle to send the heartbeat and refresh cached motor feedback. Without it, motors stop responding.

In TELEOP, joystick axes go directly to `drivetrain.drive_task(...)` and the auger triggers/buttons drive the actuator. In AUTO, button presses start/stop AutoTasks (excavation, dump) and `run_task_states()` advances each active state machine.

### Mission Control

Command center / laptop side.

#### Control.py

The control side where the user provides inputs to control the robot. Reads gamepad events via pygame, sends them through `client.py` over TCP, and optionally launches viewer subprocesses (camera viewer, field dashboard).

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
- DPAD_DOWN: fold-out maneuver (drivetrain)
- A: move auger to **intake** angle
- B: move auger to **transport** angle
- Y: move auger to **dump** angle
- LB: toggle auger intake on/off
- RB: toggle auger outtake on/off
- LT (held): retract linear actuator fully; release freezes at current position
- RT (held): extend linear actuator fully; release freezes at current position

**Auto button actions:**
- LB: toggle the excavation task
- RB: toggle the dump task
- DPAD_UP: command auger to position 1.0 in
- DPAD_DOWN: command auger to position 0.0 in

## Drivetrain Drive Modes

**Arcade drive** (default): Takes 3 axes (forward/backward, strafe, rotation). Uses a non-linear `atan` response curve and an 8% deadzone. Strafing is boosted by 1.1× to compensate for the screw geometry.

**Tank drive**: Left/right forward axes are independent, strafe is averaged across both right-stick axes. Same response curve as arcade.

## Key Configuration

All in `onboard_software/robot_params.py`:

- **Robot dimensions**: `ROBOT_LENGTH = 1.08 m`, `ROBOT_WIDTH = 1.108 m`
- **CAN bus**: `can0` at 1 Mbps
- **Robot IP**: `100.87.109.7`, port 8080
- **Camera stream**: port 5001
- **Update rate**: 50 Hz control loop (20 ms period)
- **Motor controller is a singleton** per CAN bus — one instance shared across subsystems
- **Subsystem toggles**: `useDrivetrain`, `useAuger`, `useLidar` (default `False` — lidar is being phased out)
- **Drive mode**: `drivetrainMode` — `DriveMode.ARCADE` or `DriveMode.TANK`
- **Drivetrain max speed**: `drivetrainMaxSpeed` (clipped at the duty-cycle write)
- **PID drive**: `usePIDDrive`, `pidXCoeffs`, `pidYCoeffs`, `pidHCoeffs`
- **UWB**: `uwbAnchorAX/AY/BX/BY`, `uwbTagSep`, `uwbForwardOffset`, `uwbLeftPort`, `uwbRightPort`, `uwbDashboard` (overlay toggle)
- **Dashboard**: `fieldDashboard` (laptop-side field viewer)
- **Telemetry toggles**: `useTelemetry`, `logDrivetrainTelemetry`, `logAugerTelemetry`, `logLiDarTelemetry`
- **Network selection**: `NetworkConfig.SELECTED_NETWORK` (key into `NETWORKS` dict mapping friendly names to nmcli connection profiles)

## Adding a New Subsystem

1. Create a file in `onboard_software/subsystems/`
2. Subclass `library.subsystem.Subsystem`. Constructor takes `mc` (motor controller singleton) as parameter.
3. Implement the abstract methods: `shutdown()`, `start_logging()`, `stop_logging()`, `log_data()`, `print_telemetry()`.
4. Use `TelemetryLogger` from `library/telemetry_logger.py` for CSV logging.
5. Register and initialize in `robot.py.__init__()`.
6. Add enable/telemetry toggles in `robot_params.RobotConfig`.

## Adding a New Auto Task

1. Create a file in `onboard_software/autoTasks/`.
2. Subclass `library.auto_task.AutoTask`. Define an `Enum` for the task's states.
3. Implement `start_auto_task()`, `stop_auto_task()`, `is_finished`, `run_task_states()`, `claim_subsystem_ownership()`, `release_subsystem_ownership()`.
4. Use `transition_to(state)` to enter a new state and `wait_for_event(next_state, condition, timeout=...)` to advance.
5. Instantiate it in `Robot.__init__()` and wire a button to start/stop it in `auto.py.on_button_event()`.

## Code Conventions

- Python 3.10+ (the codebase uses `match`/`case`, structural type hints, `X | Y` unions).
- Type hints throughout.
- Daemon threads for server, perception, motor heartbeat, dashboard, and UWB serial reading (no thread synchronization beyond locks already in those modules — be careful adding shared state).
- Telemetry logs written as CSV to `onboard_software/logs/`.
- `TYPE_CHECKING` guards used to avoid circular imports.
- All classes, methods, and functions must have docstrings.

### When developing

While the architecture is divided into two sides, `/onboard_software` and `/mission_control`, both computers have access to the full software base. Although each primarily runs its respective side of the code, it is better to keep software implementations simple and implement everything within a single class, rather than creating two classes — one for each side — running the same software implementation. Doing so would only add unnecessary complexity where it is not needed.

**UWB:** When working on anything UWB-related, read `uwbs/UWB.md` first. It is the authoritative reference for hardware setup, DIP switch addresses, LED status meanings, the CH340 serial-reset workaround, packet format, the trilateration math, and tested noise-mitigation strategies.

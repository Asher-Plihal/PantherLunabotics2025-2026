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

**Python dependencies:** `pip install -r requirements.txt`

## Hardware

The robot has an auger for intaking and outtaking regolith (sand/dirt). The drive base uses four Archimedean screws. All motors are REV NEO motors controlled via SparkCAN devices over CAN bus through a USB-to-CAN adapter. The onboard processor (and server side) is a Jetson Orin Nano Super Developer Kit.

## Sensors

RPLIDAR is used for object detection.

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

### Library

Lives at the project root. Contains season-agnostic shared infrastructure: networking protocol, gamepad abstraction, and low-level hardware wrappers used by both sides of the system. Code belongs here when it is used by both mission control and onboard software — it will generally take the form of support utilities or shared abstractions that both sides build on top of.

#### Client/Server Protocol

The Jetson and mission control laptop communicate over TCP/IP using a custom ACK-based protocol defined in `library/protocol.py`. Message types: COMMAND, TELEMETRY, ACK. JSON-encoded with message ID deduplication and 500ms ACK timeout with automatic resend.

### Onboard Software

Jetson/robot side of the repository.

#### Subsystems

Define each hardware device on the robot and set up functionality and utilities for them. This is where all sensors and motors are declared.

- **drivetrain.py**: Archimedean screw drive (4 motors: FL=7, FR=4, BL=1, BR=2). Supports ARCADE and TANK drive modes. Max speed: 0.2.
- **auger.py**: Sample collection motor (ID=3). Methods: intake(), outtake(), stop().
- **perception.py**: LiDAR (RPLidar A1M8 on /dev/ttyUSB0) + camera streams. StreamMode: REMOTE (port 5000), LOCAL, or NONE.

#### Robot.py

The heart of the robot — all subsystem init methods are called here to set up subsystems. CAN bus and WiFi settings are configured here. Commands received from control.py are decided and dispatched from here.

#### Auto & TeleOp

Where the robot is told to move. They run using a button press collection process, a fast-running loop, and a slower 50Hz periodic loop.

### Mission Control

Command center / laptop side.

#### Control.py

The control side where the user provides inputs to control the robot. Also sets up the viewer for the LiDAR stream.

## Key Configuration

- **CAN bus**: `can0` at 1Mbps
- **Robot IP**: 100.87.109.7, port 8080
- **LiDAR stream**: port 5000
- **Update rate**: 50Hz control loop (20ms period)
- **Motor controller is a singleton** per CAN bus — one instance shared across subsystems
- **Central config**: `onboard_software/robot_params.py`

## Code Conventions

- Python 3.10+ with `from __future__ import annotations`
- Type hints throughout
- Daemon threads for server, perception, and motor heartbeat
- Telemetry logs written as CSV to `onboard_software/logs/`

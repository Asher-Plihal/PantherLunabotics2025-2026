# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PantherLunabotics 2025-2026 is a robotics competition project (NASA Lunabotics). It is a distributed control system: Python runs on both the operator's laptop (mission control) and the robot's onboard Jetson computer, while a C++ layer (compiled via CMake/pybind11) handles low-level motor controller communication over CAN bus.

## Feature Workflow
1. Understand the prompt
2. Display a brief summary of the action you will take
3. Proceed with implementation
4. Code review

## Prompts

Prompts will often contain misspellings. Before taking action, read the prompt carefully and correct any misspellings to determine the intended meaning. If a word or phrase is ambiguous and the intent is unclear, ask for clarification before proceeding. If requirements are missing, ask for them as well.

## Code Review

Every time you write code, review the implementation to ensure it is efficient, clean, and correct. Make the smallest viable change — do not modify what does not need to be changed.

## Build Commands

All code is developed on a Windows laptop (the "mission_control" side of the project). The "onboard_software" side runs on a Jetson running Ubuntu Linux. Do not attempt to run build commands or scripts that require Linux or are specific to the Jetson — these cannot be executed on this machine.

## Running the System

### On the robot (Jetson):
```bash
python onboard_software/robot.py
```
Starts the TCP server on port 8080, initializes hardware subsystems, and enters the main 50Hz control loop.

### On the laptop (mission control):
```bash
python mission_control/control.py
```
Reads Xbox controller input via pygame and sends commands to the robot over TCP.

### Hardware/motor testing:
```bash
python onboard_software/motor_config_test.py
```

## Architecture

### Two-computer distributed system
```
Laptop (Mission Control)          Jetson (Onboard)
  control.py                        robot.py
  └── client.py  ──TCP/IP:8080──>   └── server.py
      (sends commands)                  (receives commands)
      (receives telemetry)              (sends telemetry)
```

### `library/` — Shared reusable code
`library/` lives at the project root and contains season-agnostic, reusable modules shared across both the robot and mission control — think of it like an internal package you'd install each season without the season-specific code. It includes:
- **`protocol.py`** — Network protocol: `Connection` base class (TCP + ACK-based messaging), enums for commands (`Command`, `Mode`, `Button`, `ButtonAction`).
- **`controller.py`** — Gamepad input processing (axes and button events).
- **`motor_controller/`** — C++ pybind11 wrapper around SparkCAN (REV SPARK MAX/Flex). Exposes `initialize_motor()`, `set_motor_duty_cycle()`, `get_motor_feedback()`, `update()`. Singleton via `GetInstance()`.

### Onboard software layers
1. **`robot.py`** — Top-level orchestrator. Initializes subsystems, manages mode state (TELEOP vs AUTO), runs the 50Hz event loop.
2. **Modes** — `teleOp.py` (manual gamepad control) and `auto.py` (autonomous, WIP). Both follow the same periodic loop structure.
3. **Subsystems** — `subsystems/drivetrain.py` (4-motor mecanum drive, motor IDs [7,1] left, [4,2] right) and `subsystems/auger.py` (single intake motor, ID 1).
4. **`robot_params.py`** — Shared constants (loop frequency: 50Hz, update periods, motor IDs, network config).

### Mission control layers
- `control.py` — pygame event loop, gamepad axis/button processing, sends command dicts.
- `client.py` — TCP client (thin subclass of `library.protocol.Connection`).

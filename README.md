# Panther Lunabotics 2025–2026

Software for Panther Robotics' entry in the **NASA Lunabotics Challenge**: a robot that excavates regolith simulant, navigates an obstacle arena autonomously or by remote control, and deposits material into a target berm.

## Architecture

A distributed control system in **Python (type-hinted, 3.10+) and C++**:

- **Mission control** (operator laptop) ↔ **Jetson Orin Nano** (onboard) over a custom TCP/IP protocol with ACKs and a 20 ms safety heartbeat
- **C++ motor layer** (pybind11 + CMake) driving CAN-bus motors; Archimedean-screw drivetrain with arcade/tank teleop and nonlinear response curves
- **Perception:** RPLiDAR + camera streams on daemon threads; SLAM (`src/panther_slam`) and UWB localization in progress
- **Telemetry:** per-subsystem CSV logging; ownership locks keep TeleOp and autonomous tasks from commanding the same hardware
- `robot_params.py` is the single source of truth for robot configuration

## Background

Software written by two Panther Robotics programmers working with a PhD student mentor. The project was presented at the FIT Senior Design Showcase 2026, and the team submitted the NASA Systems Engineering Paper. The team competed at the [UCF Florida Space Institute's Exolith Lab](https://sciences.ucf.edu/class/) Challenge in Orlando — the top 10 scoring teams there advance to the Lunabotics finals at NASA's Kennedy Space Center.

## Operations & setup

Startup commands, SSH access, and the current blocker/TODO list live in [OPERATIONS.md](OPERATIONS.md) — split out separately since that's day-to-day team reference, not project introduction.

# Development Status

Living document tracking what's functional, in progress, and not yet implemented.

## Functional
- TeleOp driving (arcade and tank modes)
- Auger control (intake, outtake, stop)
- LiDAR streaming (remote, local, and off modes)
- Client/server protocol with ACK-based reliability
- Telemetry CSV logging (drivetrain, auger, LiDAR)

## Stubs / Not Yet Implemented
- Auto mode — all button handlers are `pass`
- CameraStream — `_run()` is empty
- PID controller — `library/pid_controller.py` exists but is not used anywhere


## To be tested
- Test LOCAL mode streaming with ssh and without as well as NONE and make sure hte lidar still runs 
- Test New wifi setup

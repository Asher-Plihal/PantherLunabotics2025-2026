# Start Up commands

## SSH into onboard rPi

```bash
ssh rmcnasa@100.76.221.110
```
## SSH into onboard jetson

```bash
ssh luna01@100.87.109.7
```

## Start venv on onboard server

```bash
source lunaenv/bin/activate
```

## Start up the robot power up

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
python robot.py
```

## Restart the robot after power up

```bash
python robot.py
```

## Start mission control on laptop

```bash
python control.py
```

# Blockers:

1. Waiting for UWB parts to arrive
2. Full scale robot to be built

# TODO:

1. Set up linear actuator
2. Test joystick drive both ways
2. Test auto tasks without pathing

1. Set up UWB
2. Set up live PID tuning
2. Figure out camera
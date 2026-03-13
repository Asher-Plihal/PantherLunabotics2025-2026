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

# TODO:

# Phase 1: TeleOp/Gen:
1. SSH does not work with local
2. Add loging for lidar like test_lidar.py
3. WiFi (NMCLI)

# Phase 2: Auto/Automations:
1. Order parts
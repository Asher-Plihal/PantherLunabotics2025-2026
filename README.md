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

TODOs at comp:

Stage 1: Test full manual
Motor speeds
Auger speeds

Stage 2: Partial assist
AutoTasks
Auger full auto detect


# TODO:

fix control
fix can
make for live tuning for auger

2. Test auto tasks without pathing

1. Set up UWB
2. Set up live PID tuning
2. Figure out camera

Make sure the can port is can0
---

# CAN Interface Binding (One-Time Jetson Setup)

Pins the USB-to-CAN adapter (`gs_usb`) permanently to `can0` and the onboard Tegra CAN (`mttcan`) to `can1`, so the assignment never changes between reboots.

## Steps (run on Jetson)

**1. Create the link file for the USB adapter:**
```bash
sudo nano /etc/systemd/network/10-can-usb.link
```
Paste:
```ini
[Match]
Driver=gs_usb

[Link]
Name=can0
```

**2. Create the link file for the onboard CAN:**
```bash
sudo nano /etc/systemd/network/11-can-onboard.link
```
Paste:
```ini
[Match]
Driver=mttcan

[Link]
Name=can1
```

**3. Reboot:**
```bash
sudo reboot
```

**4. Verify:**
```bash
ip link show can0   # should be the USB adapter
ip link show can1   # should be the onboard Tegra CAN
```
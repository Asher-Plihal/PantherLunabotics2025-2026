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

Look at ownership system going to have a problem with teleOp and stick drive and pid drive
test robot auger is full

1. Finish up the dashbaord
 - Target for dashbaord should having a heading
 - Read and cleanup
2. Setup UWB
 - Check math
 - Cleanup `/uwb_localizer.py` to work well
 - test it using dashbaord
 - setup to test the uwbs with dashbaord as well
3. Write pathing software
4. Auto Tasks when pathing is done
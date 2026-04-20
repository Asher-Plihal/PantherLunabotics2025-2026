import serial
import time

ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
time.sleep(2)
print("ready")

# move actuator
ser.write(b"MOVE 0.0\n")

while True:
    line = ser.readline().decode(errors='ignore').strip()

    if line.startswith("POS"):
        _, val = line.split()
        print("Current position (in):", float(val))

    if line == "DONE":
        print("Target reached")
        break
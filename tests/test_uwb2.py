import serial
import threading
import time
import re

# CHANGE THESE to your actual detected USB ports
TAG1_PORT = "/dev/ttyUSB0"
TAG2_PORT = "/dev/ttyUSB1"

BAUD = 115200

def parse_distance(line):
    """
    Expected Arduino print format:
      "TAG1,A1,3.25"
      "TAG1,A2,4.88"
    """
    match = re.match(r"(TAG\d+),(A\d+),([\d\.]+)", line)
    if match:
        tag, anchor, dist = match.groups()
        return tag, anchor, float(dist)
    return None

def reader(tag_name, ser):
    print(f"[STARTED] {tag_name} listening on {ser.port}")
    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            parsed = parse_distance(line)
            if parsed:
                tag, anchor, distance = parsed
                print(f"{tag} → {anchor}: {distance:.3f} m")

        except Exception as e:
            print(f"[ERROR] {tag_name}: {e}")
            break

def main():
    # open both serial ports
    ser1 = serial.Serial(TAG1_PORT, BAUD, timeout=0.1)
    ser2 = serial.Serial(TAG2_PORT, BAUD, timeout=0.1)

    # start readers
    t1 = threading.Thread(target=reader, args=("TAG1", ser1))
    t2 = threading.Thread(target=reader, args=("TAG2", ser2))

    t1.daemon = True
    t2.daemon = True

    t1.start()
    t2.start()

    print("Listening to two UWB tags simultaneously...\n")

    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
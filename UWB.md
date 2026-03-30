# UWB (Ultra-Wideband) Research & Integration Notes

## Hardware

**Purchased:** Haorutech UWB Ultra Wideband Positioning Module w/ ESP32 + DWM1000
- Product: https://www.robotshop.com/products/haorutech-uwb-ultra-wideband-positioning-module-w-esp32-dwm1000-arduino?gad_source=1&gad_campaignid=20145188159&gbraid=0AAAAAD_f_xyd2QeL8QUJbNMWtKexMKTdV&gclid=CjwKCAjwyMnNBhBNEiwA-KcguzA9lJCxSRVWnZdtJBUSi9fLwqcgFXmFZaQTYKC6r8gptsoTLA4-DxoCZJgQAvD_BwE
- Chip: Decawave DWM1000 (UWB transceiver)
- Bridge: ESP32 microcontroller handles SPI communication to DWM1000
- Host interface: Serial (UART over USB) to Jetson

**Architecture:** Jetson ←USB/Serial→ ESP32 ←SPI→ DWM1000

## Python Reference Library

**Repo:** https://github.com/fuchst/DW1000_Python_library

### What it does
Python adaptation of the Arduino DW1000 library. Provides:
- Wireless messaging over UWB
- Distance ranging using Single-Sided Two-Way Ranging (SS-TWR)
- Indoor localization via trilateration (Tag + Anchor node model)
- Web-based position visualization (port 8080)

### Interface & API
- **Target platform:** Raspberry Pi 3+ with direct SPI access to DW1000
- **SPI config:** Mode 0, 4 MHz → 7.8 MHz after init, CS=GPIO8, RST=GPIO12, IRQ=GPIO16
- **Key classes:** `DW1000` (low-level SPI), `Tag` (mobile ranging node), `Anchor` (fixed reference node)
- **Ranging protocol:** SS-TWR — `distance = 0.5 * (round_time - reply_time) * SPEED_OF_RADIO`
- **Timestamp resolution:** ~15.65 picoseconds
- **PAN ID:** 0xdeca (default), channels 1-7 (excl. 6)

### Compatibility Warning
**This library will NOT work directly with our hardware.**
The library expects a direct SPI connection to the DW1000 chip. Our module has an ESP32 sitting between the Jetson and the DWM1000 — the ESP32 owns the SPI bus. Communication with the Jetson happens over serial (USB).

### What We Need Instead
We need to communicate with the ESP32 firmware over serial. The ESP32 handles all UWB ranging and sends results back over serial. Protocol/format TBD — must be verified by:
1. Connecting the module via USB
2. Opening a serial terminal (e.g. `screen /dev/ttyUSB1 115200`)
3. Observing raw output to determine the data format

## Deployment Architecture

**3 modules total:**

| Role | Location | Notes |
|------|----------|-------|
| Anchor | Arena wall, starting zone | Fixed, known (x, y) position — single reference point |
| Tag LEFT | Left side of robot | Wired to Jetson via USB |
| Tag RIGHT | Right side of robot | Wired to Jetson via USB |

**How localization works:**
The anchor is at a known (x, y) in arena coordinates. Each tag measures its distance to the anchor independently. Because the two tags are a known fixed distance apart (robot width W), the three distances form a fully constrained triangle — giving both the robot's heading and its position relative to the anchor.

```
        Anchor (known x,y on wall)
           *
          /|\
    d_L  / | \  d_R
        /  |  \
   [TAG_L]---[TAG_R]
       ←  W  →        W = robot width (known constant)
```

**What you can solve for:**
- **Heading:** from the asymmetry between d_L and d_R via law of cosines
- **Position:** anchor is known → robot center position follows from d_L, d_R, and W

**Geometric ambiguity (mirror problem):**
The triangle is symmetric — the math produces two valid solutions that are mirror images across the anchor-to-robot-midpoint axis. With a single anchor you cannot algebraically distinguish which side of that axis the robot is on. Disambiguation options:
1. Use LiDAR scan to match against known arena walls and rule out the impossible solution
2. Use dead-reckoning (wheel odometry / IMU) between updates — if the robot moves consistently, the correct branch stays continuous
3. Add a second anchor on a different wall (preferred long-term — eliminates ambiguity entirely)

**This is our primary localization source.** Robot position is not known at match start and must be determined from UWB before autonomous navigation begins.

**Serial ports (expected):**
- `/dev/ttyUSB0` — LiDAR (already in use)
- `/dev/ttyUSB1` — Tag LEFT
- `/dev/ttyUSB2` — Tag RIGHT

Use udev rules to pin each device to a stable port name (e.g. `/dev/ttyUWB_LEFT`, `/dev/ttyUWB_RIGHT`) by USB vendor/product ID so boot order doesn't matter.

## Open Questions (TODO)
- [ ] What serial baud rate does the ESP32 use? (likely 115200)
- [ ] What is the serial output format? (e.g. `DIST,<id>,<meters>`, JSON, or raw)
- [ ] Does the module act as tag or anchor by default? Can it switch roles over serial?
- [ ] Exact (x, y) position of the wall anchor in arena coordinates (measure and record here)
- [ ] Exact lateral distance between TAG_LEFT and TAG_RIGHT on the robot (measure and record here)
- [ ] Which USB port each tag enumerates on — verify with `dmesg` after plugging in
- [ ] Can it output position (x,y) directly, or only distances to the anchor?

## Integration Plan
1. Verify serial output format by connecting and monitoring raw output
2. Update `UWBSensor` serial parser to match actual format
3. Integrate as a subsystem in `onboard_software/subsystems/uwb.py` (two UWBSensor instances)
4. Add `useUWB` feature toggle to `robot_params.py`, plus anchor position and tag separation constants
5. Implement heading calculation from the two tag distances
6. Use heading + anchor distance for localization in auto mode

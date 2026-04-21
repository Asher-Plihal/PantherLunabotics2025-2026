# UWB (Ultra-Wideband) Research & Integration Notes

## Hardware

**Module:** Haorutech ULA1 — ESP32 + DWM1000 (Decawave UWB transceiver)
- Product: https://www.robotshop.com/products/haorutech-uwb-ultra-wideband-positioning-module-w-esp32-dwm1000-arduino
- User manual: https://www.haorutech.com/download/ULA1_UserManual-EN.pdf

**Architecture:** Jetson <--USB/Serial (CH340)--> ESP32 <--SPI--> DWM1000

The ESP32 owns the SPI bus to the DWM1000 and handles all UWB ranging internally. The Jetson only communicates with the ESP32 over serial (USB) — it never touches the DWM1000 directly.

**Serial:** 115200 baud, 8-N-1

**Connector:** USB (CH340 USB-to-UART bridge — requires CH340 driver on Windows)

**Key specs:**
| Parameter | Value |
|-----------|-------|
| Power | DC 5V via USB |
| Max range | 50m (open area) |
| Ranging accuracy | ±10 cm |
| Frequency domain | 3.5–6.5 GHz |
| Data rates | 110 kb/s, 850 kb/s, 6.8 Mb/s |
| SPI clock | 20 MHz max |
| Antenna | Onboard |
| Module size | 40 × 25 mm |
| Display | None |
| MCU | ESP32 |
| UWB chip | DWM1000 (Decawave) |
| Working temp | -20 to 80°C |

## Built-In Features

The ULA1 is a simpler module than the previous ULM3 — no OLED, no Kalman filter switch, and no power bank keep-alive switch. Configuration is done entirely via a 4-bit DIP switch.

- **Arduino-compatible firmware:** The ULA1 ships with Arduino-based firmware on the ESP32. The Arduino library (DW1000-Arduino) is included in the ULA1 package for custom firmware development.
- **TWR ranging:** Uses Two-Way Ranging (TWR) to measure distances between tags and anchors.
- **TDMA scheduling:** Multiple tags share airtime via time-division multiplexing.
- **No on-device Kalman filter:** Unlike the ULM3, there is no built-in Kalman filter. Raw distances are output directly. Software-side filtering is required if smoothing is desired.
- **No tag position computation:** The ULA1 does not compute (x, y, z) position on-device. All localization math must be done in software (handled by `UWBLocalizer`).

## Role Configuration

Role and address are set via a **4-bit DIP switch**. Power-cycle required after changing switches.

| Switch | Function | ON | OFF |
|--------|----------|----|-----|
| S4 | Role | Anchor | Tag |
| S5 | Address bit 0 (LSB) | 1 | 0 |
| S6 | Address bit 1 | 1 | 0 |
| S7 | Address bit 2 (MSB) | 1 | 0 |

Address is a 3-bit binary number encoded across S5–S7.

| Module | S4 | S5 | S6 | S7 | Address |
|--------|----|----|----|----|---------|
| Anchor A | ON | OFF | OFF | OFF | A0 |
| Anchor B | ON | OFF | OFF | ON  | A1 |
| Tag LEFT | OFF | OFF | OFF | OFF | T0 |
| Tag RIGHT | OFF | OFF | OFF | ON  | T1 |

- Once configured, anchors only need USB power — no serial data connection required during a match.
- Tags stay connected to the Jetson via USB throughout the match.

> From the manual: "During navigation mode, the tag needs to be connected to the PC while other anchors only need to power on."

### Serial Commands (Downlink Protocol)

The ULA1 does **not** expose downlink serial commands — configuration is done entirely via DIP switches, not serial. The device outputs range data continuously once powered; no initialization commands are needed from the Jetson.

### LED Status Indicators

LED behavior is not documented in the ULA1 manual. Observed behavior:
- **Blue LED** — module is configured as a **Tag**
- **Yellow LED** — module is configured as an **Anchor**
- **Red LEDs blinking in unison** — Tag is actively ranging (anchors detected)
- **Red LEDs blinking in sequence (up/down chase)** — Anchor is active and broadcasting
- **Red LEDs solid** — ESP32 stuck in download/bootloader mode (see Known Issues)

## Jetson Driver Setup (CH340)

The Jetson's Tegra kernel does not include the CH340 driver by default. Build and load it from source once:

```bash
sudo apt install build-essential
git clone https://github.com/juliagoda/CH341SER.git
cd CH341SER
make
sudo make load      # loads for current session
sudo make install   # persists across reboots
```

The `linux-headers` package is already present at `/usr/src/linux-headers-5.15.148-tegra-ubuntu22.04_aarch64/`. After loading, unplug and replug the tag USB cable — it should appear as `/dev/ttyUSB*`.

> **Note:** `sudo make load` only lasts until reboot. `sudo make install` was run to make it permanent.

## UWB Setup

Step-by-step instructions to configure all 4 ULA1 modules for our 2-anchor, 2-tag system.

**Before starting:** Disconnect power from all modules. DIP switch changes only take effect after a power cycle.

### Module 1 — Anchor A (address A0)

1. Set DIP switches:
   - S4: **ON** (Anchor)
   - S5: OFF, S6: OFF, S7: OFF (address = 000 = A0)
2. Connect a USB power bank (5V) via USB cable.
3. Power on. No serial connection needed — this module runs standalone on battery power during the match.

### Module 2 — Anchor B (address A1)

1. Set DIP switches:
   - S4: **ON** (Anchor)
   - S5: OFF, S6: OFF, S7: **ON** (address = 001 = A1)
2. Connect a USB power bank (5V) via USB cable.
3. Power on. Standalone — no data connection needed.

### Module 3 — Tag LEFT (address T0)

1. Set DIP switches:
   - S4: **OFF** (Tag)
   - S5: OFF, S6: OFF, S7: OFF (address = 000 = T0)
2. Connect via USB to the Jetson. This provides both power and data.
3. Power on. LED should indicate ranging once anchors are on.

### Module 4 — Tag RIGHT (address T1)

1. Set DIP switches:
   - S4: **OFF** (Tag)
   - S5: OFF, S6: OFF, S7: **ON** (address = 001 = T1)
2. Connect via USB to the Jetson.
3. Power on. LED should indicate ranging once anchors are on.

### Power Summary

| Module | Power source | Notes |
|--------|-------------|-------|
| Anchor A | USB power bank (5V) | Mounted on arena wall, no data cable |
| Anchor B | USB power bank (5V) | Mounted on arena wall, no data cable |
| Tag LEFT | Jetson USB port | No battery needed |
| Tag RIGHT | Jetson USB port | No battery needed |

**The two anchors need dedicated USB power banks** because they are mounted on the arena walls away from the robot and have no wired connection to the Jetson. Any 5V USB power bank works. Unlike the ULM3, the ULA1 does not have a power-bank keep-alive switch — if the power bank auto-shutoff is triggered by low load, use a power bank with a always-on mode or a USB dummy load.

### Verification

After powering on all 4 modules:
1. Both anchor modules should be powered and outputting range data (verify with a USB serial monitor if needed).
2. Both tag LEDs should indicate successful ranging. If no ranging occurs, check that all modules are on the same channel and within 50m.
3. On the Jetson, run `dmesg | grep ttyUSB` to identify serial port assignments. Then run `tests/test_uwb_hardware.py` to see raw packets from both tags.

## Deployment Architecture

**4 modules total:**

| Role | Location | Connection |
|------|----------|------------|
| Anchor A | Left wall of starting zone | Power only — standalone |
| Anchor B | Right wall of starting zone | Power only — standalone |
| Tag LEFT | Left side of robot | USB → Jetson `/dev/ttyUSB1` |
| Tag RIGHT | Right side of robot | USB → Jetson `/dev/ttyUSB2` |

The two anchors are on separate perpendicular walls of the starting zone, giving two independent triangulation baselines.

**Note:** USB port numbers depend on plug order at boot. Always verify with `dmesg | grep ttyUSB` after connecting. Use udev rules to pin stable names (e.g. `/dev/ttyUWB_LEFT`, `/dev/ttyUWB_RIGHT`) by USB vendor/product ID.
- `/dev/ttyUSB0` is reserved for LiDAR.

## How Localization Works

Four measurements per cycle — each tag ranges to each anchor independently:

| Measurement | Meaning |
|-------------|---------|
| d_LA | TAG_LEFT → Anchor A |
| d_RA | TAG_RIGHT → Anchor A |
| d_LB | TAG_LEFT → Anchor B |
| d_RB | TAG_RIGHT → Anchor B |

With 2 labeled tags and 2 anchors at known positions there are 4 measurements and only 3 unknowns (x, y, theta) — the system is over-constrained. **Absolute heading is derivable from UWB alone, no IMU required.**

```
  Anchor A (left wall)              Anchor B (right wall)
        *                                    *
        |  \                            /    |
     d_LA   \ d_RA              d_LB /      d_RB
        |     \                  /           |
        |      \                /            |
   [TAG_LEFT]-------------------------------[TAG_RIGHT]
                        <-- W -->
```

**Method — independent trilateration of each tag:**

Each tag's position is found by intersecting two distance circles (one from each anchor). The tags are labeled, so the heading comes directly from the TAG_LEFT → TAG_RIGHT vector.

```
TAG_LEFT  = intersect( circle(Anchor A, d_LA),  circle(Anchor B, d_LB) )
TAG_RIGHT = intersect( circle(Anchor A, d_RA),  circle(Anchor B, d_RB) )
```

Each trilateration yields 2 candidate points → 4 candidate (L, R) pairs total. The valid pair is the one where the separation equals W. From there:

- **Position:** robot center = midpoint of TAG_LEFT and TAG_RIGHT
- **Heading theta:** atan2(-(ry-ly), (rx-lx)) — derived from the lateral tag vector (see Math section)

**Residual ambiguity:** If two pairs both satisfy the W constraint (rare, occurs near the anchor-to-anchor diagonal), resolve using arena bounds or the previous position estimate.

**Role of IMU:** Not required for heading. Useful as a tiebreaker for residual ambiguity and for angular velocity estimation between UWB cycles.

**This is our primary localization source.** Robot position and heading are not known at match start and must be determined from UWB before autonomous navigation begins.

## Math

### Coordinate System

+X = east (right), +Y = north (up). Heading: 0 deg = north, clockwise positive (90 deg = east). Arena origin at bottom-left corner.

### Notation

| Symbol | Meaning |
|--------|---------|
| d_LA, d_RA | Distances from TAG_LEFT, TAG_RIGHT to Anchor A (metres) |
| d_LB, d_RB | Distances from TAG_LEFT, TAG_RIGHT to Anchor B (metres) |
| W | TAG_LEFT-to-TAG_RIGHT separation (metres) — measure on robot |
| (ax, ay), (bx, by) | Anchor A and B positions in arena coordinates |
| (lx, ly), (rx, ry) | TAG_LEFT and TAG_RIGHT positions (computed) |
| (x, y) | Robot center in arena coordinates (metres) |
| theta | Robot heading (degrees, 0 deg = north, CW positive) |
| f | Forward offset: distance tag midpoint is ahead of robot center |

### Step 1 — Trilaterate each tag

Shown for TAG_LEFT (r1 = d_LA, r2 = d_LB). Repeat identically for TAG_RIGHT (r1 = d_RA, r2 = d_RB).

```
d_AB = sqrt((bx-ax)^2 + (by-ay)^2)           # anchor separation

a = (r1^2 - r2^2 + d_AB^2) / (2 * d_AB)      # signed distance along A->B to radical axis
h = sqrt(max(r1^2 - a^2, 0))                  # half-chord length (clamp noise)

# Foot of perpendicular on A->B:
Px = ax + a * (bx - ax) / d_AB
Py = ay + a * (by - ay) / d_AB

# Perpendicular unit vector:
Qx = -(by - ay) / d_AB
Qy =  (bx - ax) / d_AB

# Two candidate positions:
P1 = (Px + h*Qx,  Py + h*Qy)
P2 = (Px - h*Qx,  Py - h*Qy)
```

If d_AB > r1 + r2 or d_AB < |r1 - r2|, the circles do not intersect — discard the reading (sensor noise or tag out of range).

### Step 2 — Select valid pair

Test all 4 combinations (TAG_L in {P1_L, P2_L}, TAG_R in {P1_R, P2_R}):

```
For each (L_candidate, R_candidate):
    sep = sqrt((rx - lx)^2 + (ry - ly)^2)
    if |sep - W| < tolerance:          # ~10-20 cm for ULA1 noise
        valid pair
```

If two pairs pass the test, select the one closest to the previous position estimate.

### Step 3 — Robot center

```
x = (lx + rx) / 2
y = (ly + ry) / 2
```

With a forward offset f (tag midpoint mounted ahead of robot center, positive forward):

```
x_center = x - f * sin(theta)          # apply after Step 4
y_center = y - f * cos(theta)
```

### Step 4 — Heading

The TAG_LEFT → TAG_RIGHT vector points along the robot's right-lateral axis. At heading theta, the right-lateral unit vector is (cos theta, -sin theta), so:

```
vx = rx - lx = W * cos(theta)
vy = ry - ly = -W * sin(theta)

theta = atan2(-vy, vx)    # standard math atan2; result in degrees
```

Normalize to [0 deg, 360 deg).

**Verification:**
- Facing north (theta = 0 deg): TAG_R is east of TAG_L → vx > 0, vy ~ 0 → atan2(0, +) = 0 deg
- Facing east (theta = 90 deg): TAG_R is south of TAG_L → vx ~ 0, vy < 0 → atan2(+, 0) = 90 deg
- Facing south (theta = 180 deg): TAG_R is west of TAG_L → vx < 0, vy ~ 0 → atan2(0, -) = 180 deg

### Output

| Value | Type | Description |
|-------|------|-------------|
| x | float (metres) | Robot center east coordinate |
| y | float (metres) | Robot center north coordinate |
| theta | float (degrees) | Robot heading, 0 deg = north, CW positive |

## Serial Output Format

Each tag emits one line per cycle containing raw distances to all configured anchors. Format (from ULA1 manual):

```
mc 0f 00000663 000005a3 00000512 000004cb 095f c1 0 a0:0
      RANGE0    RANGE1    RANGE2    RANGE3
```

| Field | Example | Description |
|-------|---------|-------------|
| HEAD | `mc` | Fixed packet header |
| MASK | `0f` | Bitmask of valid ranges. `0x03` (0000 0011) = RANGE0, RANGE1 valid. `0x0f` = RANGE0–3 valid. |
| RANGE0–RANGE3 | `00000663` | Distance to Anchor A0–A3 in hex mm. `0x663` = 1635 mm = 1.635 m. |
| NRANGES | `095f` | Message flow counter, accumulated 0x0–0xFFFF |
| RSEQ | `c1` | Range sequence number, accumulated 0x0–0xFF |
| DEBUG | `0` | Reserved debug field |
| rIDt:IDa | `a0:0` | Role + IDs. Format: `{role}{tagID}:{anchorID}`. `a` = anchor, `t` = tag. |
| END | `\r\n` | Packet terminator |

**Differences from ULM3 (STM32+DWM3000) format:**
- The ULA1 has only 4 RANGE fields (not 8) — no RANGE4–RANGE7 with `ffffffff`.
- No RANGTIME (MCU timestamp) field.
- No DIAGNOSIS field at end (anchor RX power).
- No `$K` or `$NK` tag-position line — the ULA1 does not compute on-device position.

For our 2-anchor setup: RANGE0 = Anchor A (A0), RANGE1 = Anchor B (A1). RANGE2–RANGE3 will be `ffffffff` if no A2/A3 anchor is present.

## Code

- `library/uwb_localizer.py` — all UWB math and hardware access. Full localizer setup should return accurate heading, x, and y.
- `tests/test_uwb_hardware.py` — minimal raw serial test. Prints everything both tags send over USB. Use this first to verify the serial output format and baud rate before relying on the parser.
- `tests/test_uwb_reading_serial.py` — serial reading + parsing test. Prints parsed distances in metres.

## Noise Mitigation

The ULA1 has ±10 cm accuracy (vs ±5 cm on the ULM3) and **no built-in Kalman filter**. Software-side filtering is more important with this module.

1. **Exponential moving average** — simple low-pass filter on (x, y, theta) outputs. Easiest to implement (~5 lines of code), minimal latency at moderate smoothing factors. Start here.
2. **Extended Kalman Filter (EKF)** — fuses UWB ranges with a motion model over time. Handles missed readings, reduces jitter, and can incorporate IMU data later. Upgrade to this if EMA isn't sufficient.

The ULA1 outputs raw unfiltered distances, so both EMA and EKF operate directly on what the module produces.

## Open Questions

1. **MASK field parsing:** The parser currently ignores the MASK field. We should validate that RANGE0 and RANGE1 bits are set (mask & 0x03 == 0x03) before trusting the distances. If either bit is unset, the corresponding range is invalid.
2. **Packet field count:** Confirm whether RANGE2–RANGE3 fields are always present in the packet even with only 2 anchors, or if the packet is shorter. This affects `_parse()` field indexing.
3. **Baud rate confirmation:** 115200 is the assumed default based on the CP2102 driver and ULM3 precedent. Confirm with a serial monitor if there is any doubt.
4. **Power bank auto-shutoff:** The ULA1 has no keep-alive switch. If anchors shut off during a match, investigate power banks with always-on modes or add a USB dummy load.
5. ~~**LED behavior:**~~ Resolved — blue = Tag, yellow = Anchor, red LEDs in unison = Tag ranging, red LEDs in sequence = Anchor broadcasting.

## Known Issues

### CH340 Auto-Reset on Serial Port Open

**Problem:** Opening a serial port on Linux asserts DTR and RTS via the CH340, which triggers the ESP32 auto-reset circuit (DTR → GPIO0, RTS → EN). If GPIO0 is held LOW during reset, the ESP32 enters download/bootloader mode (`boot:0x3 DOWNLOAD_BOOT`) and hangs waiting for firmware — it never runs the ranging firmware.

**Cause:** Hardware — the CH340 auto-reset circuit uses capacitors to pulse GPIO0 and EN when the port is opened. This cannot be fixed in firmware.

**Fix:** After opening the serial port in Python, manually drive the correct reset sequence: set `dtr=False` (GPIO0 HIGH = normal boot), assert `rts=True` (EN LOW = hold in reset), release `rts=False` (EN HIGH = boot starts), then wait ~1 second for the ESP32 to fully boot before reading.

```python
ser = serial.Serial(port, 115200, timeout=1.0)
ser.dtr = False   # GPIO0 HIGH → normal boot mode
ser.rts = True    # EN LOW → hold in reset
time.sleep(0.1)
ser.rts = False   # EN HIGH → release, ESP32 boots
time.sleep(1.0)   # wait for boot + ranging init
```

This is implemented in `tests/test_uwb_hardware.py` and `tests/test_uwb_reading_serial.py` and must also be applied in `library/uwb_localizer.py`.

### Intermittent Download Mode with Two Tags

**Problem:** When both tag serial ports are opened simultaneously (two threads), one module occasionally still enters download mode. The two reset sequences race and the timing is not always reliable.

**Fix:** Stagger thread starts by 1.5 seconds so the first module completes its boot before the second port is opened. Both test scripts do this.

### brltty Conflict (Ubuntu 22.04)

**Problem:** The `brltty` accessibility service (Braille TTY) automatically claims CH340 devices on Ubuntu 22.04, immediately disconnecting them.

**Fix:** `sudo apt remove brltty`

## Integration Plan

1. Run `tests/dashboard.py` to validate that the `UWBLocalizer` math in `library/uwb_localizer.py` produces correct (x, y, theta) with manual test values.
2. Connect a tag and run `tests/test_uwb_hardware.py` to confirm live packets match the format in the Serial Output section above. Confirm exact field count.
3. Update `UWBLocalizer._parse()` in `library/uwb_localizer.py` for the ULA1 packet format (4 RANGE fields, no timestamp, no `$K` line). Use `tests/test_uwb_reading_serial.py` for testing before moving to `UWBLocalizer`.
4. Measure anchor positions and tag separation (W). Add them as constants in `robot_params.py`. Run full hardware integration test with all 4 modules active.
5. Move `UWBLocalizer` to `onboard_software/subsystems/uwb_localizer.py`. Add a `useUWB` feature toggle to `robot_params.py`. Call `update_from_hardware()` each cycle in the auto loop.

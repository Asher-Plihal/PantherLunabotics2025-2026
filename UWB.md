# UWB (Ultra-Wideband) Research & Integration Notes

## Hardware

**Module:** Haorutech ULM3 — STM32F103CBT6 + DWM3000 (Qorvo UWB transceiver)
- Product: https://www.robotshop.com/products/haorutech-uwb-ultra-wideband-positioning-module-w-stm32-dwm3000
- User manual: https://www.haorutech.com/download/ULM3_UserManual-EN.pdf

**Architecture:** Jetson <--USB/Serial--> STM32F103 <--SPI--> DWM3000

The STM32 owns the SPI bus to the DWM3000 and handles all UWB ranging internally. The Jetson only communicates with the STM32 over serial (USB) — it never touches the DWM3000 directly.

**Serial:** 115200 baud, 8-N-1 (confirmed — matches manual default)

**Connector:** Micro-USB (data + power)

**Key specs:**
| Parameter | Value |
|-----------|-------|
| Power | DC 3.7V–5V (USB or external power bank / li-ion battery) |
| Max range | 40m (open area) @ 6.8 Mbps |
| Ranging accuracy | ±5 cm |
| Data update frequency | 100 Hz max (adjustable) |
| Frequency domain | 6250–8250 MHz (CH5 / CH9) |
| Bandwidth | 500 MHz |
| Communication rate | 6.8 Mbps |
| Antenna | Onboard ceramic |
| Module size | 27 x 70 mm (including antenna and base) |
| Display | 0.6" OLED (shows firmware version, mode, channel, ID, filter status) |
| MCU | STM32F103CBT6 (or GD32F103CBT6 depending on batch) |
| Working temp | -20 to 70 C |

## Built-In Features

The ULM3 has several features running on the STM32 firmware that the older ULA1 (ESP32/DWM1000) did not have:

- **Kalman filter (S8 DIP switch):** Smooths raw ranging distances on the module itself before outputting them over serial. Reduces jitter from individual noisy measurements by predicting the next distance based on prior readings. Enabled by default — leave it on.
- **On-device position computation:** The tag can compute its own (x, y, z) position internally if you send it anchor coordinates via `$sanccd`. It outputs the result in the `$K` line (`LO = [x, y, z]`). We ignore this and compute position ourselves in `UWBLocalizer` because the tag doesn't know about the second tag and cannot derive heading.
- **OLED display:** Shows module status (role, ID, channel, filter state, update rate) directly on the hardware. Useful for verifying configuration without a serial connection.
- **Power bank keep-alive (S3 DIP switch):** Artificially increases current draw so low-load power banks don't auto-shutoff. Essential for the battery-powered anchors.
- **TDMA multi-tag scheduling:** Multiple tags share airtime via time-division multiplexing. Each tag gets a 10 ms slot. Our 2-tag setup cycles in 20 ms (50 Hz per tag).
- **Antenna delay calibration (`$santdly`):** Fine-tunes ranging accuracy by adjusting the internal antenna delay parameter. Calibrate on-site against a known distance.

## Role Configuration

Role and address are set via the **8-bit DIP switch**. Power-cycle required after changing switches.

| Switch | Function | ON | OFF |
|--------|----------|-----|-----|
| S1 | Reserved | — | — |
| S2 | Max tags / comm period | 1 tag, 10 ms period | 10 tags, 100 ms period |
| S3 | Increase external current (power bank keep-alive) | Enabled | Disabled |
| S4 | Role | Anchor | Tag |
| S5–S7 | 3-bit device address (0–7) | Binary encoding | — |
| S8 | Kalman filter | Enabled | Disabled |

**Default configuration:** 10 tags max, 100 ms period (10 Hz), external current increase ON, Kalman filter ON.

**S2 detail:** At 6.8 Mbps, one tag + 4 anchors takes 10 ms per ranging cycle. Multiple tags use TDMA — total period = 10 ms x number of tags. If a tag is offline, its 10 ms slot outputs empty.

**S3 detail:** DW3000 draws very little current. Most power banks auto-shutoff when load is too low. S3 increases current draw to keep power banks alive.

| Module | S4 | S5 | S6 | S7 | Address |
|--------|----|----|----|----|---------|
| Anchor A | ON | OFF | OFF | OFF | A0 |
| Anchor B | ON | OFF | OFF | ON  | A1 |
| Tag LEFT | OFF | OFF | OFF | OFF | T0 |
| Tag RIGHT | OFF | OFF | OFF | ON  | T1 |

- Once configured, anchors only need power — no serial connection required during a match
- Tags stay connected to the Jetson via USB throughout the match

> From the manual: "During navigation mode, the tag needs to be connected to the PC while other anchors only need to power on."

### Serial Commands (Downlink Protocol)

Commands begin with `$` and end with `\r\n`. Send over serial at 115200 baud.

| Command | Description |
|---------|-------------|
| `$rboot` | Reboot module, outputs startup info |
| `$reset` | Restore all parameters to factory defaults |
| `$santdly,16375` | Set antenna delay (decimal) for ranging calibration. Smaller value = longer measured distance. |
| `$stxpwr,1f1f1f1f` | Set transmit gain (hex) |
| `$sanccd,A0X,A0Y,A0Z,A1X,A1Y,A1Z,...` | Set anchor coordinates on a tag (metres, float). Tag uses these to compute its own position. |
| `$saddr,9` | Set tag ID (overrides DIP switch, tag only) |

### LED Status Indicators

| State | LED |
|-------|-----|
| Tag ranging successfully (1+ anchor responding) | Green blink |
| Tag ranging, no anchor response | Red blink |
| Anchor connected to a tag | Light blue blink |
| Anchor, no tag connected | Light blue steady (on or off) |

### OLED Display

Shows: firmware version, max anchors/tags, update period, UWB air rate, channel, role + ID, Kalman filter status, power bank keep-alive status.

## UWB Setup

Step-by-step instructions to configure all 4 ULM3 modules for our 2-anchor, 2-tag system.

**Before starting:** Disconnect power from all modules. DIP switch changes only take effect after a power cycle.

### Module 1 — Anchor A (address A0)

1. Set DIP switches:
   - S1: OFF (reserved)
   - S2: OFF (10 tags max — matches default, doesn't matter for anchors)
   - S3: ON (power bank keep-alive — anchors run on battery packs)
   - S4: **ON** (Anchor)
   - S5: OFF, S6: OFF, S7: OFF (address = 000 = A0)
   - S8: ON (Kalman filter enabled)
2. Connect a battery pack (5V USB power bank) via micro-USB.
3. Power on. OLED should show `Anc:0`, `CH5`. LED should be light blue (no tags yet).
4. **No USB data connection needed** — this module runs standalone on battery power during the match.

### Module 2 — Anchor B (address A1)

1. Set DIP switches:
   - S1: OFF
   - S2: OFF
   - S3: ON (power bank keep-alive)
   - S4: **ON** (Anchor)
   - S5: OFF, S6: OFF, S7: **ON** (address = 001 = A1)
   - S8: ON
2. Connect a battery pack via micro-USB.
3. Power on. OLED should show `Anc:1`, `CH5`.
4. Standalone — no data connection needed.

### Module 3 — Tag LEFT (address T0)

1. Set DIP switches:
   - S1: OFF
   - S2: ON (1 tag, 10 ms period — faster updates since we only have 2 tags)
   - S3: OFF (powered by Jetson USB, no power bank needed)
   - S4: **OFF** (Tag)
   - S5: OFF, S6: OFF, S7: OFF (address = 000 = T0)
   - S8: ON
2. Connect via micro-USB to the Jetson. This provides both power and data.
3. Power on. OLED should show `Tag:0`, `CH5`. LED should blink green once anchors are on.

### Module 4 — Tag RIGHT (address T1)

1. Set DIP switches:
   - S1: OFF
   - S2: ON (1 tag, 10 ms period)
   - S3: OFF (powered by Jetson USB)
   - S4: **OFF** (Tag)
   - S5: OFF, S6: OFF, S7: **ON** (address = 001 = T1)
   - S8: ON
2. Connect via micro-USB to the Jetson.
3. Power on. OLED should show `Tag:1`, `CH5`. LED should blink green once anchors are on.

### Power Summary

| Module | Power source | Notes |
|--------|-------------|-------|
| Anchor A | Battery pack (5V USB power bank) | S3 = ON to prevent auto-shutoff |
| Anchor B | Battery pack (5V USB power bank) | S3 = ON to prevent auto-shutoff |
| Tag LEFT | Jetson USB port | No battery needed |
| Tag RIGHT | Jetson USB port | No battery needed |

**The two anchors need dedicated battery packs** because they are mounted on the arena walls away from the robot and have no wired connection to the Jetson. Any 5V USB power bank works. Enable S3 (power bank keep-alive) on both anchors — DW3000 draws very little current and most power banks will auto-shutoff otherwise.

### Verification

After powering on all 4 modules:
1. Both anchor OLEDs should show their respective IDs and `CH5`.
2. Both tag LEDs should blink **green** (successfully ranging with anchors). If red, the tags cannot reach the anchors — check that all modules are on the same channel (CH5) and within 40m.
3. On the Jetson, run `dmesg | grep ttyUSB` to identify serial port assignments. Then run `tests/test_uwb_hardware.py` to see raw packets from both tags.

## Deployment Architecture

**4 modules total:**

| Role | Location | Connection |
|------|----------|------------|
| Anchor A | Left wall of starting zone | Power only — standalone |
| Anchor B | Right wall of starting zone | Power only — standalone |
| Tag LEFT | Left side of robot | USB --> Jetson `/dev/ttyUSB1` |
| Tag RIGHT | Right side of robot | USB --> Jetson `/dev/ttyUSB2` |

The two anchors are on separate perpendicular walls of the starting zone, giving two independent triangulation baselines.

**Note:** USB port numbers depend on plug order at boot. Always verify with `dmesg | grep ttyUSB` after connecting. Use udev rules to pin stable names (e.g. `/dev/ttyUWB_LEFT`, `/dev/ttyUWB_RIGHT`) by USB vendor/product ID.
- `/dev/ttyUSB0` is reserved for LiDAR

### UART Alternative

The ULM3 also has onboard UART TTL pins (TX, RX, GND) for secondary development. Connect TX of ULM3 to RX of the target device, and connect GND directly. This is an alternative to USB if USB ports are scarce.

## How Localization Works

Four measurements per cycle — each tag ranges to each anchor independently:

| Measurement | Meaning |
|-------------|---------|
| d_LA | TAG_LEFT --> Anchor A |
| d_RA | TAG_RIGHT --> Anchor A |
| d_LB | TAG_LEFT --> Anchor B |
| d_RB | TAG_RIGHT --> Anchor B |

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

Each tag's position is found by intersecting two distance circles (one from each anchor). The tags are labeled, so the heading comes directly from the TAG_LEFT --> TAG_RIGHT vector.

```
TAG_LEFT  = intersect( circle(Anchor A, d_LA),  circle(Anchor B, d_LB) )
TAG_RIGHT = intersect( circle(Anchor A, d_RA),  circle(Anchor B, d_RB) )
```

Each trilateration yields 2 candidate points --> 4 candidate (L, R) pairs total. The valid pair is the one where the separation equals W. From there:

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
    if |sep - W| < tolerance:          # ~5-10 cm for UWB noise
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

The TAG_LEFT --> TAG_RIGHT vector points along the robot's right-lateral axis. At heading theta, the right-lateral unit vector is (cos theta, -sin theta), so:

```
vx = rx - lx = W * cos(theta)
vy = ry - ly = -W * sin(theta)

theta = atan2(-vy, vx)    # standard math atan2; result in degrees
```

Normalize to [0 deg, 360 deg).

**Verification:**
- Facing north (theta = 0 deg): TAG_R is east of TAG_L --> vx > 0, vy ~ 0 --> atan2(0, +) = 0 deg
- Facing east (theta = 90 deg): TAG_R is south of TAG_L --> vx ~ 0, vy < 0 --> atan2(+, 0) = 90 deg
- Facing south (theta = 180 deg): TAG_R is west of TAG_L --> vx < 0, vy ~ 0 --> atan2(0, -) = 180 deg

### Output

| Value | Type | Description |
|-------|------|-------------|
| x | float (metres) | Robot center east coordinate |
| y | float (metres) | Robot center north coordinate |
| theta | float (degrees) | Robot heading, 0 deg = north, CW positive |

## Serial Output Format

Each tag emits one line per cycle containing raw distances to all configured anchors. Format (confirmed from manual Section 7.1):

```
mc 0f 00000663 000005a3 00000512 000004cb ffffffff ffffffff ffffffff ffffffff 095f c1 00146fb7 a0:0 22be
      RANGE0    RANGE1    RANGE2    RANGE3    RANGE4    RANGE5    RANGE6    RANGE7
```

| Field | Example | Description |
|-------|---------|-------------|
| HEAD | `mc` | Fixed packet header |
| MASK | `0f` | Bitmask of valid ranges. `0x07` (0000 0111) = RANGE 0,1,2 valid. `0x0f` = RANGE 0,1,2,3 valid. |
| RANGE0–RANGE3 | `00000663` | Distance to Anchor A0–A3 in hex mm. `0x663` = 1635 mm = 1.635 m. |
| RANGE4–RANGE7 | `ffffffff` | Distance to Anchor A4–A7 (only present in 8-anchor firmware). `ffffffff` = invalid/no anchor. |
| NRANGES | `095f` | Message flow counter, accumulated 0x0–0xFFFF |
| RSEQ | `c1` | Range sequence number, accumulated 0x0–0xFF |
| RANGTIME | `00146fb7` | MCU timestamp in ms |
| rIDt:IDa | `a0:0` | Role + IDs. `a` = anchor, `t` = tag. Format: `{role}{tagID}:{anchorID}` |
| DIAGNOSIS | `22be` | Only present when role is anchor. RX power of last tag communication. |
| END | `\r\n` | Packet terminator |

**Tag-only additional line** (immediately follows `mc` line when device is a tag):

```
$KT0, 1.69, 2.93, 4.98, NULL, LO = [-2.45, 5.44, 1.43]
```

| Part | Meaning |
|------|---------|
| `K` | Kalman filter enabled (`NK` = disabled) |
| `T0` | Current role is Tag, ID = 0 |
| `1.69, 2.93, 4.98` | Distances to A0, A1, A2 in metres |
| `NULL` | A3 not responding / doesn't exist |
| `LO = [x, y, z]` | Tag-computed position (only valid if anchor coordinates have been configured via `$sanccd`) |

For our 2-anchor setup: RANGE0 = Anchor A (A0), RANGE1 = Anchor B (A1). RANGE2–RANGE7 will be `ffffffff`. Ranging is **sequential (multiplexed)** — the tag TWRs to each anchor in order, then outputs one packet with all distances.

**Important:** The `$K` line provides the tag's own position computation. We ignore this and compute position ourselves in `UWBLocalizer` because the tag doesn't know about the second tag and cannot compute heading.

## Code

- `library/uwb_localizer.py` — all UWB math and hardware access. Full localizer setup should return accurate heading, x, and y.
- `tests/test_uwb_hardware.py` — minimal raw serial test. Prints everything both tags send over USB. Use this first to verify the serial output format and baud rate before relying on the parser.

## Noise Mitigation

If UWB ranging noise causes jittery position estimates, consider implementing:

1. **Exponential moving average** — simple low-pass filter on (x, y, theta) outputs. Easiest to implement (~5 lines of code), minimal latency at moderate smoothing factors. Start here.
2. **Extended Kalman Filter (EKF)** — fuses UWB ranges with a motion model over time. Handles missed readings, reduces jitter, and can incorporate IMU data later. Upgrade to this if EMA isn't sufficient.

**Note:** The ULM3 has a built-in Kalman filter (S8 DIP switch, enabled by default). This filters the raw ranging distances on the module itself. Our software-side filtering (EMA/EKF) operates on top of this — it smooths the computed (x, y, theta) output, not the raw distances.

## Open Questions

1. **MASK field parsing:** The parser currently ignores the MASK field. We should validate that RANGE0 and RANGE1 bits are set (mask & 0x03 == 0x03) before trusting the distances. If either bit is unset, the corresponding range is invalid.
2. **8-anchor firmware vs 4-anchor firmware:** The manual mentions RANGE4–7 are only present in 8-anchor firmware. Need to verify which firmware ships by default and whether the packet always has 8 RANGE fields or only 4. This affects `_parse()` field indexing.
3. **Antenna delay calibration:** The `$santdly` command adjusts ranging accuracy. Need to calibrate on-site by measuring a known distance and adjusting until the reading matches.
4. **CH5 vs CH9:** Default is CH5. Verify all modules are on the same channel.

## Integration Plan
1. Run `tests/dashboard.py` to validate that the `UWBLocalizer` math in `library/uwb_localizer.py` produces correct (x, y, theta) with manual test values.
2. Connect a tag and run `tests/test_uwb_hardware.py` to confirm live packets match the format in the Serial Output section above. Check whether 4 or 8 RANGE fields are present.
3. Update `UWBLocalizer._parse()` in `library/uwb_localizer.py` to handle both 4-anchor and 8-anchor packet formats, and validate the MASK field. Use `tests/test_uwb_reading_serial.py` for testing before moving to `UWBLocalizer`.
4. Measure anchor positions and tag separation (W). Add them as constants in `robot_params.py`. Run full hardware integration test with all 4 modules active.
5. Move `UWBLocalizer` to `onboard_software/subsystems/uwb_localizer.py`. Add a `useUWB` feature toggle to `robot_params.py`. Call `update_from_hardware()` each cycle in the auto loop.

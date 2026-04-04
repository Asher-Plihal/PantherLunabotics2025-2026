# UWB (Ultra-Wideband) Research & Integration Notes

## Hardware

**Module:** Haorutech ULA1 — ESP32 + DWM1000 (Decawave UWB transceiver)
- Product: https://www.robotshop.com/products/haorutech-uwb-ultra-wideband-positioning-module-w-esp32-dwm1000-arduino
- User manual: https://www.haorutech.com/download/ULA1_UserManual-EN.pdf

**Architecture:** Jetson ←USB/Serial→ ESP32 ←SPI→ DWM1000

The ESP32 owns the SPI bus to the DWM1000 and handles all UWB ranging internally. The Jetson only communicates with the ESP32 over serial (USB) — it never touches the DWM1000 directly.

**Serial:** 115200 baud (confirmed — matches manual default)

## Role Configuration

Role and address are set via the **4-bit DIP switch** (S4–S7). Can also be set over serial (`r=a` for anchor, `r=t` for tag).

**S4 = Role:** ON = Anchor, OFF = Tag  
**S5/S6/S7 = 3-bit device address (0–7)** — each module must have a unique address.

| Module | S4 | S5 | S6 | S7 | Address |
|--------|----|----|----|----|---------|
| Anchor A | ON | OFF | OFF | OFF | A0 |
| Anchor B | ON | OFF | OFF | ON  | A1 |
| Tag LEFT | OFF | OFF | OFF | OFF | T0 |
| Tag RIGHT | OFF | OFF | OFF | ON  | T1 |

- Once configured, anchors only need power — no serial connection required during a match
- Tags stay connected to the Jetson via USB throughout the match

> From the manual: "During navigation mode, the tag needs to be connected to the PC while other anchors only need to power on."

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
- `/dev/ttyUSB0` is reserved for LiDAR

## How Localization Works

Four measurements per cycle — each tag ranges to each anchor independently:

| Measurement | Meaning |
|-------------|---------|
| d_LA | TAG_LEFT → Anchor A |
| d_RA | TAG_RIGHT → Anchor A |
| d_LB | TAG_LEFT → Anchor B |
| d_RB | TAG_RIGHT → Anchor B |

With 2 labeled tags and 2 anchors at known positions there are 4 measurements and only 3 unknowns (x, y, θ) — the system is over-constrained. **Absolute heading is derivable from UWB alone, no IMU required.**

```
  Anchor A (left wall)              Anchor B (right wall)
        *                                    *
        |  \                            /    |
     d_LA   \ d_RA              d_LB /      d_RB
        |     \                  /           |
        |      \                /            |
   [TAG_LEFT]───────────────────────────[TAG_RIGHT]
                        ←── W ──→
```

**Method — independent trilateration of each tag:**

Each tag's position is found by intersecting two distance circles (one from each anchor). The tags are labeled, so the heading comes directly from the TAG_LEFT → TAG_RIGHT vector.

```
TAG_LEFT  = intersect( circle(Anchor A, d_LA),  circle(Anchor B, d_LB) )
TAG_RIGHT = intersect( circle(Anchor A, d_RA),  circle(Anchor B, d_RB) )
```

Each trilateration yields 2 candidate points → 4 candidate (L, R) pairs total. The valid pair is the one where the separation equals W. From there:

- **Position:** robot center = midpoint of TAG_LEFT and TAG_RIGHT
- **Heading θ:** atan2(−(ry−ly), (rx−lx)) — derived from the lateral tag vector (see Math section)

**Residual ambiguity:** If two pairs both satisfy the W constraint (rare, occurs near the anchor-to-anchor diagonal), resolve using arena bounds or the previous position estimate.

**Role of IMU:** Not required for heading. Useful as a tiebreaker for residual ambiguity and for angular velocity estimation between UWB cycles.

**This is our primary localization source.** Robot position and heading are not known at match start and must be determined from UWB before autonomous navigation begins.

## Math

### Coordinate System

+X = east (right), +Y = north (up). Heading: 0° = north, clockwise positive (90° = east). Arena origin at bottom-left corner.

### Notation

| Symbol | Meaning |
|--------|---------|
| d_LA, d_RA | Distances from TAG_LEFT, TAG_RIGHT to Anchor A (metres) |
| d_LB, d_RB | Distances from TAG_LEFT, TAG_RIGHT to Anchor B (metres) |
| W | TAG_LEFT-to-TAG_RIGHT separation (metres) — measure on robot |
| (ax, ay), (bx, by) | Anchor A and B positions in arena coordinates |
| (lx, ly), (rx, ry) | TAG_LEFT and TAG_RIGHT positions (computed) |
| (x, y) | Robot center in arena coordinates (metres) |
| θ | Robot heading (degrees, 0° = north, CW positive) |
| f | Forward offset: distance tag midpoint is ahead of robot center |

### Step 1 — Trilaterate each tag

Shown for TAG_LEFT (r1 = d_LA, r2 = d_LB). Repeat identically for TAG_RIGHT (r1 = d_RA, r2 = d_RB).

```
d_AB = sqrt((bx-ax)^2 + (by-ay)^2)           # anchor separation

a = (r1^2 - r2^2 + d_AB^2) / (2 * d_AB)      # signed distance along A→B to radical axis
h = sqrt(max(r1^2 - a^2, 0))                  # half-chord length (clamp noise)

# Foot of perpendicular on A→B:
Px = ax + a * (bx - ax) / d_AB
Py = ay + a * (by - ay) / d_AB

# Perpendicular unit vector:
Qx = -(by - ay) / d_AB
Qy =  (bx - ax) / d_AB

# Two candidate positions:
P1 = (Px + h*Qx,  Py + h*Qy)
P2 = (Px - h*Qx,  Py - h*Qy)
```

If d_AB > r1 + r2 or d_AB < |r1 − r2|, the circles do not intersect — discard the reading (sensor noise or tag out of range).

### Step 2 — Select valid pair

Test all 4 combinations (TAG_L ∈ {P1_L, P2_L}, TAG_R ∈ {P1_R, P2_R}):

```
For each (L_candidate, R_candidate):
    sep = sqrt((rx - lx)^2 + (ry - ly)^2)
    if |sep - W| < tolerance:          # ~5–10 cm for UWB noise
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
x_center = x - f * sin(θ)          # apply after Step 4
y_center = y - f * cos(θ)
```

### Step 4 — Heading

The TAG_LEFT → TAG_RIGHT vector points along the robot's right-lateral axis. At heading θ, the right-lateral unit vector is (cos θ, −sin θ), so:

```
vx = rx - lx = W * cos(θ)
vy = ry - ly = -W * sin(θ)

θ = atan2(-vy, vx)    # standard math atan2; result in degrees
```

Normalize to [0°, 360°).

**Verification:**
- Facing north (θ = 0°): TAG_R is east of TAG_L → vx > 0, vy ≈ 0 → atan2(0, +) = 0° ✓
- Facing east (θ = 90°): TAG_R is south of TAG_L → vx ≈ 0, vy < 0 → atan2(+, 0) = 90° ✓
- Facing south (θ = 180°): TAG_R is west of TAG_L → vx < 0, vy ≈ 0 → atan2(0, −) = 180° ✓

### Output

| Value | Type | Description |
|-------|------|-------------|
| x | float (metres) | Robot center east coordinate |
| y | float (metres) | Robot center north coordinate |
| θ | float (degrees) | Robot heading, 0° = north, CW positive |

## Serial Output Format

Each tag emits one `RangeData` packet per cycle (message type `0x84`) containing raw distances to all configured anchors. Format (confirmed from manual Section 4):

```
mc 0f 00000663 000005a3 00000512 000004cb 095f c1 0 a0:0
        RANGE0    RANGE1    RANGE2    RANGE3
```

- **RANGE0–RANGE3**: distance from the tag to Anchor A0–A3 in hexadecimal millimeters. Example: `0x663` = 1635 mm = 1.635 m.
- **MASK** (`095f` in example): bitmask indicating which RANGE fields are valid.
- Ranging is **sequential (multiplexed)** — the tag TWRs to each anchor in order, then broadcasts one packet with all distances at the end of the cycle. Both anchor distances arrive in the same packet, so no synchronization is needed between RANGE0 and RANGE1.

For our 2-anchor setup: RANGE0 = Anchor A (A0), RANGE1 = Anchor B (A1). Update `UWBLocalizer._parse()` to parse this format.

## Code

- `tests/uwb_localizer.py` — all UWB math and hardware access. Full localizer setup should return accurate heading, x, and y. 
- `tests/test_uwb_hardware.py` — minimal raw serial test. Prints everything both tags send over USB. Use this first to verify the serial output format and baud rate before relying on the parser.

## Noise Mitigation

If UWB ranging noise causes jittery position estimates, consider implementing:

1. **Exponential moving average** — simple low-pass filter on (x, y, θ) outputs. Easiest to implement (~5 lines of code), minimal latency at moderate smoothing factors. Start here.
2. **Extended Kalman Filter (EKF)** — fuses UWB ranges with a motion model over time. Handles missed readings, reduces jitter, and can incorporate IMU data later. Upgrade to this if EMA isn't sufficient.

## Integration Plan
1. Run `tests/dashboard.py` to validate that the `UWBLocalizer` math in `tests/uwb_localizer.py` produces correct (x, y, θ) with manual test values.
2. Connect a tag and run `tests/test_uwb_hardware.py` to confirm live packets match the format in the Serial Output section above.
3. Update `UWBLocalizer._parse()` in `tests/uwb_localizer.py` to parse the `mc 0f RANGE0 RANGE1 ...` packet format.
4. Measure anchor positions and tag separation (W). Add them as constants in `robot_params.py`. Run full hardware integration test with all 4 modules active.
5. Move `UWBLocalizer` to `onboard_software/subsystems/uwb_localizer.py`. Add a `useUWB` feature toggle to `robot_params.py`. Call `update_from_hardware()` each cycle in the auto loop.


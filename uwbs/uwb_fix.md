# UWB Firmware Fix Plan

## Problems

### 1. T1 tag address is unsupported

`DW1000.h` defines `T0_ADDR` but never defines `T1_ADDR`. In `DW1000.cpp`, `ReadSwitch()` handles the tag branch like this:

```cpp
if(sw_role==ROLE_TAG)
{
    sw_addr1=digitalRead(SWITCH_SW2);
    sw_addr2=digitalRead(SWITCH_SW3);
    sw_addr3=digitalRead(SWITCH_SW4);
    if((sw_addr1==1)&&(sw_addr2==1)&&(sw_addr3==1))  // only T0 (all OFF)
    {
        LedCtl(T0_ADDR);
        return T0_ADDR;
    }
    else  // everything else → error
    {
        LedCtl(ERR_ADDR);
        return ERR_ADDR;
    }
}
```

Any switch combination other than all-OFF on a tag module returns `ERR_ADDR` and halts with `Switch setting error`. T1 (S7=ON, rest OFF) has no defined case. The anchor branch correctly handles A0–A3 but this was never mirrored for tags.

### 2. Tag sketch hardcodes T0 in serial output

`RTLS_T0.ino` line 127:

```cpp
sprintf(out_data,"mc %02x %08x %08x %08x %08x %04x %02x %08x t%d:0",
    range_mask, range_A0, range_A1, range_A2, range_A3,
    out_data_count++, seq_number, range_time, (uint8_t)T0_ADDR);  // hardcoded 0
```

Even if `ReadSwitch()` were fixed to return `T1_ADDR`, the output would still print `t0:0` instead of `t1:0`.

### 3. Accuracy settings not applied

Neither `RTLS_T0.ino` nor `RTLS_Anchor.ino` sets PRF, data rate, or preamble length after `setDefaults()`. This leaves all modules on the factory defaults (6.8 Mbps, PRF 16 MHz, preamble 64), which is the fastest but noisiest configuration. Our robot moves slowly — we should trade update rate for accuracy.

---

## Fix Approaches

### Approach A — Two separate sketches (RTLS_T0.ino + new RTLS_T1.ino)

Copy `RTLS_T0.ino` to `RTLS_T1.ino` and change the hardcoded `T0_ADDR` to `T1_ADDR`. Flash T0 sketch to one tag, T1 sketch to the other.

**Pros:**
- Simple, minimal library change.

**Cons:**
- Code duplication — two sketches that are 99% identical diverge over time.
- Flashing the wrong sketch to the wrong module causes silent mislabeling in the serial output (the switch reading would be right but the output tag ID would be wrong).
- Any future change (accuracy settings, ranging logic) must be applied twice.

### Approach B — Single generic tag sketch

Make `Dev_Addr` a global in `RTLS_T0.ino` and use it in the `sprintf` instead of the hardcoded `T0_ADDR`. The sketch reads its own address from the DIP switches at boot and uses it everywhere. Flash the same sketch to both tags — the DIP switches determine which tag it becomes.

**Pros:**
- Single sketch, no duplication.
- Any future change is made once.
- The printed tag ID always matches what the DIP switches say — no silent mislabeling risk.
- Consistent with how the anchor sketch already works (one sketch, DIP switches set the address).

**Cons:**
- Requires making `Dev_Addr` a module-level global instead of a local in `setup()`. This is a minor, standard Arduino pattern.

---

## Decision: Approach B

Approach B is strictly better. The only reason to choose Approach A would be to avoid making `Dev_Addr` global, which is a trivial change with no downside. Approach A introduces real maintenance risk and a silent failure mode that Approach B eliminates.

---

## Implementation Plan

### `uwbs/DW1000-Arduino/src/DW1000.h`

Add `T1_ADDR` alongside the existing address defines:

```cpp
#define T0_ADDR  0x0000
#define T1_ADDR  0x0001   // add this line
```

### `uwbs/DW1000-Arduino/src/DW1000.cpp`

Add T1 case inside the `ROLE_TAG` branch of `ReadSwitch()`, directly after the T0 case:

```cpp
if(sw_role==ROLE_TAG)
{
    sw_addr1=digitalRead(SWITCH_SW2);
    sw_addr2=digitalRead(SWITCH_SW3);
    sw_addr3=digitalRead(SWITCH_SW4);
    if((sw_addr1==1)&&(sw_addr2==1)&&(sw_addr3==1))
    {
        LedCtl(T0_ADDR);
        return T0_ADDR;
    }
    else if((sw_addr1==1)&&(sw_addr2==1)&&(sw_addr3==0))  // add this case
    {
        LedCtl(T1_ADDR);
        return T1_ADDR;
    }
    else
    {
        LedCtl(ERR_ADDR);
        return ERR_ADDR;
    }
}
```

Switch encoding (INPUT_PULLUP — OFF=HIGH=1, ON=LOW=0):
- T0: S5=OFF, S6=OFF, S7=OFF → (1,1,1)
- T1: S5=OFF, S6=OFF, S7=ON  → (1,1,0) ← matches anchor A1 pattern, correct

### `uwbs/DW1000-Arduino/examples/RTLS_T0/RTLS_T0.ino`

Two changes:

**1.** Promote `Dev_Addr` to a global (before `setup()`):

```cpp
uint16_t Dev_Addr = ERR_ADDR;
```

**2.** In `setup()`, assign instead of declare:

```cpp
Dev_Addr = DW1000.ReadSwitch(ROLE_TAG);
```

**3.** In `next_range()`, replace the hardcoded `T0_ADDR` with `Dev_Addr`:

```cpp
sprintf(out_data,"mc %02x %08x %08x %08x %08x %04x %02x %08x t%d:0",
    range_mask, range_A0, range_A1, range_A2, range_A3,
    out_data_count++, seq_number, range_time, (uint8_t)Dev_Addr);  // was T0_ADDR
```

**4.** Add accuracy settings after `setDefaults()` and before `commitConfiguration()`:

```cpp
DW1000.setDataRate(DW1000.TRX_RATE_110KBPS);
DW1000.setPulseFrequency(DW1000.TX_PULSE_FREQ_64MHZ);
DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_1024);
```

### `uwbs/DW1000-Arduino/examples/RTLS_Anchor/RTLS_Anchor.ino`

Add the same three accuracy lines after `setDefaults()` and before `commitConfiguration()`. Settings must match the tag sketch exactly or ranging will fail.

---

## Edge Cases Verified

| Case | Result |
|------|--------|
| `Dev_Addr == ERR_ADDR` at startup | Sketch halts at existing `while(1)` before `Dev_Addr` is used anywhere — no change in behavior |
| `(uint8_t)Dev_Addr` cast in sprintf | T0=0x0000→0, T1=0x0001→1 — both fit in uint8_t, correct output |
| Anchor flashed with tag sketch accidentally | `ReadSwitch(ROLE_TAG)` checks SW1==ROLE_TAG first; an anchor (SW1=ON=0) fails this check and returns `ERR_ADDR`, halting — same behavior as before |
| Accuracy mismatch between modules | All four files change together in one flash session — no partial-update risk if all modules are reflashed |

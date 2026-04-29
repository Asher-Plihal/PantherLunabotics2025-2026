# UWB Problems Log

## 1. T1 tag address unsupported in original firmware

**Problem:** The PeterSun01 fork of DW1000-Arduino only supported one tag address (T0 = 0x0000). Any module with DIP switches set to the T1 pattern returned `ERR_ADDR` and halted with "Switch setting error". The T1 address was never defined or handled in `ReadSwitch()` or `LedCtl()`.

**Fix (committed 9008fb1):**
- Added `#define T1_ADDR 0x0001` to `DW1000.h`
- Added T1 case in `ReadSwitch()` (switch pattern SW5=OFF, SW6=OFF, SW7=ON → pins read `1,1,0`)
- Added T1 LED case in `LedCtl()` (all LEDs OFF = white)
- Promoted `Dev_Addr` to a global in `RTLS_T0.ino` so T1 prints `t1:0` in serial output instead of hardcoded `t0:0`

---

## 2. Two tags collide at anchors — multi-tag ranging broken

**Problem:** When both T0 and T1 are active simultaneously, they poll the same anchors at nearly the same time. The anchor state machine uses `expectedMsgId` to track which message it expects next. When it receives T0's POLL and sends RESP, it sets `expectedMsgId = FC_FINAL`. If T1's POLL arrives before T0's FINAL, the anchor sees `msgId != expectedMsgId` and calls `resetInactive()` — abandoning the T0 exchange entirely. Both tags end up interrupting each other every cycle, resulting in `mc 00` (no anchors responding) on almost every packet.

**Root cause:** The RTLS_T0/RTLS_Anchor firmware was designed for one tag at a time. No multi-tag support exists in the original PeterSun01 firmware.

**Fix (possible fix, written but not yet flashed/tested):**

Tag side (`RTLS_T0.ino`):
1. T1 delays 125ms at the end of `setup()` before sending its first POLL. One full 4-anchor cycle is ~250ms, so 125ms puts T1 approximately half a cycle behind T0 at boot.
2. When the tag receives an unexpected msgId mid-cycle (typically the *other* tag's broadcast `FC_RANGEDATA` to `0xFFFF`), it now just re-arms the receiver instead of calling `resetInactive()`. The 6ms RX timeout still fires and moves us to the next anchor if the expected reply is genuinely lost. Without this, T0's broadcast RANGEDATA was knocking T1 back to A0 every cycle.

Anchor side (`RTLS_Anchor.ino`): When `msgId != expectedMsgId` and the unexpected message is `FC_POLL` while the anchor is mid-exchange (waiting for `FC_FINAL` or `FC_RANGEDATA`), the anchor ignores the competing POLL and re-opens the receiver instead of calling `resetInactive()`. **`noteActivity()` is intentionally NOT called on this path** — if T1 keeps spamming POLLs while T0's FINAL is lost to collision, refreshing the watchdog would let us hang forever. Letting the watchdog tick is the recovery mechanism.

**Status:** Firmware changes written, not yet reflashed. Needs testing with both tags active.

**Notes:**
- The cycles will naturally drift over time since the ESP32 clocks are not synchronized. If they drift back into phase, a few cycles of degraded readings will occur before they drift apart again. For a 30-minute competition run this should be acceptable.
- If sustained collision becomes a problem, a proper TDMA implementation with synchronized time slots would be the next step.

**For a future AI attempting a better fix:**
Read `uwbs/DW1000-Arduino/examples/RTLS_Anchor/RTLS_Anchor.ino` and `uwbs/DW1000-Arduino/examples/RTLS_T0/RTLS_T0.ino`. The problem is two tags colliding at the anchor because they transmit at the same time and the anchor can only handle one exchange at a time. A current partial fix exists in both files but has not been tested. Write a robust fix — the goal is both T0 and T1 getting reliable `mc 03` readings simultaneously with 2 anchors active. Read `uwbs/UWB.md` first for full hardware and protocol context. keep it simple and concise dont change more than you need
- **Comparison to MaUWB (reddit.com/r/diyelectronics/comments/1fp0kg5):** MaUWB is a commercial module (STM32 + DW3000, newer chip) that solves the multi-tag collision problem by design — it supports up to 8 anchors and 64 tags with scheduling built into the firmware. The Reddit post specifically opens by calling out UWB signal interference between multiple anchors and tags as the core problem UWB faces. Our situation is the same problem on cheaper hardware (DW1000-based ULA1 modules) with a minimal firmware that was never designed for it. MaUWB is not a drop-in replacement — it uses AT commands over serial and a completely different communication model — but it confirms this is a well-known problem and the fix is proper time-division scheduling.

---

## 3. ESP32 boots into download mode when opened via serial

**Problem:** When modules are plugged into the Jetson via USB, the CH340 USB-to-serial chip asserts DTR low on power-up, which holds GPIO0 low and causes the ESP32 to boot into `DOWNLOAD_BOOT` instead of running the ranging firmware. This happens before any Python code runs. When two modules are plugged in simultaneously the problem is worse — both go into download mode and the RTS reset sequence in the test script is not reliable enough to recover both.

**Fix (in `test_uwb_hardware.py`):**
- Run `stty -F <port> -hupcl` before opening each port to prevent the kernel serial driver from toggling control lines on open
- Set `dtr=False` and re-assert it before releasing RTS reset to ensure GPIO0 stays HIGH throughout the reset sequence
- Stagger the two tag threads by 4 seconds so T0 is fully booted before T1's reset fires

**Notes:**
- The fix is reliable but not 100% consistent across runs — occasionally a second run is needed. This is a hardware quirk of the CH340 auto-reset circuit on these specific modules.
- When only one module is plugged in it always boots correctly on the first try.

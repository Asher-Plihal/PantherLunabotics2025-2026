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

**Fix (written, partially tested — see Iteration 2):**

### Iteration 1 (insufficient — T1 still got `mc 00`)

Tag boot offset (T1 delays 125ms) + anchor ignored stray `FC_POLL` mid-exchange. After flashing, T0 went from `mc 00` → `mc 01` (still missing A1) and T1 stayed at `mc 00`. Wasn't enough.

### Iteration 2 (current — root cause)

Reading T1's serial output showed `seq_number` jumping by exactly 5 each cycle (4 POLLs + 1 broadcast RANGEDATA, **zero FINALs**). T1 was transmitting POLLs but receiving zero RESPs from any anchor. That eliminated "collision corrupts T1's POLL" — T1's POLLs were arriving at anchors but anchors weren't responding.

The actual root cause is in `RTLS_Anchor.ino`'s FC_RANGEDATA handler: after a successful cycle the anchor calls `DW1000.idle()` and sits deaf until the 200ms watchdog fires. The original firmware was designed for a single tag — its watchdog was tuned so the anchor's deaf window ended exactly when the tag's next POLL was due. With two tags the second tag's POLLs almost always landed in the deaf window and got no response.

Three changes ([RTLS_Anchor.ino](DW1000-Arduino/examples/RTLS_Anchor/RTLS_Anchor.ino), [RTLS_T0.ino](DW1000-Arduino/examples/RTLS_T0/RTLS_T0.ino)):

1. **Anchor never goes deaf.** After processing FC_RANGEDATA, replace `DW1000.idle()` with `resetInactive()` so the anchor goes straight back to listening for the next POLL.
2. **Generalized stray-frame handling on anchor.** Any wrong msgId (the other tag's POLL *or* its broadcast RANGEDATA) just re-arms the receiver. No `noteActivity` on that path — the watchdog must be allowed to fire if the expected reply is genuinely lost.
3. **Watchdog shortened to 50ms (anchor + tag).** Only used for stuck-mid-exchange recovery now, not inter-cycle pacing. 50ms gives plenty of margin over a real ~25ms exchange while recovering quickly under collisions.

Tag-side stray-frame ignore (broadcast RANGEDATA from the other tag) and T1 boot offset are kept from iteration 1.

**Status (Iteration 2):** Confirmed working for single-tag operation (T1 alone achieves consistent `mc 03`). Multi-tag collision persists: each tag monopolizes one anchor exclusively; `recv time out 0x82` on A1 indicates A1 completes a TWR exchange with T1 but T1's broadcast never arrives because T0's interference disrupts T1's cycle timing.

### Iteration 3 (current — passive TDMA for T1)

Root cause of remaining collision: T0 and T1 free-run independently and inevitably overlap on-air. The fix is to remove the free-running behavior from T1 entirely.

T1 now waits for T0's broadcast `FC_RANGEDATA` frame (sent to 0xFFFF at the end of every cycle) before starting its own POLL cycle. Since T0's broadcast is received by every node simultaneously, T1 knows exactly when T0 has finished — and starts immediately after, with no simultaneous air-time. The 200ms watchdog fires as a fallback if T0 is absent (standalone T1 test).

Changes (RTLS_T0.ino only — anchor unchanged):

1. **T1 boot**: instead of `delay(125)` + immediate POLL, T1 opens receiver and sets `waitingForT0 = true`. T0 boots and starts free-running immediately.
2. **After T1's RANGEDATA sent**: instead of `DW1000.idle()`, T1 opens receiver and sets `waitingForT0 = true`.
3. **Receive handler (T1 only)**: if `waitingForT0`, check if received frame is `FC_RANGEDATA` from T0 (`data[7:8]` == `T0_ADDR`). If yes: clear flag, call `resetInactive()` → start T1's cycle. If no: re-arm receiver.
4. **RX timeout handler**: if `waitingForT0`, re-arm receiver instead of calling `next_range()`.
5. **`resetInactive()`**: clears `waitingForT0` (watchdog fallback path).

Expected timing: T0 cycle ~80-100ms → T0 broadcasts → T1 starts → T1 cycle ~80-100ms → T1 broadcasts → T1 waits → T0 starts next cycle at ~200ms after its broadcast. Gap between T1 finishing and T0 starting: ~100-120ms. No overlap.

**Status:** Written, not yet flashed. Needs testing with both tags and both anchors active.

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

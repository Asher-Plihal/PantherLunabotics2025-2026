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

### Iteration 3 — Passive TDMA + anchor deaf-window fix (**SOLVED**)

Two root causes remained after Iteration 2:

**Root cause A — free-running collision:** T0 and T1 both free-ran at the same ~240ms cycle period. Their phases inevitably aligned, causing simultaneous POLLs and anchor confusion.

**Root cause B — anchor deaf window (60ms hardware RX timeout):** The anchor had `setReceiveFrameWaitTimeoutPeriod(60000)` = 60ms. After each `resetInactive()`, the receiver ran for 60ms, then the hardware timeout fired. The `receivetimeoutAck` handler only printed a message — it never called `receiver()`. The anchor's DW1000 sat deaf for 140ms (gap between the 60ms HW timeout and the 200ms watchdog). T0's next poll always arrived during this deaf window and got no response → `mc 00`.

**Fix A — passive TDMA (RTLS_T0.ino):** T1 waits for T0's `FC_RANGEDATA` broadcast (to 0xFFFF, detected by source address `T0_ADDR = 0x0000`) before starting its own cycle. Timeout is 400ms (> T0's ~280ms full cycle period). `noteActivity()` is called on each 6ms RX tick during the wait so the 200ms watchdog doesn't fire early. Standalone fallback: if T0's broadcast never arrives within 400ms, T1 free-runs.

**Fix B — anchor always-receive (RTLS_Anchor.ino):** Added `receiver()` at the end of the `receivetimeoutAck` handler so the anchor immediately re-arms after every 60ms timeout. Eliminated the deaf window entirely. Also suppressed noisy FC_POLL timeout logs (normal idle churn).

**Result:** Both T0 and T1 achieve solid `mc 03` every cycle simultaneously with no drops. Confirmed across two consecutive test runs.

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

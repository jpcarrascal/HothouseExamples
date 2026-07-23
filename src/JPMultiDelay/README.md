# JPMultiDelay

A stereo ping-pong delay with a modulated filter, reverse delay modes, granular pitch transposition, and lo-fi degradation. Heavily extended from the original `petal/MultiDelay` DaisyExamples port.

### How to build:
make clean; make ; make program-dfu

---

## Controls

### Knobs

| Knob | Function | Notes |
|---|---|---|
| KNOB 1 | Delay time | 50ms–10s, logarithmic. Also the baseline for tap tempo. |
| KNOB 2 | Feedback | 0–100%. Values above 95% snap to exactly 100% for a stable infinite loop. |
| KNOB 3 | Send level | How much dry signal feeds into the delay. |
| KNOB 4 | Filter cutoff | 200Hz–20kHz, logarithmic. |
| KNOB 5 | Filter resonance | 0–0.9. |
| KNOB 6 | Filter LFO rate | 0–100Hz. Cubic taper for finer control at low speeds. |

### Toggle switches (normal layer)

| Switch | UP | MIDDLE | DOWN |
|---|---|---|---|
| TS1 | Low-pass filter | Band-pass filter | High-pass filter |
| TS2 | LFO depth: off | LFO depth: gentle | LFO depth: deep |
| TS3 | Forward delay | One-shot reverse | Compound reverse |

**One-shot reverse:** the dry input is reversed once before entering the delay. Repeats echo the reversed audio forward, cleanly decaying.

**Compound reverse:** the delay's own read position is grain-reversed inside the feedback loop. Every repeat re-reverses, building a smeared, turbulent texture. Engaging freeze in this mode causes self-oscillation (intentional).

### Footswitches

| Footswitch | Short press | Long hold |
|---|---|---|
| FS1 | Tap tempo | — |
| FS2 | Bypass toggle | Freeze (infinite loop) |

**Tap tempo:** tap FS1 repeatedly to set delay time. Moving KNOB 1 more than a small threshold cancels tap tempo and hands control back to the knob.

**Freeze:** hold FS2 for 500ms to latch the delay into an infinite loop. Short-press FS2 again to exit.

**Reset to bootloader:** hold both footswitches for 2 seconds. LEDs alternate to confirm, then the Daisy Seed enters DFU mode.

---

## Boot-time configuration

These settings are saved to QSPI flash and persist across power cycles. Each gesture **toggles** the stored value, so repeating it at the next boot reverses it. The LED blinks to confirm.

| Gesture | Effect | LED |
|---|---|---|
| Hold **FS2** while powering on | Toggle mono/stereo input (`led_delay` blinks 3×) | led_delay |
| Hold **FS1** while powering on | Toggle mono/stereo output (`led_tap` blinks 3×) | led_tap |
| Hold **both** | Both toggle independently | Both blink |

**Mono input (default):** L and R inputs are summed to mono before entering the delay.

**Stereo input:** L and R are kept separate throughout the delay — R feeds the second delay tap independently.

**Mono output:** both delay taps are summed and sent to both output channels (useful for mono rigs). Dry signal is unaffected.

---

## Hidden modifier layer

Hold **FS1** and move a toggle switch to update a stored parameter. The switch returns to its normal function when FS1 is released. `led_tap` blinks 3× to confirm each change.

| Hold FS1 + flip | UP | MIDDLE | DOWN |
|---|---|---|---|
| **TS1** | Lo-fi: off | Lo-fi: mild (24kHz / 8-bit) | Lo-fi: heavy (12kHz / 4-bit) |
| **TS2** | Filter: after delay | Filter: before delay | Filter: in feedback loop |
| **TS3** | Pitch: none | Pitch: +fifth (×1.5) | Pitch: +octave (×2.0) |

### Lo-fi modes
Applied to the wet signal after the delay, before the filter. Sample rate reduction runs first (zero-order hold, no anti-aliasing — aliasing is intentional), then bit crushing.

### Filter positions
- **After delay (default):** filter colours the wet output. Classic tone control on the echoes.
- **Before delay:** filter is applied to the input before it enters the delay buffer. Echoes are filtered copies of what you played; LFO modulation state is baked into each echo at capture time.
- **In feedback loop:** filter is applied inside the feedback path before each write. Each repeat passes through the filter again — LP creates progressively darker echoes; BP with high resonance can self-oscillate.

### Pitch transpose
Granular pitch shifting applied to the dry input before the delay. One-shot reverse and pitch shift chain in series (reversed audio is then pitch-shifted). Grain length is set by `PITCH_GRAIN_MULTIPLIER` in the source (default 0.9 × sample rate ≈ 900ms).

---

## Signal chain

```
Input
  └─ [dryCapture]
       └─ [oneShotReverser] (if TS3=MIDDLE)
            └─ [reversedCapture]
                 └─ [pitchShifter] (if pitch selected)
                      └─ [filter before] (if TS2 hidden=MIDDLE)
                           └─ PingPongDelay (with optional compound reverse / in-loop filter)
                                └─ [lo-fi] (if TS1 hidden=MIDDLE or DOWN)
                                     └─ [filter after] (default)
                                          └─ Mix with dry → Output
```

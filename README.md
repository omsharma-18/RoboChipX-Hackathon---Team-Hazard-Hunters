# Edge AI Safety System — Confined Space / Manhole Monitor
### Problem 05: Edge Analytics IP | SDG 9 & SDG 11

**Platform:** PYNQ-Z2 (Zynq-7020, Dual Cortex-A9 PS + Artix-7 PL)
**Language:** Verilog / SystemVerilog RTL, PYNQ Python (PS-side)
**Sensor acquisition: fully digital (no XADC, no analog front-end).**
**Status document — read this before touching any RTL.**

---

## 1. What We're Building (One Paragraph)

A chip-level IP core that sits inside the Zynq-7020 PL (fabric) and watches four sensors — gas (MQ-2, digital D0), flame (IR module, digital D0), depth/proximity (HC-SR04 ultrasonic, digital pulse timing), and temperature/humidity (DHT11, digital single-bus protocol) — for a worker in a confined space like a manhole or storage tank. It filters, extracts features from, and fuses these signals **entirely in hardware**, with zero cloud or network dependency, and produces a digital alert level (NORMAL/WARNING/CRITICAL/FAULT) plus controller outputs (relay, buzzer, LED, UART telemetry, and a live Jupyter dashboard) in under 10 µs of core-logic latency. A lightweight quantized neural network (Tiny MLP, 4-8-4-2, Q1.15 fixed point) running in the PL catches multi-variable anomalies that fixed thresholds miss, but a hard-rule safety layer can never be overridden by the ML — this is a life-safety system first, an "AI demo" second.

---

## 2. Why This Problem / Why This Approach

The brief (Problem 05) requires: sensor collection, a moving average filter, real-time edge processing, and output analytics — with bonus points for AI-driven anomaly detection, multi-sensor fusion, and cloud sync.

**Why edge, not cloud:** underground/confined spaces have no reliable signal, and a gas spike can become lethal in seconds — round-tripping to a cloud inference server is a non-starter for both connectivity and latency reasons. This is the literal argument the brief itself makes (SDG 9: resilient infrastructure; SDG 11: safe, sustainable human settlements), and doing it on an FPGA rather than a microcontroller is what makes it a "chip-level IP core" instead of just embedded firmware.

**Why digital-only sensor acquisition (D0 outputs), not analog + XADC:**
- Every sensor in this build has a digital output pin available: MQ-2 and Flame IR breakout boards both carry an onboard LM393 comparator with a potentiometer-adjustable threshold, giving a clean binary D0 (detected / not detected); HC-SR04 is inherently digital (trigger/echo pulse timing); DHT11 is inherently digital (single-bus timed protocol).
- Going digital-only **removes an entire category of complexity**: no XADC sequencer/DRP configuration, no voltage-divider *scaling* math to fit sensor output into a 0–1V ADC window, no RC low-pass anti-aliasing filter design, no analog noise/grounding concerns.
- The tradeoff (addressed head-on in Section 7) is that MQ-2/Flame D0 gives you a **binary** detection signal rather than a continuous concentration/intensity value — the moving average filter and feature extractor are reinterpreted accordingly (see below), and analog-style "stuck sensor" fault detection is no longer possible for these two channels. This is an honest, documented limitation, not something to gloss over in front of judges.

**Why three layers of decision-making (hard rules → ML → fusion → vote), not just one ML model:**
- A pure ML model can be wrong in ways that kill someone if the training data was thin — so **hard threshold/detection rules always win** for genuinely lethal conditions (gas detected, flame detected). This layer is deterministic and auditable to a safety inspector.
- The **Tiny MLP** exists to catch things a fixed rule cannot express — e.g., a fall event combined with a marginal gas detection duty-cycle that's each individually borderline but jointly concerning. This is the bonus "AI-driven anomaly detection" feature.
- The **fusion engine** encodes domain knowledge that is cheaper to hand-code than to have an ML model rediscover from scratch — e.g., gas + flame together is categorically worse than either alone (explosion risk), so we just state that rule rather than hoping the MLP learns it from limited training data. This is the bonus "multi-sensor fusion" feature.
- A **hybrid voter** reconciles all of this into one output so the system never has two disagreeing signals with undefined behavior.

**Why Tiny MLP (Q1.15 fixed-point) instead of a Binary Neural Network (BNN):**
BNN's entire value proposition — replacing multiply-accumulate with XNOR+popcount — only pays off when you have hundreds/thousands of weights and are genuinely resource-constrained. Our topology (4→8→4→2) has only **72 total weights**. At that scale:
- A naive Q1.15 MAC implementation costs a handful of DSP48E1 slices or LUT-mults — negligible on a Zynq-7020 (~53K LUTs, 220 DSP48E1s available).
- Binarizing weights on an already-tiny, 4-feature-input model risks losing the accuracy needed for a life-safety classifier, for zero real resource benefit.
- Training is simpler: plain PyTorch + post-training quantization to Q1.15, no Brevitas/straight-through-estimator complexity, easier `.coe` export, easier to explain to judges in one sentence.
- **Do not reintroduce BNN/binarization anywhere in this project.** The system uses a **Tiny MLP**, not a BNN. If "BNN" appears anywhere in old notes/diagrams, it is stale and wrong.

**Why true windowed moving average (majority-vote style for binary channels), not exponential moving average (EMA):**
The brief explicitly asks for "configurable window sizes." EMA has no literal window — it's an IIR filter with an implicit time-constant, not a windowed average. This design uses a **true circular buffer with running sum** (subtract-oldest, add-newest) on all four channels:
- For **multi-bit channels** (ultrasonic distance, DHT11 temp/humidity), this behaves exactly like a classic FIR moving average filter on continuous-ish data.
- For **binary channels** (MQ-2 D0, Flame D0), the same circular-buffer-and-running-sum hardware becomes a **debounce/majority-vote filter**: the "average" of the last N binary samples is the fraction of recent samples asserted, and comparing that fraction against a configurable threshold gives you a confirmed, noise-immune detection decision instead of reacting to a single transient glitch sample. Same hardware, same math, different — and equally legitimate — interpretation depending on the channel's data type.

**Why per-channel independent filters instead of one shared time-division-multiplexed (TDM) filter:**
Gas and flame (binary, debounce-style) need a completely different window semantic than ultrasonic/DHT11 (multi-bit, true averaging) — there's no way a shared filter could serve both purposes correctly. Even within the multi-bit pair, ultrasonic has more jitter than DHT11's already-slow integer output. Independent per-channel filters are a correctness requirement, not just a nice-to-have.

---

## 3. System Architecture — Full Block Diagram

Data flows top to bottom. Every block below is a distinct Verilog module (see Section 6 for the module table).

```
┌──────────────────────────────────────────────────────────────────────────────┐
│         EDGE AI SAFETY SYSTEM — CONFINED SPACE / MANHOLE                     │
│                    (Fully Digital Sensor Acquisition)                        │
│                                                                              │
│  SYSTEM FSM: INIT → CALIBRATE → ACQUIRE → PROCESS → ALERT/FAULT              │
│  (Gates everything below on sensor warm-up / readiness)                      │
│                                    │                                         │
│  ┌─────────────────────────────────┼──────────────────────────────────────┐  │
│  │           SENSOR ACQUISITION BLOCK (Mandatory Feature #1)              │  │
│  │                                                                        │  │
│  │  CH0: MQ-2 D0          CH1: Flame D0        CH2: Ultrasonic  CH3: DHT11│  │
│  │  Digital (LM393        Digital (LM393       Trigger+Echo     1-Wire    │  │
│  │  comparator, onboard   comparator, onboard  pulse timing     bit-bang  │  │
│  │  threshold pot)        threshold pot)       (HC-SR04)        protocol  │  │
│  │  Binary: 0/1           Binary: 0/1          Multi-bit:       Multi-bit:│  │
│  │                                             distance value   temp+hum  │  │
│  │       │                     │                    │              │      │  │
│  │  ┌────┴─────────────────────┴────────────────────┴──────────────┴───┐  │  │
│  │  │  2-FLOP SYNCHRONIZER (per async digital input — CDC safety)      │  │  │
│  │  │  GPIO INPUT (MQ-2 D0, Flame D0 — direct binary read)             │  │  │
│  │  │  GPIO TIMER (echo pulse width → distance, HC-SR04)               │  │  │
│  │  │  GPIO BIT-BANG (drives/reads DHT11 single-bus → temp+humidity)   │  │  │
│  │  └────┬─────────────────────────────────────────────────────────────┘  │  │
│  └───────┼────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  FAULT DETECTOR (see Section 7 — MQ-2/Flame limitation acknowledged)   │  │
│  │  • Ultrasonic echo timeout    → no return pulse in 30ms = blocked/dead │  │
│  │  • DHT11 checksum fail        → 8-bit sum mismatch = corrupted read    │  │
│  │  • DHT11 timeout              → no response = disconnected             │  │
│  │  • MQ-2 / Flame: NO reliable stuck-at fault possible on D0-only mode   │  │
│  │    (binary '0' held long-term is indistinguishable from "safe")        │  │
│  │  Any fault → alert_level = FAULT, disables MLP, hard-rules-only mode   │  │
│  └───────┬────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  SENSOR AGGREGATOR + TIMESTAMP                                         │  │
│  │  • 32-bit free-running µs counter stamped onto every sample            │  │
│  │  • 2-bit channel-ID tagging                                            │  │
│  │  • Async FIFO for clock-domain crossing (sensor timing → 100 MHz core) │  │
│  └───────┬────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  4× INDEPENDENT WINDOWED FILTER (Mandatory Feature #2)                 │  │
│  │  CH0 Gas (binary):    window=64, majority-vote debounce                │  │
│  │  CH1 Flame (binary):  window=16, majority-vote debounce                │  │
│  │  CH2 Ultra (multi-bit): window=8,  true moving average (jitter)        │  │
│  │  CH3 DHT11 (multi-bit): window=2,  true moving average (minimal)       │  │
│  │  Implementation: circular buffer + running sum, power-of-2 shift div   │  │
│  │  For binary channels: running_sum / window = "confidence fraction"     │  │
│  │  Windows are runtime-configurable via AXI4-Lite (cfg_win_log2)         │  │
│  └───────┬────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  SAMPLE-RATE RECONCILIATION (Zero-Order Hold)                          │  │
│  │  Problem: CH0≈10Hz, CH1≈50Hz, CH2≈33Hz, CH3≈1Hz — mismatched rates     │  │
│  │  • Latches each channel's last valid filtered value                    │  │
│  │  • Forms one coherent 4-D vector, gated on ALL channels having been    │  │
│  │    seen at least once (all_seen flag) AND CH3 (slowest) just updated   │  │
│  │  • MLP effectively samples at 1 Hz, aligned to DHT11                   │  │
│  └───────┬────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  FEATURE EXTRACTOR (Mandatory Feature #3 — per channel)                │  │
│  │  • Gas/Flame (binary): confidence fraction, assertion rate-of-change   │  │
│  │  • Ultra/DHT11 (multi-bit): mean, max, min, peak-to-peak, rate-of-     │  │
│  │    change, variance via Welford's online algorithm (2 registers)       │  │
│  │  • No FFT — sensors are far too slow to need spectral analysis         │  │
│  └───────┬────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  THRESHOLD DETECTOR (hard rules — life safety, non-negotiable layer)   │  │
│  │  • Gas confidence fraction > CRITICAL_THRESH → immediate CRITICAL      │  │
│  │  • Flame confidence fraction > CRITICAL_THRESH → immediate CRITICAL    │  │
│  │  • All thresholds runtime-configurable via AXI4-Lite                   │  │
│  └───────┬────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  LIGHTWEIGHT ML: TINY MLP (4-8-4-2) in Q1.15 (Bonus: Edge AI)          │  │
│  │  Input:  [gas_conf, flame_conf, ultra_norm, temp_norm]                 │  │
│  │  Hidden: 8 neurons → ReLU → 4 neurons → ReLU                           │  │
│  │  Output: [safe_prob, unsafe_prob] → argmax = class                     │  │
│  │  Weights: 72 total (32+32+8), Q1.15, BRAM-initialized from .coe file   │  │
│  │  Training: PyTorch (float32) → post-training quantize to Q1.15 → .coe  │  │
│  │  Latency: ~100 cycles ≈ 1 µs @ 100 MHz                                 │  │
│  │  Disabled automatically when FAULT is active (falls back to rules)     │  │
│  └───────┬────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  CROSS-SENSOR FUSION ENGINE (Bonus: Multi-sensor fusion)               │  │
│  │  • Gas + Flame               → EXPLOSION_RISK → CRITICAL               │  │
│  │  • Gas + No Flame            → GAS_LEAK → WARNING                      │  │
│  │  • Depth changed too fast    → FALL_DETECTED → CRITICAL                │  │
│  │  • Temp-compensated ultrasonic (speed-of-sound correction, free)       │  │
│  └───────┬────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  HYBRID VOTER                                                          │  │
│  │  IF hard_rule = CRITICAL           → CRITICAL (never overridden)       │  │
│  │  IF hard_rule = NORMAL, ML=UNSAFE  → WARNING                           │  │
│  │  IF hard_rule = NORMAL, ML=SAFE    → NORMAL                            │  │
│  │  IF hard_rule and ML disagree      → WARNING (conservative default)    │  │
│  └───────┬────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  OUTPUT ANALYTICS SYSTEM (Mandatory Feature #4)                        │  │
│  │  Digital outputs:                                                      │  │
│  │   alert_level[1:0]   00=NORMAL 01=WARNING 10=CRITICAL 11=FAULT         │  │
│  │   anomaly_type[3:0]  which sensor/fusion rule triggered (16 codes)     │  │
│  │   confidence[7:0]    MLP probability, 0–255                            │  │
│  │   timestamp[31:0]    µs since boot                                     │  │
│  │  Controller interface:                                                 │  │
│  │   relay_trigger      cuts gas valve / ventilation / alarm circuit      │  │
│  │   buzzer_pattern[2:0] off/slow/fast/urgent/evacuate                    │  │
│  │   led_rgb[2:0]       green/yellow/red/flashing (onboard RGB LEDs)      │  │
│  │   uart_tx            JSON-like telemetry → PYNQ Jupyter dashboard      │  │
│  │   lora_tx_ready       optional gateway sync (non-critical path)        │  │
│  └───────┬────────────────────────────────────────────────────────────────┘  │
│          │                                                                   │
│  ┌───────┴────────────────────────────────────────────────────────────────┐  │
│  │  AXI4-Lite CONFIG + STATUS INTERFACE (PS ↔ PL control plane)           │  │
│  │  • Threshold registers (R/W from Zynq PS)                              │  │
│  │  • Filter window config per channel (cfg_win_log2 ×4)                  │  │
│  │  • MLP weight loading (streamed, no resynthesis needed to update)      │  │
│  │  • Calibration offsets (set during CALIBRATE FSM state)                │  │
│  │  • STATUS READBACK: alert_level, anomaly_type, confidence, timestamp,  │  │
│  │    raw + filtered per-channel values — feeds the Jupyter dashboard     │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. How It Works on PYNQ-Z2 — End-to-End Data Path

**Board:** PYNQ-Z2 = Zynq-7020, dual-core Cortex-A9 (PS) running PYNQ Linux + this custom IP core synthesized into the programmable logic (PL).

**Physical I/O mapping — all four sensors are now straightforward digital GPIO, split across PMODA and PMODB:**
- MQ-2 D0 → PMODA pin (digital input, plus a **voltage-safety check** — see below)
- Flame IR D0 → PMODA pin (digital input, plus the same voltage-safety check)
- HC-SR04 Trigger (PL output) + Echo (PL input, **needs level shifting**, see below) → PMODB pins
- DHT11 data line (bidirectional single-bus) → PMODB pin, with external pull-up resistor if the breakout board doesn't already have one

**This is the whole sensor wiring plan — no XADC header needed at all**, since nothing goes through the analog front-end anymore. That simplifies your board bring-up considerably: it's just 4 GPIO signals (plus power/ground) across two PMOD connectors.

**Voltage-safety note — still relevant even though everything is digital now:**
- If your MQ-2/Flame breakout boards run their onboard LM393 comparator at **5V**, their D0 output will swing to ~5V logic-high — feeding that directly into a Zynq PL input (3.3V LVCMOS max) risks damaging the I/O bank. Check your specific module's supply voltage; if it's 5V, you still need a simple resistor divider (e.g., 1kΩ top / 2kΩ bottom → ~3.33V) on D0, same idea as the echo pin below, just simpler since it's a clean digital edge, not an analog waveform.
- HC-SR04 Echo pin: if you power the ultrasonic module at 5V (common/most reliable), Echo is a genuine 5V pulse — needs the same resistor divider (e.g., 1kΩ top / 2kΩ bottom → ~3.33V) before entering PL. Trigger direction (PL→sensor) is fine at 3.3V, no shifting needed there.
- DHT11 is typically fine at 3.3V logic directly if you're running the sensor itself off the 3.3V rail — check your specific module; some only guarantee correct operation at 5V, in which case the data line also needs the same divider treatment.

**Why the 2-flop synchronizer matters here specifically:** MQ-2 D0, Flame D0, HC-SR04 Echo, and DHT11's data line are all fully asynchronous to your PL clock — none of them share any clock relationship with your 100 MHz core. Feeding any of these directly into clocked logic risks metastability (a glitched, momentarily-invalid register value that can propagate as a corrupted bit or a misread pulse width). Every one of these four raw inputs gets its own 2-flop synchronizer immediately at the PL boundary, before any other logic touches it.

**PL (fabric) responsibilities — everything in Section 3's diagram:**
1. Synchronize, then acquire all 4 digital sensor signals continuously and independently (different native sample rates/protocols).
2. Detect the fault conditions that are actually detectable in D0-only mode (Section 7 covers exactly what's possible vs. not).
3. Timestamp, tag, and funnel samples through a clock-domain-crossing FIFO into the 100 MHz core clock domain.
4. Filter (windowed average / majority-vote debounce depending on channel), reconcile sample rates (ZOH), extract features, apply threshold rules, run the Tiny MLP, run fusion rules, vote, and produce final alert + controller outputs — all combinational/pipelined, target <10 µs end-to-end core-logic latency.
5. Expose an AXI4-Lite slave so the PS can configure thresholds, filter windows, MLP weights, and — critically for your demo — **read back full status for the Jupyter dashboard.**

**PS (Zynq Cortex-A9 / PYNQ Python) responsibilities:**
1. **Bitstream + overlay loading** — load the `.bit`/`.hwh` pair via the PYNQ `Overlay` class at boot.
2. **MLP weight deployment** — the PS reads the trained `.coe`/binary weight file (produced offline by the PyTorch training script) and writes it into the weight BRAM over AXI4-Lite at boot, or on-demand.
3. **Live Jupyter dashboard (this is your primary demo output, not the 4 onboard LEDs).** PYNQ-Z2 only has 4 individual LEDs plus 2 onboard RGB LEDs — nowhere near enough to show `alert_level`, `anomaly_type`, `confidence`, `timestamp`, and per-channel filtered values simultaneously. Use the LEDs only for coarse at-a-glance status (RGB = NORMAL/WARNING/CRITICAL color, individual LEDs = per-channel fault flags), and build the actual detailed dashboard in a Jupyter notebook: a Python loop polls the AXI4-Lite status registers via `overlay.your_ip.mmio.read(offset)`, and renders live gauges/plots/an alert banner using `ipywidgets` + `matplotlib` (or `plotly` for smoother live updates). This runs locally on the board's own Linux, viewed in a browser over your laptop's connection to the board — no internet dependency, satisfies "digital outputs for dashboards" directly.
4. **Does NOT sit in the safety-critical path.** The PL makes the alert decision autonomously in hardware; the PS is for configuration, monitoring, and the dashboard only. If the PS/Linux side hangs or reboots, the safety system in the fabric keeps running independently.
5. **Optional (bonus, not required):** cloud sync — PS can push dashboard telemetry to a cloud endpoint over WiFi/Ethernet when connectivity exists, purely as a hybrid-mode convenience layer, never as a dependency for the alert decision itself.

---

## 5. AXI4-Lite Register Map (PS ↔ PL Interface Contract)

| Offset | Register | Access | Purpose |
|--------|----------|--------|---------|
| 0x00 | CONTROL | R/W | Soft reset, global enable |
| 0x04 | STATUS | R | Current FSM state, alert level, fault flags |
| 0x08 | THRESH_GAS | R/W | Gas confidence-fraction critical threshold |
| 0x0C | THRESH_FLAME | R/W | Flame confidence-fraction critical threshold |
| 0x10 | THRESH_ULTRA | R/W | Ultrasonic (fall-detection) threshold |
| 0x14 | THRESH_TEMP | R/W | Temperature threshold |
| 0x18 | WIN_LOG2_0 | R/W | Filter window config, CH0 (gas, debounce) |
| 0x1C | WIN_LOG2_1 | R/W | Filter window config, CH1 (flame, debounce) |
| 0x20 | WIN_LOG2_2 | R/W | Filter window config, CH2 (ultrasonic, true MA) |
| 0x24 | WIN_LOG2_3 | R/W | Filter window config, CH3 (DHT11, true MA) |
| 0x28–0xFC | MLP_WEIGHT_[n] | R/W | Streamed MLP weight load, Q1.15 |
| 0x100–0x1FF | MLP_WEIGHTS | R/W | Full 72-weight block (padded region) |
| 0x200 | ALERT_LEVEL | R | Current alert_level, anomaly_type, confidence (packed) |
| 0x204 | TIMESTAMP | R | 32-bit µs-since-boot counter |
| 0x208–0x214 | CH_RAW_[0..3] | R | Latest raw/filtered value per channel — feeds dashboard plots |

**Contract rule:** every register write from the PS must be reflected in PL logic within one AXI4-Lite handshake cycle; PL must never require a PS write to complete a safety decision — all writable registers are *tuning*, not *triggering*. The read-side status registers (0x200–0x214) are new in this revision, added specifically to support the Jupyter dashboard.

---

## 6. Module List — Build Order, Status, and Responsibility

Build in this order so each module is independently testable before wiring into the next stage. Do not skip standalone testbenches — every module below should pass its own testbench before integration.

| # | Module | Approx. Lines | Depends On | What It Does |
|---|--------|---------------|------------|---------------|
| 1 | `sync_2ff.v` | ~15 | none | Generic 2-flop synchronizer; instantiated once each for MQ-2 D0, Flame D0, HC-SR04 Echo, DHT11 data line |
| 2 | `windowed_filter.v` | ~110 | 1 | Circular buffer + running-sum filter, configurable window (power-of-2). Same core logic serves both "true moving average" (ultrasonic/DHT11) and "majority-vote debounce" (gas/flame) — behavior differs only in how the caller interprets the output |
| 3 | `mq2_interface.v` | ~50 | 1 | Reads synchronized MQ-2 D0, no scaling needed (already binary) |
| 4 | `flame_interface.v` | ~50 | 1 | Reads synchronized Flame D0, no scaling needed (already binary) |
| 5 | `ultrasonic_controller.v` | ~200 | 1 | Drives HC-SR04 trigger, times synchronized echo pulse, converts to distance, enforces ≥60ms retrigger gap |
| 6 | `dht11_controller.v` | ~250 | 1 | Bit-bang single-bus read on synchronized data line, decodes temp+humidity, verifies 8-bit checksum |
| 7 | `fault_detector.v` | ~100 | 5, 6 | Fault logic for ultrasonic (echo timeout) and DHT11 (checksum/timeout) only — see Section 7 for why MQ-2/Flame are excluded |
| 8 | `sensor_aggregator.v` | ~150 | 3, 4, 5, 6 | Timestamps, tags, and FIFOs all 4 channels into core clock domain |
| 9 | `sample_hold_sync.v` | ~80 | 2, 8 | Zero-order hold with `all_seen` guard — must gate on all 4 channels having produced ≥1 valid sample before asserting vector_valid |
| 10 | `feature_extractor.v` | ~180 | 9 | Confidence-fraction + rate-of-change for binary channels; mean/max/min/p2p/Welford variance for multi-bit channels |
| 11 | `threshold_detector.v` | ~100 | 10 | Hard-rule comparators against AXI-configured thresholds |
| 12 | `tiny_mlp.v` | ~250 | 9 (parallel-developable) | Q1.15 fixed-point 4-8-4-2 MLP inference core |
| 13 | `ultrasonic_temp_comp.v` | ~50 | 5, 6 | Speed-of-sound correction using DHT11 temperature |
| 14 | `fusion_engine.v` | ~200 | 10, 11, 13 | Cross-sensor rule evaluation (explosion risk, gas leak, fall detection) |
| 15 | `hybrid_voter.v` | ~80 | 11, 12, 14 | Combines hard-rule and ML verdicts into final alert_level |
| 16 | `decision_pipeline.v` | ~120 | 10–15 | Wraps feature extraction through voting into one pipeline stage |
| 17 | `axi4_lite_reg.v` | ~220 | none (parallel-developable) | AXI4-Lite slave, register map from Section 5, now including status readback |
| 18 | `system_fsm.v` | ~150 | 3, 4, 5, 6, 7 status signals | Top-level INIT→CALIBRATE→ACQUIRE→PROCESS→ALERT/FAULT sequencing |
| 19 | `safety_sentinel_top.v` | ~150 | all of the above | Top-level interconnect, final wrapper for Vivado IP packaging |

**Total: ~2,455 lines of RTL** (slightly less than the analog version — no XADC wizard integration/DRP driver logic needed).

**Non-RTL deliverable:** `dashboard.ipynb` — PYNQ Jupyter notebook, polls AXI4-Lite status registers, renders live alert state + per-channel values using `ipywidgets`/`matplotlib`. This is your actual demo-facing output alongside the LEDs.

**Team parallelization suggestion:**
- **Track A (hardest timing):** `dht11_controller.v` + `ultrasonic_controller.v` + `fault_detector.v` + `sync_2ff.v` instantiations
- **Track B (independent, needs training data):** `tiny_mlp.v` + offline PyTorch training/quantization script
- **Track C (control plane, low risk, reusable pattern from VEDAS/DnCNN experience):** `axi4_lite_reg.v` + `system_fsm.v` + `dashboard.ipynb`
- All converge at `safety_sentinel_top.v`.

---

## 7. Critical Implementation Notes — Read Before Writing RTL

These are correctness requirements, not style preferences. Deviating from these will silently break the design's safety guarantees.

1. **Naming discipline:** the ML block is the **Tiny MLP**, never "BNN." No `torch.sign()` binarization, no Brevitas, no XNOR/popcount logic anywhere — plain Q1.15 fixed-point post-training quantization only.

2. **Every raw async digital input gets its own `sync_2ff` instance before any other logic touches it** — MQ-2 D0, Flame D0, HC-SR04 Echo, DHT11 data line. None of these share a clock relationship with the PL's 100 MHz core clock; skipping synchronization risks metastability propagating into corrupted pulse-width timing or bit-decode errors that appear intermittently rather than consistently, which is far harder to debug than a hard failure.

3. **MQ-2 and Flame D0 fault detection — acknowledged limitation, do not overclaim it.** A comparator-output-only sensor gives no way to distinguish "genuinely safe, holding at 0" from "sensor physically disconnected or dead, stuck at 0." The `fault_detector.v` module therefore only implements fault logic for ultrasonic (echo timeout) and DHT11 (checksum/timeout) — it does **not** attempt a stuck-at check on MQ-2/Flame D0. State this explicitly in your report/pitch as a known tradeoff of the digital-only, D0-based interfacing choice, rather than silently omitting it — judges respect an honestly-scoped limitation far more than a diagram implying capability that isn't actually there.

4. **`windowed_filter.v` is one generic circular-buffer module reused for two purposes.** For ultrasonic/DHT11, its output (running_sum >> window_log2) is directly the filtered value. For MQ-2/Flame, the same output is a **confidence fraction** — treat it as "how many of the last N samples were asserted," and compare that fraction against a threshold in `threshold_detector.v` rather than treating it as a literal sensor reading. Keep this distinction clear in code comments so a teammate reading `feature_extractor.v` later doesn't accidentally apply mean/variance statistics meant for continuous data onto what's actually a binary confidence measure.

5. **`sample_hold_sync.v` must implement an `all_seen` guard.** `vector_valid` must be `all_seen && ch3_valid`, where `all_seen = seen_0 & seen_1 & seen_2 & seen_3`, each a sticky flag set on that channel's first valid sample post-reset. Without this, the MLP could consume uninitialized register values during the CALIBRATE/warm-up window.

6. **Voltage-safety must be checked per your actual breakout board's supply voltage**, not assumed: if MQ-2 D0, Flame D0, or DHT11's data line run at 5V logic (comparator/sensor powered from the 5V rail), they need the same resistor-divider treatment as the HC-SR04 echo pin (e.g., 1kΩ/2kΩ → ~3.33V) before entering the Zynq PL. This is a digital-logic-level concern now, not an analog-scaling concern, but it's just as real — a 5V signal into a 3.3V-max PL input bank is a hardware risk regardless of whether the signal is analog or digital.

7. **FAULT must disable the MLP.** When `fault_flag` is asserted, `decision_pipeline` must force hard-rules-only mode.

8. **Hybrid voter priority is fixed and must not be made "smarter."** `CRITICAL` from hard rules can never be downgraded by the ML output, under any circumstance.

9. **`WARMUP_CYCLES` in `system_fsm.v` must be a defined localparam**, computed from the target clock (100 MHz) and MQ-2's real preheat time (~30s) — via a prescaled counter, since 30s × 100 MHz ≈ 3×10⁹ cycles.

10. **HC-SR04 needs ≥60ms between successive triggers** — fire too fast and a residual echo from the previous ping can falsely trigger the next reading. Build this settle time into `ultrasonic_controller.v`'s FSM as a mandatory state, not an assumption.

11. **DHT11 needs ≥1 second between reads.** This is already reflected in CH3's minimal filter window and the ZOH alignment to DHT11's update rate — don't try to poll it faster for a "smoother demo," it will return corrupted/timeout reads.

12. **MLP weight provenance:** weights are trained offline in PyTorch (float32), post-training quantized to Q1.15 (round-to-nearest, clamp to representable range), exported to a `.coe` file for Vivado BRAM initialization, and re-loadable at runtime via AXI4-Lite without resynthesis. Document your training data source (real sensor logs vs. synthesized) — judges will ask.

---

## 8. Feasibility Summary (Quick Reference)

Going fully digital removed the XADC/analog-scaling risk entirely — the remaining technical risk is concentrated in:

- **DHT11 single-bus bit-banging** — µs-level timing precision required; budget real debug time here.
- **Tiny MLP training data** — needs real or realistically-synthesized labeled sensor data to generalize; start collecting/simulating early.
- **Getting the 2-flop synchronizers right on all four async inputs** — easy to forget, hard to debug if skipped (intermittent failures, not consistent ones).

Everything else (GPIO timers, windowed filters, AXI4-Lite, threshold logic, fusion rules, voting) is well-understood, low-resource-cost, and low-risk on a Zynq-7020's available fabric (~53K LUTs, 220 DSP48E1s, 4.9Mb BRAM) — this system uses a small fraction of the chip's total capacity, arguably even less than the analog-front-end version since there's no XADC wizard/DRP integration to get right.

---

## 9. The Pitch (For Slides / Verbal Defense)

> A worker descends into a manhole. Our FPGA-based edge AI system watches four things: gas, flame, depth, and temperature — all read digitally, straight into the fabric, no analog front-end complexity to get wrong. A windowed filter smooths each sensor independently — a true moving average for distance and temperature, a debounce filter for gas and flame so a single noise glitch never triggers a false alarm. A tiny neural network in the fabric learned what "safe" looks like — but it never gets the final word. Hard rules catch the killers instantly, every time, with no exceptions. The ML catches the subtler patterns underneath — like a worker falling, or a fusion of borderline signals the rules alone wouldn't flag yet. All of this happens in under 10 microseconds, entirely inside the chip, with live status streamed straight to a dashboard in front of you. No cloud. No WiFi. No dependency that can fail underground. Just silicon, watching, and never looking away.

---

*This README is the single source of truth for the project's architecture, module responsibilities, and non-negotiable implementation constraints. Any future session picking this project back up should start here before writing or modifying any RTL.*

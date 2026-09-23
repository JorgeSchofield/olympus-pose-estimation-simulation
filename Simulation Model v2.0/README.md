# Olympus Pose Estimation — Reference Simulation Model

MATLAB/Simulink reference model for the Pose Estimation Subsystem (SEP) of the **Olympus**
rover, developed at the Space Systems Laboratory (SETEC), Instituto Tecnológico de Costa
Rica, as part of the ELANaV project.

The subsystem estimates the rover's planar pose — $x$, $y$, $\theta$ — by fusing six-wheel
skid-steer odometry with the yaw gyroscope of an IMU MPU-9250, using an Extended Kalman
Filter. This repository holds the **reference model**: the simulation used to design the
filter, size its error budget, and validate the eventual embedded implementation against a
known ground truth.

**v2.0 models both controllers and the link between them.** Earlier versions simulated the
filter in isolation, with sensor data appearing on the same clock as the plant. That hid the
dominant error source on this platform, which turned out to live in the low-level
controller's timing rather than in the filter. The model now spans plant → LLC → serial link
→ HLC, with the boundary explicit and the two clocks separate.

> **Status: design tool, not a validated reference.** Several kinematic and sensor
> parameters remain uncharacterized (marked `TBD`), and the LLC sub-task costs are
> estimates. The model demonstrates that the algorithm works and predicts the platform's
> timing behaviour; it does not yet demonstrate that the rover will meet its accuracy
> target. `rover_params` prints a consistency report on every run and warns while any
> parameter remains `TBD`.

---

## Quick start

```matlab
>> run_sep_full_demo     % whole chain: plant → LLC → link → HLC
>> run_sep_demo          % estimation agent only, much faster
>> test_sep_model        % 10 unit tests, no toolboxes
>> test_sep_frame        % reproduces the data-contract golden vector
```

`run_sep_full_demo` builds and runs the complete Simulink model, then runs the same model as
a MATLAB loop over the **same step functions and the same noise sequences**, and reports
`max|Simulink − MATLAB|`. That number should be ~0; it is the check that both paths execute
the same algorithm.

Simulink is optional throughout. If it is unavailable or fails to build, both demos fall
back to the MATLAB loop and report the same metrics.

To reproduce the finding that motivates the firmware change, flip one field in
`llc_params.m`:

```matlab
llc.clock_mode = 'software';   % the LLC as it is written today
llc.clock_mode = 'timer';      % the LLC with a free-running timer
```

---

## Architecture

```
  rover_plant          ground truth, resolved per wheel, 0.5 ms grid
       │
  llc_emulator         ATmega2560 main loop, TWO CLOCKS:
       │               t_real drives the plant, t_llc is what the firmware believes
  serial_channel       USART0 → USB at 115200, RAW ASCII frame, loss, corruption
       │
  hlc_agents           acquisition → estimation → comms, depth-1 mailboxes
       │
    metrics            accuracy, latency, delivery, rate
```

Each stage is a **base-step function** (`plant_step`, `llc_step`, `channel_step`,
`hlc_step`) called every 0.5 ms. The Simulink blocks and the MATLAB loop are thin wrappers
around those same functions — one implementation, two wrappers, no duplicated logic in the
`.slx`.

A variable-length LLC cycle is simply a variable number of base steps. That is why the
model can represent a loop whose period is not what it claims to be.

---

## What the filter does

**State (5):**

$$\mathbf{x} = [\,p_x,\ p_y,\ \theta,\ \omega,\ b_\omega\,]^\top$$

**Prediction** is driven by odometry. The LLC transmits **two per-side count accumulators**;
these are converted into a body displacement $\Delta s$ and a heading increment
$\Delta\theta_{enc}$, and the angular rate state is replaced each step by the
encoder-derived rate.

**Correction** is a scalar measurement from the gyroscope, $z = \omega + b_\omega$, applied
in Joseph form. Every measurement in the filter is scalar, so **the filter performs no
matrix inversion** — a deliberate choice for the eventual C port.

**ZARU.** When neither side registers counts, a zero-angular-rate pseudo-measurement is
applied. This is the only mechanism that makes the gyroscope bias observable.

**Slip detection.** The discrepancy $|\omega_{gyro} - b_\omega - \omega_{enc}|$ inflates the
odometry process noise, so the filter falls back on the gyroscope while wheels are slipping.

The accelerometer is read and transmitted but **does not enter the filter**. It serves only
as a rollover detector; see the findings below for why.

The GPS is **never** used as a filter measurement. It serves exclusively as offline ground
truth for validation.

---

## Files

### Codegen-safe core — this is what becomes C for the HLC

| File | Role |
|---|---|
| `sep_geo_params.m` | Geometry and the per-side tick-conversion chain. Single numeric source. |
| `sep_ekf_params.m` | Filter tuning. Single tuning point. |
| `sep_odometry.m` | Two side accumulators → body displacement and heading increment. |
| `sep_ekf_step.m` | Filter core. One prediction–correction cycle. |

### Simulation chain

| File | Role |
|---|---|
| `plant_step.m` / `plant_init.m` | Ground truth, per wheel, with independent slip ratios. |
| `llc_step.m` / `llc_init.m` | ATmega2560 loop: sub-task schedule, two clocks, IMU quantization, i32 accumulators, stall detection. |
| `channel_step.m` / `channel_init.m` | Serial link: frame-level delay, loss, single-byte corruption. |
| `hlc_step.m` / `hlc_init.m` | Four agents with depth-1 mailboxes. |
| `raw_frame_bytes.m` / `raw_frame_from_bytes.m` | RAW ASCII frame, byte-exact with the firmware's `write_u32`/`write_i32`. |

### Parameters, models and tests

| File | Role |
|---|---|
| `sim_params.m` | Numeric source for everything that exists only in simulation. Silent. |
| `llc_params.m` | LLC timing and sensor model. Holds `clock_mode` and `tx_mode`. |
| `rover_params.m` | Master wrapper: provenance, `TBD` inventory, consistency report. |
| `build_sep_full_model.m` | Builds `sep_rover_full.slx` — both controllers and the link. |
| `build_sep_model.m` | Builds `sep_ekf_rover.slx` — estimation agent only. |
| `run_sep_full_demo.m` / `run_sep_demo.m` | Entry points. |
| `test_sep_model.m` / `test_sep_frame.m` | Unit tests. |
| `sep_crc16.m`, `sep_frame_*.m` | Binary framing with CRC-16/CCITT. Not on the live path; kept because `test_sep_frame` reproduces the data contract's golden vector and can run in CI beside the C and Python implementations. |

### Three boundaries worth respecting

**The codegen boundary.** Only the four core files must be codegen-safe. `rover_params.m`
uses `fprintf` and `warning`; `rover_params_quiet.m` silences it with `evalc`. Neither can
live inside a MATLAB Function block, so the Simulink blocks call `sim_params`, `llc_params`,
`sep_geo_params` and `sep_ekf_params` directly. The LLC emulator models somebody else's
firmware and is never generated — there is no reason to keep it codegen-clean.

**The agent boundary.** `sep_odometry` holds all geometry. `sep_ekf_step` knows none of it
and receives physical quantities. This mirrors the split between the data-fusion agent and
the estimation agent in the rover's multi-agent architecture.

**The accumulator boundary.** The acquisition agent deposits **raw accumulators** in the
mailbox, never increments. See the findings.

---

## Parameter status

Every parameter carries a state in `p.status`:

| Tag | Meaning |
|---|---|
| `MED` | Measured and verified in the lab |
| `DER` | Derived from other parameters — do not edit by hand |
| `PROV` | Provisional, from a previous test campaign or a datasheet |
| `TBD` | Pending characterization |

`rover_params` refuses to stay quiet about inconsistencies. It reports the real sampling
rate against the 50 Hz requirement, the clock scale factor, the per-side conversion
asymmetry, and the ticks-per-revolution anomaly described below.

**Correction to earlier versions of this document.** The README used to report a ×178.8
discrepancy between encoder saturation rate and measured ground speed. With the values
currently in the repository the ratio is **×2.80**, inside the band the consistency check
accepts. The 14/09/2026 campaign resolved that anomaly; the text had not been updated.

---

## Results

Conditions: 2 m UMBmark square, gyroscope bias 0.012 rad/s, per-side conversion asymmetry as
measured, encoder quantization, 0.5 % frame loss.

| LLC configuration | Cycle | Sampling rate | Clock reports | Final error | Final heading error |
|---|---:|---:|---:|---:|---:|
| `software` + `blocking` (today) | 30.6 ms | 32.7 Hz | 65.5 % of real time | **42.96 %** of 8.0 m | 118.8° |
| `software` + `interrupt` | 23.3 ms | 43.0 Hz | 85.9 % | — | — |
| `timer` + `blocking` | 20.4 ms | 49.0 Hz | 100 % | — | — |
| `timer` + `interrupt` | 20.0 ms | 50.0 Hz | 100 % | **0.79 %** of 8.0 m | 2.83° |

Target: ≤ 3 % of distance travelled, ≥ 50 Hz sampling.

Gyroscope bias, estimated / true: 0.01200 / 0.01200 rad/s.

On a **straight** run the two clock modes are nearly identical (0.11 % vs 0.07 %), because
displacement comes from counts and counts do not depend on $\Delta t$. The damage appears
**when turning**. That is the signature to look for in real data: error that scales with
accumulated rotation, not with distance.

### Findings that shaped the design

Each of these came out of measurement or from reading the deployed firmware, and each is
reproducible from this repository.

**The LLC has no clock, and that is the dominant error source.** `main.rs` closes its loop
with `elapsed_ms += LOOP_MS` followed by `delay_ms(LOOP_MS)`. There is no timer anywhere in
the firmware. A `delay` at the *end* of a loop is a gap, not a period: the cycle lasts
`LOOP_MS` **plus** all the work, while the timestamp always claims `LOOP_MS`. Measured
against the sub-task schedule, the cycle is ~30 ms and the reported clock runs at ~65 % of
real time. Since $\omega_{enc} = \Delta\theta/\Delta t$ feeds the gyroscope innovation, the
two sensors then contradict each other on every turn. **The 50 Hz requirement is
unreachable by construction**, regardless of what is optimized inside the loop.

Two small changes fix it: wait until a deadline on a free-running timer instead of sleeping
a fixed amount, and make serial transmission interrupt-driven. Of the two, **the timer is
the one that matters** — it alone recovers 49.0 Hz, because the work already fits inside the
20 ms budget. This can be measured today without instrumenting anything: `engine.py` stamps
each TLM with `time.monotonic()` and the frame carries `tick_ms`, so comparing their deltas
between consecutive TLM frames yields the scale factor directly.

**The per-side conversion constant is not a per-wheel constant.** With three wheels summed
into one accumulator, each contributing $d \cdot N_i/(2\pi R_i)$ counts for the same ground
distance, the side constant is

$$k_{side} = \Big(\sum_i \frac{N_i}{2\pi R_i}\Big)^{-1}$$

which is roughly **one third** of a single wheel's constant. Using a representative wheel's
value is a 3× scale error, not a fine adjustment. With the measured spread, the two sides
differ by 3.7 %.

**Summing per side makes the track width the plain arithmetic mean.** The three axles have
different half-tracks (0.2905, 0.2855, 0.3570 m), so per-wheel odometry would need each one
separately. Summing three wheels sums three lever arms, which is exactly three times their
mean — so $B_{nom}$ *is* the arithmetic mean for this formulation. It also makes the
coupling-regime question unobservable and therefore moot: rigid kinematics and
motor-dominated rotation produce identical side sums, and whatever differs is absorbed by
$\chi$.

**Absolute accumulators must survive past the mailbox.** Absolute counts protect against
*frame loss*: a lost frame only lengthens one $\Delta t$. But if the acquisition agent
converts to increments before depositing, that protection is lost one link later — when a
depth-1 mailbox overwrites an unconsumed message, the increment it carried is gone forever
and becomes permanent position error. The mailbox therefore carries raw `(tick, encL,
encR)`, and the consumer differences against what *it* last processed. Overwrite then costs
temporal resolution, not distance. **General rule: convert to increments in the last link,
never before a mailbox that can overwrite.** `test_sep_model` case 10 guards this.

**The accelerometer cannot detect slip on this rover.** The full braking signature is about
0.145 m/s², while **one degree of pitch** projects 0.171 m/s² of gravity onto the
longitudinal axis. A rocker-bogie pitches by degrees continuously, by design. The signal is
buried under tilt, not under noise — which is also the quantitative reason the accelerometer
biases are not in the state vector.

**Gyroscope bias is only observable at rest.** During a sustained turn, $\omega$ and
$b_\omega$ are not separable: the bias absorbs the odometry calibration error and the filter
converges on the *wrong* encoder-derived rate. Restricting the bias update to standstill
fixes it. Practical consequence: **standstill intervals are part of the measurement method,
not a courtesy.**

**Adapt Q, not R.** With odometry in the prediction, encoder uncertainty lives in the
process noise. Inflating the measurement noise during a slip would degrade the gyroscope —
precisely the sensor being relied on at that moment.

**A UMBmark square does not measure distance scale.** A uniform scale error scales the
square, which still closes. The straight-line and square scenarios measure different things
and neither is redundant. Since the gyroscope also makes the filter nearly insensitive to
track width, this reprioritizes calibration: **ticks-per-revolution and wheel radius matter
far more than effective track width.**

### Open anomaly: ticks per revolution

The 14/09/2026 campaign measured ~46 400 counts per wheel revolution, with a **24.8 %
systematic spread** between wheels. Two things are wrong with that:

- it implies 7.2 µm of resolution at the tyre and a ~2111:1 reduction with an 11 PPR Hall
  encoder and ×2 decoding — that motor does not have that gearbox;
- counts per revolution depend on the encoder and the gearbox, not on diameter, so the
  spread has no geometric explanation.

One hypothesis covers both: **edge bounce in the Hall sensor**, which inflates the count and
depends on the magnet–sensor gap, which varies unit to unit. It can be tested without
returning to the bench: the campaign measured at 20, 50 and 80 % PWM and `sep_geo_params`
averages the three. If counts per revolution **rise with speed**, it is bounce. If they are
flat, the spread is real.

---

## Known limitations

- **This is a model of the LLC, not the LLC.** The real firmware runs on an AVR8 without an
  FPU, with interrupts competing for cycles. The sub-task costs in `llc_params.m` are
  estimates until measured with a GPIO toggle and a scope. The model predicts timing
  behaviour; it does not replace it. Closing that gap fully needs hardware-in-the-loop.
- **Time is quantized to the base step.** With `Tb` = 0.5 ms and ~30 ms cycles that is 1.6 %
  — far below the ~50 % clock error under study, but the model cannot resolve anything
  finer. A byte at 115200 takes 86.8 µs, so transmission is modelled per frame, not per byte.
- **Per-wheel slip detection no longer exists.** The LLC exposes only the two side sums, so
  intra-side dispersion is unavailable. One slip signal remains — the gyroscope/encoder
  discrepancy — and it detects *asymmetric* slip only. The plant can still inject per-wheel
  slip, but only to quantify how much error that blind spot lets through.
- **Blind to whole-vehicle bogging.** If all six wheels spin without advancing, encoders and
  gyroscope agree and the filter integrates distance that never happened.
- **Unbounded position drift.** No sensor observes absolute position, so error grows with
  distance. This is why the accuracy requirement is a percentage of distance travelled, and
  why the honest answer to "where is the rover" is a point plus a confidence ellipse.
- **Thermal drift of the gyroscope bias is not tracked.** Because the bias is frozen while
  moving, a bias that drifts with temperature will not be corrected. The real drift has not
  been measured.
- **No slope compensation.** The gyroscope measures in the body frame; at 15° of pitch the
  missing $1/\cos\theta$ term introduces roughly 3.5 % of systematic heading error.
- **The RAW frame has no CRC.** A byte corrupted by motor noise produces a *plausible*
  number that no parser can detect. The only defence on this path is plausibility rejection
  in the estimation agent. `channel_step` can inject corruption to quantify the exposure.
- **The models are not self-contained.** The Simulink blocks read their inputs from the base
  workspace; the `run_*` scripts generate them.

---

## Contributing notes

The four core files must stay codegen-safe. MATLAB Coder rejects, in particular:

- adding a field to a struct after that struct has been read — and reading a field on the
  right-hand side of an expression counts as reading it;
- `persistent` declarations placed after executable statements, or guarded by a different
  variable than the one being initialized;
- variable-size arrays and dynamic allocation.

These are not Simulink quirks. They are the same constraints Embedded Coder will impose when
the estimation agent is generated for the rover's high-level controller.

Two further conventions specific to v2.0:

**Noise enters as a signal, never from `Random Number` blocks.** If Simulink generated its
own noise, the cross-check against the MATLAB loop would be meaningless. With sequences
injected as signals, both paths produce identical results and `max|Simulink − MATLAB|`
remains a useful metric.

**Wheel order is normative: `FR, FL, CR, CL, RR, RL`.** The sides are **not contiguous** —
right is `{1,3,5}`, left is `{2,4,6}` in MATLAB indexing. Use `geo.idx_R` and `geo.idx_L`,
never ranges. `test_sep_model` case 2 fails if this is mixed up.

---

## Context

Undergraduate thesis, Electronic Engineering, Instituto Tecnológico de Costa Rica.
Space Systems Laboratory (SETEC) — ELANaV project.

Related repositories: `rover-low-level-controller` (Rust `no_std`, ATmega2560) and
`olympus-hlc-rpi5` (Yocto image and Python bridge for the Raspberry Pi 5). The LLC model in
this repository tracks firmware v2.20; the dead MPU-6050 driver and the placeholder EKF in
that repository are future-work scaffolding and are deliberately **not** modelled here.

Requirements this model supports: ≥ 50 Hz sensor sampling, ≥ 10 Hz pose output, < 100 ms
end-to-end latency, ≥ 99 % inter-agent message delivery, ≤ 3 % position error against ground
truth.

Method reference: J. Borenstein and L. Feng, "Measurement and correction of systematic
odometry errors in mobile robots," *IEEE Transactions on Robotics and Automation*, 1996.

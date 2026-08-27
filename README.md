# Olympus Pose Estimation — Reference Simulation Model

MATLAB/Simulink reference model for the Pose Estimation Subsystem (SEP) of the **Olympus**
rover, developed at the Space Systems Laboratory (SETEC), Instituto Tecnológico de Costa
Rica, as part of the ELANaV project.

The subsystem estimates the rover's planar pose — $x$, $y$, $\theta$ — by fusing
six-wheel skid-steer odometry with the yaw gyroscope of an IMU MPU-9250, using an Extended
Kalman Filter. This repository holds the **reference model**: the simulation used to design
the filter, size its error budget, and validate the eventual embedded implementation
against a known ground truth.

> **Status: design tool, not a validated reference.** Nine kinematic parameters are still
> uncharacterized (marked `TBD`). The model demonstrates that the algorithm works; it does
> not yet demonstrate that the rover will meet its accuracy target. `rover_params` prints a
> consistency report on every run and warns while any parameter remains `TBD`.

---

## Quick start

```matlab
>> run_sep_demo
```

This generates ground truth for a UMBmark square, synthesizes encoder counts and gyroscope
readings with realistic error sources, builds and runs the Simulink model, cross-checks it
against a pure-MATLAB implementation of the same filter, and reports the accuracy metrics.

Simulink is optional. If it is unavailable or fails to build, the demo falls back to the
MATLAB loop and reports the same metrics.

---

## What the filter does

**State (5):**

$$\mathbf{x} = [\,p_x,\ p_y,\ \theta,\ \omega,\ b_\omega\,]^\top$$

**Prediction** is driven by odometry. The six wheel-count increments are averaged per side
and converted into a body displacement $\Delta s$ and a heading increment
$\Delta\theta_{enc}$; the angular rate state is replaced each step by the encoder-derived
rate.

**Correction** is a scalar measurement from the gyroscope, $z = \omega + b_\omega$, applied
in Joseph form. Every measurement in the filter is scalar, so **the filter performs no
matrix inversion** — a deliberate choice for the eventual C port.

**ZARU.** When no wheel registers counts, a zero-angular-rate pseudo-measurement is applied.
This is the only mechanism that makes the gyroscope bias observable.

**Slip detection.** The discrepancy $|\omega_{gyro} - b_\omega - \omega_{enc}|$ inflates the
odometry process noise, so the filter falls back on the gyroscope while wheels are slipping.

The GPS is **never** used as a filter measurement. It serves exclusively as offline ground
truth for validation.

---

## Files

| File | Role | Codegen-safe |
|---|---|---|
| `sep_geo_params.m` | Geometry and tick-conversion chain. Single numeric source. | yes |
| `sep_ekf_params.m` | Filter tuning. Single tuning point. | yes |
| `sep_odometry.m` | Six encoders → body displacement and heading increment. | yes |
| `sep_ekf_step.m` | Filter core. One prediction–correction cycle. | yes |
| `rover_params.m` | Master file. Wraps the above, adds plant, sensor models, scenarios, parameter provenance and consistency checks. | no |
| `build_sep_model.m` | Builds `sep_ekf_rover.slx` programmatically. | no |
| `run_sep_demo.m` | Entry point: ground truth, sensor synthesis, execution, metrics. | no |

### Two boundaries worth respecting

**The codegen boundary.** `rover_params.m` uses `fprintf` and `warning`, so it cannot cross
into Embedded Coder. It therefore holds no numeric values of its own — it calls
`sep_geo_params` and `sep_ekf_params`, which are the actual source and are codegen-safe.
No duplication, one place to edit each number.

**The agent boundary.** `sep_odometry` holds all geometry — wheel radii, ticks per
revolution, effective track width. `sep_ekf_step` knows none of them and receives physical
quantities. This mirrors the split between the data-fusion agent and the estimation agent in
the rover's multi-agent architecture, so the model and the software architecture do not
contradict each other.

---

## Parameter status

Every parameter carries a state in `p.status`:

| Tag | Meaning |
|---|---|
| `MED` | Measured and verified in the lab |
| `DER` | Derived from other parameters — do not edit by hand |
| `PROV` | Provisional, from a previous test campaign or a datasheet |
| `TBD` | Pending characterization |

`rover_params` refuses to stay quiet about inconsistencies. With the values currently in the
repository it reports a **×178.8 discrepancy** between the encoder saturation rate and the
measured ground speed, which is the open anomaly that gates everything else.

---

## Results

Conditions: 2 m UMBmark square, effective track width miscalibrated by 20 %, 1 % wheel
diameter spread, gyroscope bias 0.012 rad/s, slip events injected.

| Metric | Result |
|---|---|
| Final position error, clockwise | 0.57 % of 7.96 m |
| Final position error, counter-clockwise | 0.98 % of 7.96 m |
| Gyroscope bias, estimated / true | 0.01187 / 0.01200 rad/s |
| Target (project requirement) | ≤ 3 % |

### Findings that shaped the design

Each of these came out of measurement, not argument, and each is reproducible from this
repository.

**Gyroscope bias is only observable at rest.** During a sustained turn, $\omega$ and
$b_\omega$ are not separable: the bias absorbs the odometry calibration error and the filter
converges on the *wrong* encoder-derived rate. Left free, heading ended 58° off. Restricting
the bias update to standstill brings it down to fractions of a degree. Practical
consequence: **standstill intervals are part of the measurement method, not a courtesy.**

**Adapt Q, not R.** With odometry in the prediction, encoder uncertainty lives in the
process noise. Inflating the measurement noise during a slip would degrade the gyroscope —
precisely the sensor being relied on at that moment.

**The inverse formulation is worse here.** Predicting inertially and correcting with
encoders was tested and is 10–20× more sensitive to track-width miscalibration (19.4 % vs
0.78 %). The classic advantage of inertial prediction — bridging the gap between slow
corrections — does not exist on this platform: IMU and encoders arrive in the same frame, at
the same instant, at 50 Hz.

**A UMBmark square does not measure distance scale.** A uniform scale error scales the
square, which still closes. A 10 % wheel-radius error produces under 1 % error on the square
but **9.98 % on a straight run**. The straight-line and square scenarios measure different
things and neither is redundant.

**The gyroscope makes the filter nearly insensitive to track width.** Heading no longer comes
from the difference between wheels, so the effective-track-width error stops dominating. This
reprioritizes calibration: ticks-per-revolution and wheel radius matter far more than
effective track width.

---

## Known limitations

- **Unbounded position drift.** No sensor observes absolute position, so error grows with
  distance. This is why the accuracy requirement is a percentage of distance travelled and
  not an absolute figure, and why the honest answer to "where is the rover" is a point plus
  a confidence ellipse.
- **Thermal drift of the gyroscope bias is not tracked.** Because the bias is frozen while
  moving, a bias that drifts with temperature will not be corrected. In simulation, a modest
  drift pushed the error from 1.42 % to 14.67 %; adding brief standstill intervals at each
  corner brought it back to 3.72 %. The real drift has not been measured yet.
- **Blind to whole-vehicle bogging.** If all six wheels spin without advancing, encoders and
  gyroscope agree and the filter integrates distance that never happened.
- **No slope compensation.** The gyroscope measures in the body frame; at 15° of pitch the
  missing $1/\cos\theta$ term introduces roughly 3.5 % of systematic heading error.
- **The model is not self-contained.** The Simulink blocks read `enc_log` and `gyro_log` from
  the base workspace; `run_sep_demo` generates them.

---

## Contributing notes

The four codegen-safe files must stay that way. MATLAB Coder rejects, in particular:

- adding a field to a struct after that struct has been read — and reading a field on the
  right-hand side of an expression counts as reading it;
- `persistent` declarations placed after executable statements, or guarded by a different
  variable than the one being initialized;
- variable-size arrays and dynamic allocation.

These are not Simulink quirks. They are the same constraints Embedded Coder will impose when
the estimation agent is generated for the rover's high-level controller.

---

## Context

Undergraduate thesis, Electronic Engineering, Instituto Tecnológico de Costa Rica.
Space Systems Laboratory (SETEC) — ELANaV project.

Requirements this model supports: ≥ 50 Hz sensor sampling, ≥ 10 Hz pose output, < 100 ms
end-to-end latency, ≥ 99 % inter-agent message delivery, ≤ 3 % position error against ground
truth.

Method reference: J. Borenstein and L. Feng, "Measurement and correction of systematic
odometry errors in mobile robots," *IEEE Transactions on Robotics and Automation*, 1996.

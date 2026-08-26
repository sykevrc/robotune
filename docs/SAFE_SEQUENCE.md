# Safe characterization sequence

The generated programs intentionally do not jump directly to full-power PID tuning.

## General sequence (any mechanism)

1. **Preflight**
   - Robot on a stand or otherwise physically constrained when possible.
   - Mechanism travel limits verified.
   - Encoder direction verified.
   - Motor direction verified at very low output.
   - Battery sufficiently charged.
   - Operator has an immediate disable/E-stop path.

2. **Static-friction test**
   - Increase command in small increments.
   - Stop as soon as measurable motion occurs.
   - Estimate `kS`.

3. **Progressive ramps**
   - 25%, 50%, 75% output.
   - Record voltage, position, velocity, current, battery voltage.
   - Stop if velocity or time limits are exceeded.

4. **Step tests**
   - Several moderate command levels.
   - Each step is followed by zero output.
   - Estimate dynamic response and `kV` / `kA`.

5. **Coast-down**
   - Zero motor command.
   - Estimate friction and natural decay.

6. **Validation**
   - A separate sequence should be used to validate the model.
   - Never tune and validate on exactly the same samples if evaluating model quality.

## Mechanism-specific safety

### Elevator / slide
Use a physical lower/upper limit and a software position envelope.

### Pivot
Use a physical hard stop or independent limit sensor and keep the first runs well inside the travel range.

### Drivebase (tank / differential)
Start with wheels lifted for motor characterization. Do not run aggressive drive characterization on the floor until the velocity envelope is known.

### Mecanum drive (primary focus)
Mecanum characterization needs **three independent axes**. Treat them separately:

1. **Lifted wheels first (strongly preferred)**
   - Characterize static friction and free-spin velocity on a stand.
   - Confirms motor/encoder polarity and basic `kS` / `kV` before the robot can move.

2. **On-floor axis isolation**
   - **Forward**: all wheels same polarity, moderate ramps → identify translation `kS_f`, `kV_f`.
   - **Strafe**: classic mecanum signs (FL+, FR−, BL−, BR+) → identify strafe `kS_s`, `kV_s` (usually higher than forward).
   - **Rotate**: opposite left/right → identify rotational `kS_r`, `kV_r` (and optionally track-width from odometry).
   - Keep peak command ≤ 50–60% until the velocity envelope is known.
   - Abort on excessive yaw rate, current, or if the robot approaches field boundaries.

3. **Never combine axes in the first characterization runs**
   - Diagonal or mixed commands couple the plant and make least-squares identification unreliable.

4. **Odometry / IMU (optional but valuable)**
   - If `vx`, `vy`, `omega` are logged, the browser can fit chassis-level models in addition to per-wheel models.
   - Otherwise it falls back to average wheel velocity under pure-axis commands.

5. **Software limits**
   - Soft velocity caps per axis.
   - Hard time limit per phase.
   - Driver-hold abort button checked every sample.

### Flywheel
Use a containment strategy and conservative maximum RPM. Do not use a step command that can exceed the flywheel's mechanical rating.

The browser tuner rejects data that is saturated, clipped, incomplete, or inconsistent rather than silently returning gains.

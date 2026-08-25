# Safe characterization sequence

The generated programs intentionally do not jump directly to full-power PID tuning.

## Sequence

1. **Preflight**
   - Robot on a stand or otherwise physically constrained.
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
   - Estimate dynamic response and `kV/kA`.

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

### Drivebase
Start with wheels lifted for motor characterization. Do not run aggressive drive characterization on the floor until the velocity envelope is known.

### Flywheel
Use a containment strategy and conservative maximum RPM. Do not use a step command that can exceed the flywheel's mechanical rating.

The browser tuner should reject data that is saturated, clipped, incomplete, or inconsistent rather than silently returning gains.

# RoboTune characterization JSON

All exporters use the same top-level structure. Mecanum drive is a first-class mechanism.

## Single-DOF (velocity / slide / pivot / one wheel)

```json
{
  "schema": "robotune.characterization.v1",
  "metadata": {
    "platform": "VEX|FTC|FRC|Generic",
    "mechanism": "velocity|slide|pivot|drive|mecanum",
    "units": {
      "time": "s",
      "voltage": "V",
      "position": "platform-native",
      "velocity": "platform-native/s"
    },
    "sequence": "safe-velocity-v1"
  },
  "samples": [
    {
      "time": 0.000,
      "voltage": 0.0,
      "position": 0.0,
      "velocity": 0.0,
      "current": 0.0,
      "batteryVoltage": 12.0,
      "phase": "preflight"
    }
  ]
}
```

## Mecanum / multi-wheel (preferred for drivebases)

When `mechanism` is `"mecanum"` or `"drive"`, samples may include per-wheel and/or chassis fields. The browser concatenates runs and identifies:

- **Wheel-level** feedforward (shared or per corner) from `fl/fr/bl/br` voltage & velocity
- **Chassis-level** models for forward (`vx`), strafe (`vy`), and rotation (`omega`) when those channels are present

```json
{
  "schema": "robotune.characterization.v1",
  "metadata": {
    "platform": "FTC",
    "mechanism": "mecanum",
    "units": {
      "time": "s",
      "voltage": "V",
      "position": "ticks",
      "velocity": "ticks/s",
      "linear": "m/s",
      "angular": "rad/s"
    },
    "trackWidth": 0.35,
    "wheelBase": 0.30,
    "wheelRadius": 0.048,
    "sequence": "safe-mecanum-v1"
  },
  "samples": [
    {
      "time": 0.0,
      "phase": "forward_ramp",
      "batteryVoltage": 12.4,
      "fl": { "voltage": 3.1, "position": 120, "velocity": 800, "current": 2.1 },
      "fr": { "voltage": 3.1, "position": 118, "velocity": 790, "current": 2.0 },
      "bl": { "voltage": 3.1, "position": 121, "velocity": 805, "current": 2.2 },
      "br": { "voltage": 3.1, "position": 119, "velocity": 798, "current": 2.1 },
      "vx": 0.42,
      "vy": 0.01,
      "omega": 0.02
    }
  ]
}
```

### Field rules

| Field | Meaning |
|-------|---------|
| `fl` / `fr` / `bl` / `br` | Front-left, front-right, back-left, back-right motor samples |
| `vx`, `vy`, `omega` | Chassis body velocity (forward, left, CCW) if odometry/IMU available |
| `voltage` (top-level) | Used when only one channel is recorded (legacy) |
| `current`, `batteryVoltage` | Optional; 0 when unavailable |

`phase` identifies the sequence segment (`static_ramp`, `forward_ramp`, `strafe_ramp`, `rotate_ramp`, `step_*`, `coastdown`, …).

The browser can load multiple independent JSON files from repeated runs. Prefer several short, clean runs over one saturated file.

Legacy single-channel arrays (`samples[].voltage` + `velocity`) still work and are treated as one shared wheel model.

# RoboTune characterization JSON

All three exporters use the same top-level structure:

```json
{
  "schema": "robotune.characterization.v1",
  "metadata": {
    "platform": "VEX|FTC|FRC",
    "mechanism": "velocity|slide|pivot|drive",
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

`current` and `batteryVoltage` may be 0 when the platform/controller does not expose them.
`phase` identifies the characterization sequence segment.

The browser can concatenate multiple files from repeated runs. Prefer multiple independent runs over one very long run.

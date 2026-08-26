# RoboTune v0.3 — mecanum-first PIDF characterization

Browser-based system identification and PIDF autotuning for competitive robotics, with **mecanum drive as the primary target**.

## Workflow

```
robot → pure-axis characterization → robotune_mecanum.json → GitHub Pages tool
      → kS/kV/kA per axis → PID search in simulation → export constants
```

## Contents

| Path | Role |
|------|------|
| `index.html` | Client-side lab (forward / strafe / rotate axis tabs) |
| `demo_realistic.json` | Fallback single-channel demo (tool also synthesizes mecanum) |
| `docs/JSON_SCHEMA.md` | Single-DOF + multi-wheel schema |
| `docs/SAFE_SEQUENCE.md` | Safety procedure, mecanum axis isolation |
| `ftc-sdk/` | FTC OpMode — per-wheel JSON for mecanum |
| `frc-wpilib/` | FRC template — same sequence |
| `vex-pros/` | PROS C++ template |

## Mecanum characterization tips

1. **Lifted wheels first** — static friction + free-spin velocity.  
2. **On floor, pure axes only** — never mix forward + strafe in the same characterization run.  
3. **Expect higher kS / kV for strafe** than forward (roller scrub).  
4. Load the JSON, select each axis tab, and run **Characterize & Autotune**.  
5. Export multi-axis constants for chassis feedforward + velocity PID.

The robot programs are templates: configure motors, encoders, limits, and E-stop before any on-robot run.

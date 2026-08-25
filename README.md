# RoboTune v2 characterization package

This package adds realistic demo data and robot-side characterization exporters
for VEX/PROS, FTC SDK, and FRC/WPILib.

## Contents

- `robotune-web/` — browser tool + realistic synthetic demo JSON
- `vex-pros/` — PROS C++ test program
- `ftc-sdk/` — FTC SDK Java OpMode
- `frc-wpilib/` — FRC WPILib Java test program
- `docs/JSON_SCHEMA.md` — common JSON schema
- `docs/SAFE_SEQUENCE.md` — staged characterization/safety procedure

The common workflow is:

robot → staged characterization → robotune.json → GitHub Pages tool → system identification → feedforward → PIDF → validation.

The robot programs are templates and must be configured for the team's actual motor,
encoder, mechanism limits, and electrical hardware before running on a robot.

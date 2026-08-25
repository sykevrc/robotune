# RoboTune VEX PROS characterization program

This is a competition-safe *template* for V5/PROS. Put it in a dedicated
test project, not your match code. Configure the motor port and optional
sensors before running.

The program uses staged, low-output characterization and writes
`/usd/robotune.json`. It intentionally requires an operator to hold the
controller buttons during the test and has software output/time limits.

Sequence:
1. preflight / zero
2. static-friction ramp
3. 25/50/75% ramps
4. step tests
5. coast-down
6. zero and close file

Hardware safety:
- Put the mechanism on a stand when appropriate.
- For slides/arms, use mechanical travel limits and/or a limit switch.
- Have an E-stop/competition disable available.
- Never run this with a mechanism that can contact people.

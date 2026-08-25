# RoboTune FTC SDK characterization

Add `RoboTuneCharacterizationOpMode.java` to your FTC TeamCode module.
Configure the motor name and encoder direction in the file.

The OpMode is intended for a controlled test stand. It requires START,
has a STOP/abort path, applies conservative power limits, enforces a
time limit, and writes JSON to the app's internal storage.

Use the FTC Driver Station to select it as a utility/test OpMode.

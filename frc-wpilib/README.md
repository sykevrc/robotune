# RoboTune FRC WPILib characterization

Add `RoboTuneCharacterization.java` to your robot project and map the motor
in `RobotContainer`/constructor. The example uses a `PWMSparkMax`-style
motor only to keep dependencies generic; replace it with your actual
Spark/CTRE/REV motor controller and encoder.

The program writes `/home/lvuser/robotune.json`. For a USB drive, change
the path to `/media/sda1/robotune.json` after verifying the mount.

WPILib also has its own SysId/DataLog ecosystem; this exporter is intended
to produce the common RoboTune JSON schema so the same browser tool can
analyze VEX/FTC/FRC data. WPILib's DataLogManager provides timestamped
on-robot logging and is a good production upgrade path.

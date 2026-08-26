# RoboTune — FRC / WPILib mecanum characterization

`RoboTuneCharacterization.java` is a template for pure-axis mecanum logging:

- Static friction → forward → strafe → rotate → coast-down  
- Writes `/home/lvuser/robotune_mecanum.json` (per-wheel samples)  
- **A** starts sequence, **B** aborts  

Replace `PWMSparkMax` / `Encoder` channels with your actual controllers (CANSparkMax, TalonFX, etc.).  
Keep first runs with wheels lifted and `MAX_OUTPUT` ≤ 0.45.

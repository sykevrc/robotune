# RoboTune — VEX / PROS mecanum characterization

`main.cpp` logs pure-axis mecanum data to `/usd/robotune_mecanum.json`.

- **A** starts the sequence, **B** aborts  
- Adjust motor ports / reverse flags so equal power drives forward  
- First runs with wheels lifted; keep `MAX_PCT` conservative  

Upload the JSON into the browser tool and tune Forward / Strafe / Rotate separately.

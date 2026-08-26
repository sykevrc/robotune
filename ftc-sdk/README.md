# RoboTune — FTC mecanum characterization

`RoboTuneCharacterizationOpMode.java` logs a pure-axis sequence for mecanum drives:

1. Static friction ramp  
2. Forward ramps + steps  
3. Strafe ramps + steps  
4. Rotate ramps + steps  
5. Coast-down  

Output: `robotune_mecanum.json` in the robot controller app files directory  
(schema matches `docs/JSON_SCHEMA.md` with per-wheel `fl/fr/bl/br` objects).

## Setup

1. Rename motors in the OpMode to match your hardware map (`frontLeft`, …).  
2. Set motor directions so that equal positive power drives **forward**.  
3. Start with wheels **lifted** for the first run.  
4. Keep `MAX_POWER` ≤ 0.45 until you know free-spin velocity limits.  
5. Upload the JSON to the RoboTune GitHub Pages tool; select axes Forward / Strafe / Rotate and tune each.

## Safety

STOP aborts. Soft velocity and time limits are enforced every sample.  
Do not run mixed diagonal commands during characterization.

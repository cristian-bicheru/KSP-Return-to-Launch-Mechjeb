# Kerbal Space Program Automated Return to Launch Site
Python script using kRPC and Mechjeb2 to launch and land a reusable first stage.
Upon landing the first stage, the code switches to the second stage and executes
the final burn(s) to enter the desired orbit.

See demo video at https://youtu.be/avXz_AQ7HDI

## Mod Requirements
- Mechjeb2
- kRPC
- kRPC.Mechjeb
- Kerbal Reusability Expansion (for grid fins, landing legs and large RCS thrusters)

## Craft Reqirements
- The first stage remote control pod must be the root node of the rocket (the
first item placed in the VAB). Otherwise, upon staging KSP will switch control
to the second stage.
- Make sure the first stage does not run out of electric charge before engine
reignition for the landing burn.
- Make sure you have an antenna on both stages, otherwise they can't be controlled.
- Make sure you have lots of RCS control authority, especially for large rockets.
    - The KRE mod adds large RCS thrusters.

## Tunable Parameters
`flight_program.py`
- Target orbit height, inclination. First stage staging point. Autopilot PID parameters.

`Ascent.py`
- Various trajectory control parameters. Max pitch/throttle rates. Max craft dynamic pressure.

`MechjebRTL.py`
- Flip and landing autopilot PID parameters.


## Bugs
- Occasionally when reverting a flight to launch, the `conn.mech_jeb.api_ready`
fails to ever become `True`, which will cause the first stage to fall to Earth.
This is probably a bug in kRPC.Mechjeb.
    - Fix: Revert to VAB first, then proceed to launch from there.
import time

# Import the kRPC library.
# This is used to interface with KSP
import krpc
from krpc.services.spacecenter import SASMode

# Custom Ascent/Return-to-Launch code
from Ascent import AscentGuidance
from MechjebRTL import MechjebRTLGuidance

### TARGET ORBIT ###
TARGET_ORBIT = 400000 #m
TARGET_INCLINATION = 0

# Apogee given by the first stage. I.e, when the trajectory of the rocket
# has this apogee, initiate staging.
FIRST_STAGE_KICK_APOGEE = 140000

# Connect to kRPC
conn = krpc.connect()
vessel = conn.space_center.active_vessel
sc = conn.space_center
mj = conn.mech_jeb

# Set vessel PID parameters
vessel.auto_pilot.time_to_peak = (5, 5, 5)
vessel.auto_pilot.stopping_time = (0.2, 0.1, 0.2)

### Perform ascent
ascent_program = AscentGuidance(FIRST_STAGE_KICK_APOGEE, TARGET_INCLINATION)
ascent_program.perform_manuver(conn, vessel)

### Staging
print("Performing staging...")
conn.ui.message("Deploying second stage...")
vessel.control.rcs = False

# Wait 0.25s for the craft to stabilize
time.sleep(0.25)

stages = vessel.control.activate_next_stage()
# Find the second stage based on the SAS capabilities
for second_stage in stages:
    try:
        second_stage.control.sas = True
        second_stage.control.sas_mode = SASMode.prograde
        break
    except:
        pass

# Allow the second stage to separate a bit to prevent collisions
second_stage.control.throttle = 0.2
time.sleep(0.5)
second_stage.control.throttle = 0.5
time.sleep(0.5)

# Extend second stage solar panels and activate lights
second_stage.control.solar_panels = True
second_stage.control.lights = True

# Throttle up to give it some more speed
second_stage.control.throttle = 1

# Restore RCS
vessel.control.rcs = True

### Return to Launch
rtl_program = MechjebRTLGuidance()
rtl_program.perform_manuver(conn, sc, vessel)

### Switch to second stage and circularize
sc.active_vessel = second_stage
second_stage.lights = True

print("Second stage circularization.")
conn.ui.message("Executing second stage burn...")

ascent = mj.ascent_autopilot
ascent.desired_orbit_altitude = TARGET_ORBIT
ascent.desired_inclination = TARGET_INCLINATION
ascent.enabled = True
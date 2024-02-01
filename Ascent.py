""" Ascent program.

The rocket will roll to the desired orbit inclination, then execute a special
pitch-over manuver. The manuver must be carefully tuned so that the first stage
does not expend too much propellant or stray too far from the launch site
otherwise it will be unable to return.
"""
import time

from krpc.client import Client
from krpc.services.spacecenter import Vessel

from simple_pid import PID

APOGEE_CUTOFF = 1 # fraction of target apogee at which to end the turn
STRAIGHT_OUT = 0.85 # fraction of target apogee to end the pitch program

"""
A lower exponent will make the rocket travel closer to vertical for longer. This
increases second stage propellant requirements but reduces first stage
requirements.
"""
GRAVITY_TURN_EXPONENT = 0.8

"""
This is the final angle achieved in the first stage trajectory. Higher values
increase second stage propellant requirements but reduce first stage
requirements.
"""
STAGING_ANGLE = 30

ASCENT_LOOP_DT = 1/20 # 20 Hz
MAX_PITCH_RATE = 1 # degrees/second
MAX_THROTTLE_RATE = 0.25 # /second
MAX_DYNAMIC_PRESSURE = 45000 # adjust according to craft capabilities

class AscentGuidance:
    """
    Perform a special gravity turn towards the specified orbit.
    """
    def __init__(self, orbit_apogee, orbit_inclination):
        self.target_apogee = orbit_apogee
        self.target_inclination = orbit_inclination
    
    def perform_manuver(self, conn : Client, vessel : Vessel):
        print("Control flow obtained by AscentGuidance.")

        # Set initial course
        vessel.auto_pilot.target_roll = self.target_inclination
        vessel.auto_pilot.target_pitch = 90
        vessel.auto_pilot.target_heading = 90
        vessel.auto_pilot.engage()

        # Takeoff
        vessel.control.rcs = True
        vessel.control.throttle = 1
        vessel.control.activate_next_stage()
        vessel.control.legs = False

        print("Beginning gravity turn...")
        conn.ui.message("Liftoff!")

        # Stream apogee and dynamic pressure data
        apogee = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
        dynamic_pressure = conn.add_stream(
            getattr, vessel.flight(), 'dynamic_pressure'
        )

        # This loop handles throttle down if the aero forces are too high
        pid_throttle = PID()
        pid_throttle.sample_time = ASCENT_LOOP_DT
        pid_throttle.setpoint = 1
        pid_throttle.output_limits = (-1, 1)

        throttle = 1
        m = STAGING_ANGLE/90

        # logging variables
        last_throttle = 1
        throttle_down = False
        max_q = False

        while True:
            time.sleep(ASCENT_LOOP_DT)

            current_apogee = apogee()
            current_dyn_pressure = dynamic_pressure()

            # check if craft apogee is above cutoff
            if current_apogee > self.target_apogee * APOGEE_CUTOFF:
                # throttle down (slowly) for stage separation
                throttle -= 1 * ASCENT_LOOP_DT
                if throttle < 0:
                    vessel.control.throttle = 0
                    break

                vessel.control.throttle = throttle
                continue

            # check if craft is still pitching over
            if current_apogee < self.target_apogee * STRAIGHT_OUT:
                # compute the target pitch angle
                vessel.auto_pilot.target_pitch = 90 * (
                    1 - (current_apogee / (self.target_apogee * STRAIGHT_OUT))
                    * (1-m)
                ) ** GRAVITY_TURN_EXPONENT

            # update PID controller with dynamic pressure and throttle down if
            # needed.
            adjustment = pid_throttle(current_dyn_pressure/MAX_DYNAMIC_PRESSURE)
            throttle += adjustment * MAX_THROTTLE_RATE * ASCENT_LOOP_DT
            throttle = max(0.2, min(1, throttle))
            vessel.control.throttle = throttle

            # log throttle down
            if throttle_down is False and throttle < 0.99:
                throttle_down = True
                print("Throttle down for max q.")
                conn.ui.message("Throttling down for max q...")
            
            # log max q
            if throttle_down is True and max_q is False and throttle > last_throttle:
                max_q = True
                print("Max q")
                conn.ui.message("Max q")
            
            last_throttle = throttle

        print("Ascent profile completed.")
        print("Yielding control flow...")

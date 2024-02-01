""" Return to Launch program.

Executes a retrograde burn until the first stage velocity is cancelled out. Then,
the descent is performed by the Mechjeb landing autopilot. Once the landing is
complete, control flow is transferred back the the main program for second stage
orbital insertion.
"""
import time

from krpc.services.spacecenter import Vessel, SpaceCenter
from krpc.client import Client

RETROGRADE_BURN_LOOP_DT = 1/100 # 100 Hz
RETROGRADE_ANGLE_TOL = 0.005

class MechjebRTLGuidance:
    """
    RTL using mechjeb landing autopilot
    """
    def __init__(self, grid_fins_open_altitude = 40000, rcs_altitude = 25000):
        self.grid_fins_open_altitude = grid_fins_open_altitude
        self.rcs_altitude = rcs_altitude

    def wait_for_threshold(self, *args, threshold):
        # Wait until the argument is lower than the threshold
        value = self.conn.get_call(*args)
        expr = self.conn.krpc.Expression.less_than(
            self.conn.krpc.Expression.call(value),
            self.conn.krpc.Expression.constant_double(threshold))
        event = self.conn.krpc.add_event(expr)
        with event.condition:
            event.wait()

    def perform_manuver(self, conn : Client, sc : SpaceCenter, vessel : Vessel):
        print("Beginning RTL sequence...")
        self.conn = conn
        self.sc = sc
        self.vessel = vessel
        self.mj = conn.mech_jeb

        # Fast pitch PID args
        vessel.auto_pilot.time_to_peak = (1, 1, 1)
        vessel.auto_pilot.stopping_time = (4, 4, 4)

        # Rotate craft to the retrograde position
        retrograde_dir = vessel.flight().retrograde
        vessel.auto_pilot.target_direction = retrograde_dir
        vessel.auto_pilot.engage()

        # Wait for the rotation to complete
        vehicle_dir = conn.get_call(getattr, vessel.flight(), 'direction')

        expr = conn.krpc.Expression
        dists = [
            expr.power(
                expr.subtract(
                    expr.get(expr.call(vehicle_dir), expr.constant_int(i)),
                    expr.constant_double(retrograde_dir[i])
                ),
                expr.constant_double(2)
            )
            for i in range(3)
        ]
        condition = expr.less_than(
            expr.max(expr.create_list(dists)),
            expr.constant_double(RETROGRADE_ANGLE_TOL)
        )
        event = conn.krpc.add_event(condition)
        with event.condition:
            event.wait()
        
        # Once the rotation is within tol, wait 1s for the RCS to stabilize
        time.sleep(1)

        print("Executing retrograde burn...")
        conn.ui.message("Executing retrograde burn...")

        # More precise PID args for landing and retro burn.
        vessel.auto_pilot.time_to_peak = (1.5, 2.5, 1.5)
        vessel.auto_pilot.stopping_time = (0.3, 0.2, 0.3)
        vessel.auto_pilot.overshoot = (0.005, 0.001, 0.005)

        vessel.control.throttle = 0.1
        throttle = 0.1

        # Stream craft speed data and retrograde orientation
        speed = conn.add_stream(
            getattr,
            vessel.flight(sc.launch_sites[0].body.reference_frame),
            'speed'
        )
        retrograde = conn.add_stream(getattr, vessel.flight(), 'retrograde')

        last_speed = speed()

        # Cancel forward velocity
        while True:
            for _ in range(10):
                vessel.auto_pilot.target_direction = retrograde()
                time.sleep(RETROGRADE_BURN_LOOP_DT)

                # Throttle up over 1s
                if throttle < 1:
                    throttle += RETROGRADE_BURN_LOOP_DT
                else:
                    throttle = 1
                vessel.control.throttle = throttle
            
            # Once every 10 iterations, check if speed has been reduced
            # sufficiently
            current_speed = speed()
            if current_speed > last_speed:
                break
            last_speed = current_speed

        print("Retrograde burn complete.")

        # Burn for an additional second to give a bit of backwards kick.
        time.sleep(1)
        vessel.control.throttle = 0

        # Wait until mechjeb is ready
        print("Acquiring mechjeb...")
        while not self.mj.api_ready:
            time.sleep(0.1)
        print("Acquired.")

        conn.ui.message("Fine-tuning descent profile...")

        vessel.auto_pilot.disengage()

        self.mj.landing_autopilot.enabled = True
        self.mj.landing_autopilot.rcs_adjustment = False
        self.mj.landing_autopilot.land_at_position_target()

        # Wait until we're in the atmosphere to deploy grid fins
        self.wait_for_threshold(
            getattr, vessel.flight(), 'mean_altitude',
            threshold=self.grid_fins_open_altitude
        )
        print("Grid fins deployed.")
        conn.ui.message("Deploying grid fins...")
        vessel.control.brakes = True

        # Only allow RCS fine adustment near the ground to conserve RCS propellant
        self.wait_for_threshold(
            getattr, vessel.flight(), 'mean_altitude',
            threshold=self.rcs_altitude
        )
        print("RCS fine adjustment enabled for landing.")
        self.mj.landing_autopilot.rcs_adjustment = True

        # Wait for landing and recover
        print("Awaiting landing confirmation...")
        value = conn.get_call(getattr, vessel, 'recoverable')
        expr = conn.krpc.Expression.equal(
            conn.krpc.Expression.call(value),
            conn.krpc.Expression.constant_bool(True)
        )
        event = conn.krpc.add_event(expr)
        with event.condition:
            event.wait()

        print("Landing confirmed. Recovering vehicle.")
        conn.ui.message("Vehicle has touched down. Recovering...")
        # Instead of using the recover() method, we'll just switch to the
        # second stage and execute the circularization burn.
        #vessel.recover()
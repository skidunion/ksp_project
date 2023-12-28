# ============================================================== #
# Supertos Industries (2012 - 2023)
# [------------------------------------]
# >Author: Supertos, 2023
# >Version: 0.1 Alpha Rev 2
# ============================================================== #
import json
import math
import time
import krpc


class Autopilot:
    def can_next_stage(self):
        """
        Returns whether the next stage can be activated.
        """
        if (
            self.mission_stage < 5
            and self.apoapsis() > 70000
            and self.periapsis() > 70000
        ):
            return True
        if (
            self.mission_stage == 0
            and self.apoapsis() > 10000
            and not self.has_resource("SolidFuel")
        ):
            return True
        elif self.mission_stage == 1 and abs(self.apoapsis() - self.altitude()) < 2000:
            return True
        elif self.mission_stage == 2 and self.apoapsis() > 72000:
            return True
        elif self.mission_stage == 3 and self.achieved_apoapsis:
            return True
        elif self.mission_stage == 4 and self.periapsis() > 70000:
            return True

        return False

    def decouple_this_stage(self):
        """
        Decouples all parts that are assigned to this stage.
        """
        cur_stage = self.vessel.control.current_stage
        parts = self.vessel.parts

        for part in parts.all:
            if (
                part.decoupler
                and part.stage + 1 == cur_stage
                and not part.decoupler.decoupled
                and part.decoupler.staged
            ):
                part.decoupler.decouple()

    def run_engines_this_stage(self):
        """
        Starts all engines on this stage
        """
        cur_stage = self.vessel.control.current_stage
        parts = self.vessel.parts

        for part in parts.all:
            if part.engine and part.stage + 1 == cur_stage:
                part.engine.thrust_limit = 1
                part.engine.active = True

    def get_engines_this_stage(self):
        """
        Returns all engines assigned to this stage.
        """
        cur_stage = self.vessel.control.current_stage
        parts = self.vessel.parts
        out = []
        for part in parts.all:
            if part.engine and part.stage + 1 == cur_stage:
                out.append(part.engine)
        return out

    def stop_engines_this_stage(self):
        """
        Stops all engines assigned to this stage.
        """
        cur_stage = self.vessel.control.current_stage
        parts = self.vessel.parts

        for part in parts.all:
            if part.engine and part.stage + 1 == cur_stage:
                part.engine.active = False

    def next_stage(self):
        """
        Activates the next stage of the vessel.

        This includes decoupling the parts and starting engines at the next stage.
        """
        if not self.has_resource("SolidFuel") and not self.has_resource("LiquidFuel"):
            self.vessel.control.activate_next_stage()
            self.decouple_this_stage()
            self.run_engines_this_stage()

    def initialize_vessel(self):
        """
        Starts the vertical ascent.
        """
        self.vessel.control.sas = False
        self.vessel.control.rcs = True
        self.vessel.control.throttle = 1.0

        self.vessel.auto_pilot.engage()
        if not self.can_next_stage():
            self.vessel.control.activate_next_stage()
            self.vessel.auto_pilot.target_pitch_and_heading(90, 90)
            self.on_vessel_stage_change()
        else:
            print("Attempting to skip stages...")
            while self.can_next_stage():
                print(f"Skipping stage {self.mission_stage}")
                self.mission_stage += 1
                self.on_vessel_stage_change()

    # ================================= #
    # *__init__
    # Autopilot constructor
    def __init__(self, vessel, conn):
        self.vessel = vessel

        self.altitude = conn.add_stream(getattr, vessel.flight(), "mean_altitude")
        self.apoapsis = conn.add_stream(getattr, vessel.orbit, "apoapsis_altitude")
        self.periapsis = conn.add_stream(getattr, vessel.orbit, "periapsis_altitude")
        self.ut = conn.add_stream(getattr, conn.space_center, "ut")

        self.conn = conn

        self.mission_stage = 0
        self.relative_x = 0
        self.achieved_apoapsis = False
        self.ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
            position=self.vessel.orbit.body.reference_frame,
            rotation=self.vessel.surface_reference_frame,
        )

        self.last_time_update = time.time()
        self.initialize_vessel()

    def compute_landing_loc(self, delta_speed=0):
        """
        Computes the landing trajectory and returns the distance from the current spot.
        """
        y, x = self.altitude(), 0
        flight = self.vessel.flight(self.ref_frame)
        mass = self.vessel.mass

        v_x, v_y = flight.horizontal_speed - delta_speed, flight.vertical_speed

        delta_t, t = 1, 0

        kerbin_mass, g = 5.2915158 * 10**22, 6.67430 * 10**-11
        kerbin_radius = 600000

        g_kerbin_mass = g * kerbin_mass
        pod_square = 2.5**2 * math.pi
        while y > 0:
            # Current Speed computation
            v = math.sqrt(v_x**2 + v_y**2)

            air_density = 1.21325 * math.exp(-y / 5000)

            drag = 0.5 * air_density * v**2 * pod_square * 0.95
            drag_x = drag / v * v_x
            drag_y = drag / v * v_y

            g = g_kerbin_mass / ((kerbin_radius + y) ** 2) * 0.41

            a_x = abs(drag_x) / mass * -v_x / abs(v_x)
            a_y = -g + abs(drag_y) / mass * -v_y / abs(v_y)
            v_x += a_x * delta_t
            v_y += a_y * delta_t
            x += v_x * delta_t
            y += v_y * delta_t

            t += delta_t
        return x

    def on_vessel_stage_change(self):
        """
        Called when the vessel's stage gets changed.

        This function handles the autopilot logic: depending on the stage, it may rotate the ship
        in some way, to achieve the desired result.
        """
        self.run_engines_this_stage()
        if self.mission_stage == 0:
            self.vessel.auto_pilot.target_pitch_and_heading(90, 90)
        elif self.mission_stage == 2:
            self.vessel.auto_pilot.target_pitch_and_heading(45, 90)
            self.vessel.control.throttle = 1.0
        elif self.mission_stage == 1:
            self.vessel.auto_pilot.target_pitch_and_heading(45, 90)
            self.vessel.control.throttle = 0.0
        elif self.mission_stage == 3:
            self.vessel.control.throttle = 0.0
        elif self.mission_stage == 5:
            self.vessel.control.throttle = 0.0
            # print("Vessel has successfully reached the orbit! Awaiting input...")
            # input()
            x, y, z = self.vessel.flight(self.ref_frame).velocity
            tot_speed = math.sqrt(x**2 + y**2 + z**2)

            new_x = x / tot_speed * (tot_speed - 520)
            new_y = y / tot_speed * (tot_speed - 520)
            new_z = z / tot_speed * (tot_speed - 520)

            print(
                f"Horizontal Speed: {math.sqrt(new_z ** 2 + new_y ** 2)}, Vertical Speed: {new_x}"
            )

            self.vessel.control.throttle = 0.0
            self.run_engines_this_stage()
        else:
            self.vessel.control.throttle = 1.0
            self.run_engines_this_stage()

    def tick(self):
        """
        Main autopilot logic. 
        """
        warp_speed_factor = 0.025

        x, orbit_height, z = self.vessel.position(self.ref_frame)
        planet_radius = 600000  # planet_radius = math.sqrt( 600000**2 - orbit_height**2 ) # Shall be computed on launch.

        planet_length = planet_radius * 2 * math.pi
        distance_scale = planet_radius / (planet_radius + self.altitude())

        self.relative_x += (
            (self.vessel.flight(self.ref_frame).horizontal_speed * distance_scale)
            * (time.time() - self.last_time_update)
            * self.conn.space_center.warp_rate
        )
        self.last_time_update = time.time()
        self.relative_x = self.relative_x % planet_length

        self.last_time_update = time.time()

        start_distance = (
            self.relative_x > planet_length * 0.5
            and planet_length - self.relative_x
            or self.relative_x
        )

        # === UPDATE FLAGS === #
        has_engines = self.mission_stage == 0 or len(self.get_engines_this_stage()) > 0
        solids_exhausted = self.mission_stage == 0 and not self.has_resource(
            "SolidFuel"
        )
        liquids_exhausted = (
            self.mission_stage != 0
            and self.get_engines_this_stage()[0].available_thrust <= 0
        )
        can_decouple = self.vessel.control.current_stage > 0

        if can_decouple and (solids_exhausted or liquids_exhausted or not has_engines):
            print(
                f"Vessel stage {self.vessel.control.current_stage} exhausted, decoupling...\nCurrent resources: "
            )
            for res in self.vessel.resources.all:
                print(f"{res.name} (x{res.amount}) at {res.part.stage} vessel stage")
            print(
                f"Following engines will be decoupled: {self.get_engines_this_stage()}"
            )
            print("=" * 32)
            self.next_stage()
            self.on_vessel_stage_change()

        if self.can_next_stage():
            self.mission_stage += 1
            self.on_vessel_stage_change()
            print(f"New mission stage {self.mission_stage} achieved!")

        if (
            not self.achieved_apoapsis
            and self.apoapsis() > 70000
            and abs(self.altitude() - self.apoapsis()) < 50
        ):
            self.achieved_apoapsis = True
            print("Achieved apoapsis!")
        if self.mission_stage == 0:
            if self.altitude() > 8000:
                self.vessel.auto_pilot.target_pitch_and_heading(75, 90)
        elif 2 < self.mission_stage < 5:
            if self.achieved_apoapsis:
                vv = self.vessel.flight(self.ref_frame).vertical_speed
                self.vessel.auto_pilot.target_pitch_and_heading(
                    vv < 0 and min(60, abs(vv) * 5) or 0, 90
                )
            else:
                self.vessel.auto_pilot.target_pitch_and_heading(45, 90)
        elif self.mission_stage == 5:
            # print(f"Space Center distance: {start_distance}, Projected Horizontal Speed: {self.vessel.flight( self.ref_frame ).horizontal_speed*distance_scale}")
            self.vessel.auto_pilot.disengage()
            self.vessel.control.sas = True
            self.vessel.control.sas_mode = self.vessel.auto_pilot.sas_mode.retrograde
            self.vessel.auto_pilot.sas = True
            self.vessel.auto_pilot.sas_mode = self.vessel.auto_pilot.sas_mode.retrograde
            self.vessel.control.throttle = 0
            engine_burn_t = self.get_resource("LiquidFuel") / 7.105
            delta_speed = 240 * 1000 * engine_burn_t / self.vessel.mass
            dist_run = (
                engine_burn_t * self.vessel.flight(self.ref_frame).horizontal_speed
            )
            target_dist = self.compute_landing_loc(delta_speed)
            print(
                f"{planet_length - target_dist - self.relative_x - dist_run}m 'til maneuver, {start_distance} to start position, Deacceleration lose: {dist_run}m, landing distance: {target_dist}m "
            )

            if planet_length - target_dist - self.relative_x < dist_run:
                print(f"Initiating landing maneuver!")
                self.mission_stage += 1

        elif self.mission_stage == 6:
            self.run_engines_this_stage()
            self.vessel.control.throttle = 1.0
            self.vessel.auto_pilot.disengage()

            if len(self.get_engines_this_stage()) == 0:
                print("Maneuver completed. Program will now gather landing data...")
                print(
                    "User shall not enter commands and/or control vessel manually if correct trajectory output is required."
                )
                with open("flight_data.txt", mode="a") as f:
                    while True:
                        a = self.step_metrics(self.vessel.flight(self.ref_frame))
                        f.write(json.dumps(a) + "\n")
                        time.sleep(0.5)
        time.sleep(warp_speed_factor)

    def step_metrics(self, flight):
        return {
            "speed": flight.speed,
            "horizontal_speed": flight.horizontal_speed,
            "vertical_speed": flight.vertical_speed,
            "rotation": flight.rotation,
            "direction": flight.direction,
            "prograde": flight.prograde,
            "retrograde": flight.retrograde,
            "atmosphere_density": flight.atmosphere_density,
            "dynamic_pressure": flight.dynamic_pressure,
            "static_pressure": flight.static_pressure,
            "aerodynamic_force": flight.aerodynamic_force,
            "lift": flight.lift,
            "drag": flight.drag,
            "altitude": self.altitude(),
            "mass": self.vessel.mass,
        }

    def has_resource(self, trg_res):
        for res in self.vessel.resources.all:
            if (
                res.name == trg_res
                and res.amount > 0
                and res.part.stage == self.vessel.control.current_stage
            ):
                return True
        return False

    def get_resource(self, trg_res):
        left = 0
        for res in self.vessel.resources.all:
            if res.name == trg_res:
                left += res.amount
        return left


print("Supertos Industries ( 2012-2023 )")
print("Author: Supertos, 2023")
print("Version: Rev 2, Generic")
print("=" * 32)
print("Establishing connection...", end="")
time.sleep(1)
conn = krpc.connect(name="Launch into orbit")
print("OK")

print("Initializing autopilot")
auto = Autopilot(conn.space_center.active_vessel, conn)

while True:
    auto.tick()

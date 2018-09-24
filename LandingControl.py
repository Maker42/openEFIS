# Copyright (C) 2016  Garrett Herschleb
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>

import time, logging, math

import PID
import Common.FileConfig as FileConfig
import Common.util as util

logger=logging.getLogger(__name__)

SUBMODE_APPROACH="approach"
SUBMODE_FLARE="flare"

class LandingControl(FileConfig.FileConfig):
    def __init__(self, flight_cont, att_cont, throttle_control, gear_control,
                 flap_control, sensors, callback):
        # Dynamic Input parameters
        self.RunwayEndpoints = None
        self.PatternAltitude = 0
        self.RunwayAltitude = 0
        self._desired_climb_rate = 0
        self.JournalPitch = False
        self.JournalThrottle = False
        self.JournalFileName = ''

        # Current State properties
        self._journal_file = None
        self._journal_flush_count = 0
        self._in_pid_optimization = ''
        self._pid_optimization_goal = 0
        self._pid_optimization_scoring = None

        # Computed private operating parameters
        self._desired_pitch = 0
        self._last_pitch = 0

        self._flight_mode = SUBMODE_APPROACH
        self._callback = callback

        # Configuration properties
        self.ApproachAirSpeed = 0
        self.PatternAirSpeed = 0
        self.FinalAirSpeed = 0
        self.ShortFinalAirSpeed = 0
        self.DescentAngle = 3.0
        self.FlareDescentCurve = None # descent fpm for feet agl
        self.InitialFlaps = .2
        self.FlarePowerCurve = [(0.4, 0.0), (0.6, 0.05)]
        self.SideSlipLimits = (-5.0, 5.0)
        self.SideSlipCurve = [(0.0, 0.0), (20.0, -10.0), (100.0, -100.0)]
        self.SlipPIDSampleTime = 10
        self.SlipPIDTuningParams = None
        self.ThresholdAgl = 50.0
        self.MaxPitchChangePerSample = 2.0
        self.PitchPIDSampleTime = 100
        self.PitchPIDLimits = (0.0, 15.0)
        self.PitchPIDTuningParams = None
        self.ClimbRateAchievementSeconds = 2.0

        # Operational properties
        self._SlipPID = None
        self._slip_history = list()
        self._last_side_error = 0.0
        self._attitude_control = att_cont
        self._flight_control = flight_cont
        self._sensors = sensors
        self._throttle_control = throttle_control
        self._gear_control = gear_control
        self._flap_control = flap_control
        self._approach_directives = list()
        self._approach_index = 0
        self._last_update_time = 0
        self._flarePitchPID = None

        FileConfig.FileConfig.__init__(self)

    def initialize(self, filelines):
        self.InitializeFromFileLines(filelines)
        if self.ApproachAirSpeed == 0:
            self.ApproachAirSpeed = self._callback.StallSpeed * 1.7
        if self.PatternAirSpeed == 0:
            self.PatternAirSpeed = self._callback.StallSpeed * 1.5
        if self.FinalAirSpeed == 0:
            self.FinalAirSpeed = self._callback.StallSpeed * 1.3
        if self.ShortFinalAirSpeed == 0:
            self.ShortFinalAirSpeed = self._callback.StallSpeed * 1.2

        ms = util.millis(self._sensors.Time())
        kp,ki,kd = self.SlipPIDTuningParams
        self._SlipPID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)
        mn,mx = self.SideSlipLimits
        self._SlipPID.SetOutputLimits (mn, mx)
        kp,ki,kd = self.PitchPIDTuningParams
        self._flarePitchPID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)
        mn,mx = self.PitchPIDLimits
        self._flarePitchPID.SetOutputLimits (mn, mx)
        self._flarePitchPID.SetSampleTime (self.PitchPIDSampleTime)

    # Notifies that the take off roll has accompished enough speed that flight controls
    # have responsibility and authority. Activates PIDs.
    def Start(self, last_desired_pitch):
        logger.debug ("Landing Control Starting")
        self._last_desired_pitch = last_desired_pitch
        self._approach_directives = list()
        self._approach_index = 0

    def Stop(self):
        self._flarePitchPID.SetMode (PID.MANUAL, 0, self._last_pitch)
        return self._last_pitch

    def Land(self, runway_points, runway_alt, outer_marker=None, outer_altitude=0,
                   middle_marker=None, right_pattern=False, pattern_alt=0, threshold_agl = -9999):
        self._flight_control.Start(self._last_desired_pitch)
        self._flight_mode = SUBMODE_APPROACH
        # If given an outer and middle marker, fly to the outer marker and approach from there.
        # otherwise use the pattern as appropriate
        self._outer_marker = outer_marker
        self._middle_marker = middle_marker
        self._right_pattern = right_pattern
        self.RunwayEndpoints = runway_points
        self.RunwayAltitude = runway_alt
        if threshold_agl != -9999:
            self.ThresholdAgl = threshold_agl
        flare_altitude = self.RunwayAltitude+self.ThresholdAgl
        self._flight_control.SetCallback (self)
        if pattern_alt > 0:
            self.PatternAltitude = pattern_alt
        else:
            self.PatternAltitude = self.RunwayAltitude+1000.0
        if outer_altitude > 0:
            self._outer_altitude = outer_altitude
        else:
            self._outer_altitude = self.PatternAltitude + 500.0

        rel_heading,dist,heading,self.rel_lng = util.VORDME(self._sensors.Position(),
                self.RunwayEndpoints)
        turn_radius = ((360.0 * (self.FinalAirSpeed / 60.0) / self._flight_control.TurnRate)
                       / (4 * util.M_PI))
        short_final_point = util.AddPosition (self.RunwayEndpoints[0],
                turn_radius, heading - 180)
        short_final_altitude = turn_radius * math.sin (self.DescentAngle * util.RAD_DEG) * util.FEET_NM
        short_final_altitude += flare_altitude
        if outer_marker != None and middle_marker != None:
            logger.debug("Making an instrument approach landing. Flying to outer marker")
            approach_course = [outer_marker, middle_marker]

            turn_radius = ((360.0 * (self.ApproachAirSpeed / 60.0) / self._flight_control.TurnRate)
                           / (4 * util.M_PI))
            if self._sensors.Altitude() > self._outer_altitude:
                altitude = self._outer_altitude
            else:
                altitude = 0    # Maintain current altitude
            if abs(rel_heading) < 45:
                logger.debug ("Inserting to approach course from opposing quadrant")
                insertion_point = util.AddPosition(outer_marker, heading+90, turn_radius*2)
                self._approach_directives.append ((self._flight_control.FlyTo,
                    (insertion_point, altitude)))
                self._approach_directives.append ((self._flight_control.TurnTo,
                    (heading, -1)))
            elif rel_heading > 0 and rel_heading < 135:
                logger.debug ("Inserting to approach course from upper quadrant")
                insertion_point = util.AddPosition(outer_marker, heading+90, turn_radius)
                self._approach_directives.append ((self._flight_control.FlyTo,
                    (insertion_point, altitude)))
                self._approach_directives.append ((self._flight_control.TurnTo,
                    (heading, -1)))
            elif rel_heading < 0 and rel_heading > -135:
                logger.debug ("Inserting to approach course from lower quadrant")
                insertion_point = util.AddPosition(outer_marker, heading-90, turn_radius)
                self._approach_directives.append ((self._flight_control.FlyTo,
                    (insertion_point, altitude)))
                self._approach_directives.append ((self._flight_control.TurnTo,
                    (heading, 1)))
            else:
                logger.debug ("Inserting to approach course straight in")
                insertion_point = outer_marker
                self._approach_directives.append ((self._flight_control.FlyTo,
                    (outer_marker, altitude)))
            self._approach_directives.append ((self._flight_control.FlyCourse,
                (approach_course, self._outer_altitude, self.PatternAirSpeed, False)))
            self._approach_directives.append ((self.DropGearAndFlaps, tuple()))
            final_course = [middle_marker, short_final_point]
        else:
            logger.debug("Making an pattern approach landing.")
            turn_radius = ((360.0 * (self.PatternAirSpeed / 60.0) / self._flight_control.TurnRate)
                           / (4 * util.M_PI))
            if right_pattern:
                pattern_offset_heading = heading + 90
            else:
                pattern_offset_heading = heading - 90

            downwind_point = util.AddPosition(self.RunwayEndpoints[1],
                    turn_radius * 2.2, pattern_offset_heading)
            descent_distance = turn_radius * 2.2
            base_distance = turn_radius * 2.2
            total_approach_distance = descent_distance * 2 + base_distance
            descent_point = util.AddPosition (self.RunwayEndpoints[0],
                    base_distance, pattern_offset_heading)
            base_point = util.AddPosition (descent_point,
                    base_distance, heading - 180)
            outer_base_point = util.AddPosition (base_point,
                    turn_radius * 4.2, pattern_offset_heading)
            final_point = util.AddPosition (self.RunwayEndpoints[0],
                    turn_radius * 2.2, heading - 180)

            pattern_alt_nm = total_approach_distance * math.sin(self.DescentAngle * util.RAD_DEG)
            self.PatternAltitude = pattern_alt_nm * util.FEET_NM + flare_altitude
            final_turn_altitude = descent_distance * math.sin(self.DescentAngle * util.RAD_DEG)
            final_turn_altitude *= util.FEET_NM
            base_turn_altitude = self.PatternAltitude - final_turn_altitude
            final_turn_altitude += flare_altitude
            logger.debug ("Pattern altitude %g feet", self.PatternAltitude)
            outer_final_point = util.AddPosition (final_point,
                    turn_radius * 4.2, heading - 180)

            if self._sensors.Altitude() > self._outer_altitude:
                altitude = self._outer_altitude
            else:
                altitude = self.PatternAltitude

            include_downwind = False
            include_base = False

            if abs(rel_heading) > 135:
                logger.debug ("Making straight in final")
                insertion_point = outer_marker
                self._approach_directives.append ((self._flight_control.FlyTo,
                    (outer_final_point, self.PatternAltitude)))
                self._approach_directives.append ((self.DropGearAndFlaps, tuple()))
            elif (((not right_pattern) and rel_heading >= 10) or
                 (right_pattern and rel_heading <= -10)):
                logger.debug ("Inserting into pattern via crosswind")
                insertion_point = self.RunwayEndpoints[1]
                crosswind_course = [self.RunwayEndpoints[1], downwind_point]
                self._approach_directives.append ((self._flight_control.FlyCourse,
                    (crosswind_course, altitude, self.ApproachAirSpeed, True)))
                include_downwind = True
                include_base = True
            elif (((not right_pattern) and rel_heading < 10 and rel_heading > -60) or
                   (right_pattern) and rel_heading > -10 and rel_heading < 60):
                logger.debug ("Inserting into downwind pattern")
                include_downwind = True
                include_base = True
            else:
                if right_pattern:
                    logger.debug ("Inserting into pattern via right base")
                    assert(rel_heading >= 0)
                else:
                    logger.debug ("Inserting into pattern via left base")
                    assert(rel_heading <= 0)
                include_base = True
                extended_base = [outer_base_point, base_point]
                self._approach_directives.append ((self._flight_control.FlyTo,
                    (outer_base_point, altitude, self.ApproachAirSpeed)))
                self._approach_directives.append ((self.DropGearAndFlaps, tuple()))
                descent_airspeed = self.PatternAirSpeed * .7 + self.FinalAirSpeed * .3
                self._approach_directives.append ((self._flight_control.DescentCourse,
                    (extended_base, base_turn_altitude, descent_airspeed, False)))
                include_base = True

            if include_downwind:
                downwind_course = [downwind_point, descent_point]
                self._approach_directives.append ((self._flight_control.FlyCourse,
                    (downwind_course, self.PatternAltitude, self.PatternAirSpeed, False)))
                self._approach_directives.append ((self.DropGearAndFlaps, tuple()))
                descent_downwind = [downwind_point, base_point]
                descent_airspeed = self.PatternAirSpeed * .7 + self.FinalAirSpeed * .3
                self._approach_directives.append ((self._flight_control.DescentCourse,
                    (descent_downwind, base_turn_altitude, descent_airspeed, True)))
            if include_base:
                base_course = [base_point, final_point]
                descent_airspeed = self.PatternAirSpeed * .5 + self.FinalAirSpeed * .5
                self._approach_directives.append ((self._flight_control.DescentCourse,
                    (base_course, final_turn_altitude, descent_airspeed, True)))

            final_course = [final_point, short_final_point]

        self._approach_directives.append ((self._flight_control.DescentCourse,
            (final_course, short_final_altitude, self.FinalAirSpeed, False)))
        self._approach_directives.append ((self.FullFlaps, tuple()))
        final_course = [short_final_point, self.RunwayEndpoints[0]]
        self._approach_directives.append ((self._flight_control.DescentCourse,
            (final_course, flare_altitude, self.ShortFinalAirSpeed, False)))
        func,args = self._approach_directives[self._approach_index]
        func (*args)

    def Stop(self):
        self._SlipPID.SetMode (PID.MANUAL, 0,0)
        self._slip_history = list()
        self._flight_control.Stop()
        self._flight_control.SetCallback (self._callback)
        if self._journal_file:
            self._journal_file.close()

    def Update(self):
        agl = self._sensors.AGL()
        if agl < 100 and self._flight_mode != SUBMODE_FLARE:
            self._begin_flare()
        if self._flight_mode == SUBMODE_APPROACH:
            self._flight_control.Update()
        elif self._flight_mode == SUBMODE_FLARE:
            ms = util.millis(self._sensors.Time())

            # Compute Pitch
            desired_descent = util.rate_curve(agl, self.FlareDescentCurve)
            self.CurrentClimbRate = self._sensors.ClimbRate()
            self._flarePitchPID.SetSetPoint (desired_descent, self.ClimbRateAchievementSeconds)
            desired_pitch = self._flarePitchPID.Compute (self.CurrentClimbRate, ms)
            logger.log (10, "Climb Pitch PID: %g/%g-->%g",
                    self.CurrentClimbRate, desired_descent, desired_pitch)
            pitch_diff = desired_pitch - self._last_pitch
            time_diff = ms - self._last_update_time
            if abs(pitch_diff) > self.MaxPitchChangePerSample * time_diff / self.PitchPIDSampleTime:
                pitch_diff = self.MaxPitchChangePerSample * time_diff / self.PitchPIDSampleTime
                if desired_pitch < self._last_pitch:
                    pitch_diff *= -1
                self._last_pitch += pitch_diff
            else:
                self._last_pitch = desired_pitch
            self._last_update_time = ms

            # Compute Roll
            side, forward, heading, heading_to_dest = util.CourseDeviation(self._sensors.Position(),
                    self.RunwayEndpoints, self.rel_lng)
            side *= util.FEET_NM
            ground_track = self._sensors.GroundTrack()
            ground_speed = self._sensors.GroundSpeed()
            relative_ground_track = ground_track - heading
            if relative_ground_track > 180:
                relative_ground_track -= 360
            elif relative_ground_track < -180:
                relative_ground_track += 360
            shift_rate = ground_speed * math.sin(relative_ground_track * util.RAD_DEG)
            shift_rate *= util.FEET_NM / (3600.0) # nm / hour --> feet / s
            shift_rate = util.LowPassFilter (shift_rate, self._slip_history)
            self._last_side_error = side
            desired_shift_rate = util.rate_curve (side, self.SideSlipCurve)
            self._SlipPID.SetSetPoint (desired_shift_rate, 2.0)
            roll = self._SlipPID.Compute (shift_rate, ms)
            self._throttle_control.Set(util.rate_curve (agl, self.FlarePowerCurve))
            logger.log(10, "Flaring with side error=%g, shift_rate=%g/%g, roll=%g, agl=%g, pitch=%g/%g",
                    side, shift_rate, desired_shift_rate, roll, agl,
                    self._last_pitch, desired_pitch)
            self._attitude_control.UpdateControls(self._last_pitch, roll, heading)
            if self._sensors.AirSpeed() < self._callback.StallSpeed * .4:
                self._throttle_control.Set(0.0)
                # Landed
                self.get_next_directive()

    def GetNextDirective(self):
        self._approach_index += 1
        if self._approach_index >= len(self._approach_directives):
            self._begin_flare()
        else:
            func,args = self._approach_directives[self._approach_index]
            func (*args)

    def _begin_flare(self):
        # TODO: Make go-around decision
        logger.info ("Beginning touchdown flare")
        self._flarePitchPID.SetMode (PID.AUTOMATIC,
                                    self._sensors.ClimbRate(), self._last_desired_pitch)
        self._flight_control.SetCallback (self._callback)
        self._flight_control.Stop()
        self._flight_mode = SUBMODE_FLARE
        self._attitude_control.StartFlight(yaw_mode = 'side_slip')
        self._SlipPID.SetMode (PID.AUTOMATIC, 0, 0)
        self._SlipPID.SetSetPoint (0.0, 5.0)
        self._last_pitch = self._sensors.Pitch()
        self._last_update_time = util.millis(self._sensors.Time())

    def DropGearAndFlaps(self):
        logger.info("Landing Control deploying landing gear and flaps")
        if self._gear_control:
            self._gear_control.Set(1)
        if self._flap_control:
            self._flap_control.Set(self.InitialFlaps)
        self.GetNextDirective()

    def FullFlaps(self):
        logger.info("Landing Control deploying full flaps")
        if self._flap_control:
            self._flap_control.Set(1.0)
        self.GetNextDirective()

    def get_next_directive(self):
        self._callback.GetNextDirective()

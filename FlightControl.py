# Copyright (C) 2015  Garrett Herschleb
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

import time, logging, math, copy

import PID, ControlComplex
import FileConfig
import util

logger=logging.getLogger(__name__)

SUBMODE_COURSE="course"
SUBMODE_TURN="turn"
SUBMODE_STRAIGHT="straight"
SUBMODE_SWOOP_DOWN="swoop_down"
SUBMODE_SWOOP_UP="swoop_up"

class FlightControl(FileConfig.FileConfig):
    def __init__(self, att_cont, throttle_control, sensors, callback):
        # Dynamic Input parameters
        self.DesiredCourse = [(0,0), (0,0)]
        self.DesiredAltitude = 0
        self.DesiredAirSpeed = 0
        self.DesiredTrueHeading = 0
        self._desired_climb_rate = 0
        self.MinClimbAirSpeed = 20.0
        self.JournalPitch = False
        self.JournalThrottle = False
        self.JournalFileName = ''

        # Current State properties
        self.CurrentHeadingRateChange = sensors.HeadingRateChange()
        self.CurrentAltitude = sensors.Altitude()
        self.CurrentAirSpeed = sensors.AirSpeed()
        self.CurrentClimbRate = sensors.ClimbRate()
        self.CurrentPosition = sensors.Position()
        self._journal_file = None
        self._journal_flush_count = 0
        self._in_pid_optimization = ''
        self._pid_optimization_goal = 0
        self._pid_optimization_scoring = None

        # Computed private operating parameters
        self._desired_heading = 0
        self._desired_roll = 0
        self._desired_pitch = 0
        self._current_pitch_mode = None

        self._flight_mode = SUBMODE_COURSE
        self._callback = callback
        self._force_turn_direction = 0
        self._swoop_low_alt = sensors.Altitude()

        # Configuration properties
        self.AltitudeAchievementMinutes = 1.2
        self.ClimbRateLimits = (-1000.0, 1000.0)        # feet / minute
        self.SwoopClimbRateLimits = (-5000.0, 5000.0)   # feet / minute
        self.PitchPIDLimits = [(0,20.0), (45,0)]  # (roll, max degrees)

        self.PitchPIDSampleTime = 1000
        self.ThrottlePIDSampleTime = 1000

        self.ClimbPitchPIDTuningParams = None
        self.AirspeedPitchPIDTuningParams = None
        self.ThrottlePIDTuningParams = None
        self.RollCurve = [(0.0, 0.0), (5.0, 5.0), (10.0, 20.0), (20.0, 30.0)]
        self.ClimbPitchMeasuredFile = None
        self.PitchFuzzyConfig = None
        self.PitchExperienceFile = None

        self.InterceptMultiplier = 35
        self.MaxRoll = 30.0

        self.MaxPitchChangePerSample = 1.0
        self.TurnRate = 180.0         # 180 degrees per minute at max roll
        self.ClimbRateCurve = None

        # Will Use one of the following 2 PIDS to request pitch angle:
        # Input: Current Climb Rate; SetPoint: Desired Climb Rate; Output: Desired Pitch Attitude
        self._climbPitchController = None
        self._climbPitchFuzzy = None
        self._fuzzy_active = False
        self._next_pitch_update_time = 0
        self._last_pitch_parameters = None
        # Input: Current Airspeed; SetPoint: Desired Climb Airspeed; Output: Desired Pitch Attitude
        self._airspeedPitchPID = None

        # Operational properties
        # Input: Current Airspeed; SetPoint: DesiredAirspeed; Output: throttle setting
        self._throttlePID = None
        self._attitude_control = att_cont
        self._sensors = sensors
        self._throttle_control = throttle_control

        FileConfig.FileConfig.__init__(self)

    def initialize(self, filelines):
        self.InitializeFromFileLines(filelines)
        ms = util.millis(self._sensors.Time())

        rmn,rmx = self._get_pitch_pid_limits(self.PitchPIDLimits)
        limits = (rmn,rmx)
        self._climbPitchController = ControlComplex.ControlComplex(ms, self.ClimbPitchPIDTuningParams, self.PitchPIDSampleTime, limits, self.PitchFuzzyConfig, self.ClimbPitchMeasuredFile)

        kp,ki,kd = self.AirspeedPitchPIDTuningParams
        self._airspeedPitchPID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)
        rmn,rmx = self._get_pitch_pid_limits(self.PitchPIDLimits, True)
        self._airspeedPitchPID.SetOutputLimits (rmn, rmx)

        kp,ki,kd = self.ThrottlePIDTuningParams
        self._throttlePID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)

        self._airspeedPitchPID.SetSampleTime (self.PitchPIDSampleTime)
        self._throttlePID.SetSampleTime (self.ThrottlePIDSampleTime)

    # Notifies that the take off roll has accompished enough speed that flight controls
    # have responsibility and authority. Activates PIDs.
    def Start(self, last_desired_pitch):
        logger.debug ("Flight Control Starting")
        mn,mx = self._throttle_control.GetLimits()
        self._throttle_range = (mn,mx)
        self._throttlePID.SetOutputLimits (mn, mx)
        self.CurrentHeadingRateChange = self._sensors.HeadingRateChange()

        self.CurrentAirSpeed = self._sensors.AirSpeed()
        self._throttlePID.SetSetPoint (self.DesiredAirSpeed)
        self._throttlePID.SetMode (PID.AUTOMATIC, self.CurrentAirSpeed, self._throttle_control.GetCurrent())
        self.CurrentAltitude = self._sensors.Altitude()
        self._desired_climb_rate = self.get_climb_rate()
        self.CurrentClimbRate = self._sensors.ClimbRate()

        self._desired_pitch = last_desired_pitch
        if self._throttle_control.GetCurrent() == self._throttle_range[1] and self.CurrentAirSpeed <= self.MinClimbAirSpeed and self._desired_climb_rate > 0:
            self.SelectAirspeedPitch()
        else:
            self.SelectClimbratePitch()
        self._next_pitch_update_time = util.millis(self._sensors.Time()) + self.PitchPIDSampleTime

        self._attitude_control.StartFlight()
        if self.JournalFileName and (not self._journal_file):
            self._journal_file = open(self.JournalFileName, 'w+')
            if self._journal_file:
                self._journal_file.write("Time")
                if self.JournalPitch:
                    self._journal_file.write(",PitchGoal,PitchInput,DesiredPitch")
                if self.JournalThrottle:
                    self._journal_file.write(",ThrottleSet,ThrottleCurrent,Throttle")
                self._journal_file.write("\n")

    def SetPitch(self, ms):
        # Figure out if we're using air speed or climb rate to select pitch:
        if self._current_pitch_mode == "climb":
            if self._throttle_control.GetCurrent() == self._throttle_range[1] and self.CurrentAirSpeed <= self.MinClimbAirSpeed and self._desired_climb_rate > 0:
                # Change to Use airspeed to control pitch
                self.SelectAirspeedPitch()
                ret = self.UpdateAirspeedPitch(ms)
            else:
                ret = self.UpdateClimbratePitch(ms)
        elif self._current_pitch_mode == "airspeed":
            if self._throttle_control.GetCurrent() < self._throttle_range[1] or self.CurrentAirSpeed > self.MinClimbAirSpeed * 1.1 or self._desired_climb_rate < 0:
                self.SelectClimbratePitch()
                ret = self.UpdateClimbratePitch(ms)
            else:
                ret = self.UpdateAirspeedPitch(ms)
        else:
            raise RuntimeError("invalid pitch mode")
        return ret

    def SelectAirspeedPitch(self):
        if self._current_pitch_mode != "airspeed":
            logger.debug("Switching to AirSpeed pitch control")
            if self._current_pitch_mode == "climb":
                # Turn off climb pitch PID
                self._climbPitchController.SetMode (PID.MANUAL, self.CurrentClimbRate, self._desired_pitch)
            # Turn on climb rate Pitch PID
            if self._in_pid_optimization != "airspeed_pitch":
                self._airspeedPitchPID.SetSetPoint (self.MinClimbAirSpeed)
            self._airspeedPitchPID.SetMode (PID.AUTOMATIC, self.CurrentAirSpeed, -self._sensors.Pitch())
            self._current_pitch_mode = "airspeed"

    def SelectClimbratePitch(self):
        if self._current_pitch_mode != "climb":
            logger.debug("Switching to Climb rate pitch control")
            if self._current_pitch_mode == "airspeed":
                # Turn off airspeed pitch PID
                self._airspeedPitchPID.SetMode (PID.MANUAL, self.CurrentClimbRate, self._desired_pitch)
            # Turn on climb rate Pitch PID
            self._climbPitchController.SetMode (PID.AUTOMATIC, self.CurrentClimbRate, self._sensors.Pitch())
            self._current_pitch_mode = "climb"

    def UpdateAirspeedPitch(self, ms):
        rmn,rmx = self._get_pitch_pid_limits(self.PitchPIDLimits, True)
        self._airspeedPitchPID.SetOutputLimits(rmn, rmx)
        desired_pitch = -self._airspeedPitchPID.Compute (self.CurrentAirSpeed, ms)
        if self._in_pid_optimization == "airspeed_pitch":
            self._pid_optimization_scoring.IncrementScore(self.CurrentAirSpeed, desired_pitch, self._pid_optimization_goal, self._journal_file)
        if self._journal_file and self.JournalPitch:
            self._journal_file.write(",%g,%g,%g"%(self.MinClimbAirSpeed, self.CurrentAirSpeed, desired_pitch))
        return desired_pitch

    def UpdateClimbratePitch(self, ms):
        inputs = [self.CurrentClimbRate, self.CurrentAltitude, self.CurrentAirSpeed,
                  self._sensors.Roll(), self._desired_climb_rate]
        limits = self._get_pitch_pid_limits (self.PitchPIDLimits)
        return self._climbPitchController.Compute (inputs, ms, self.PitchExperienceFile, limits)

    def _get_pitch_pid_limits(self, absolute_limits, invert=False):
        amn,amx = self._find_pitch_limits(absolute_limits)
        if invert:
            multiplier = -1
            temp = -amx
            amx = -amn
            amn = temp
        else:
            multiplier = 1
        rmn = multiplier * self._desired_pitch - self.MaxPitchChangePerSample
        rmx = multiplier * self._desired_pitch + self.MaxPitchChangePerSample
        if rmn < amn:
            rmn = amn
        if rmx > amx:
            rmx = amx
        if rmn > rmx:
            rmn = rmx - 1.0
        logger.log (5, "pitch limits are %g,%g", rmn, rmx)
        return (rmn, rmx)

    def _find_pitch_limits(self, absolute_limits):
        if isinstance(absolute_limits,tuple):
            return absolute_limits
        roll = abs(self._sensors.Roll())
        max_pitch = util.rate_curve(roll, absolute_limits)
        return -max_pitch,max_pitch

    def Stop(self):
        self._throttlePID.SetMode (PID.MANUAL, self.CurrentAirSpeed, self._throttle_control.GetCurrent())
        self._climbPitchController.SetMode (PID.MANUAL, self.CurrentAirSpeed, self._desired_pitch)
        self._airspeedPitchPID.SetMode (PID.MANUAL, self.CurrentClimbRate, self.DesiredAirSpeed)
        self._attitude_control.StopFlight()
        if self._journal_file:
            self._journal_file.close()
        return self._desired_pitch

    def Update(self):
        ms = util.millis(self._sensors.Time())
        self.CurrentAirSpeed = self._sensors.AirSpeed()
        self.CurrentTrueHeading = self._sensors.TrueHeading()
        if self._flight_mode == SUBMODE_TURN:
            self._desired_roll = self.compute_roll(self.DesiredTrueHeading)
            if self.completed_turn():
                self.get_next_directive()
                return
        else:
            self.CurrentPosition = self._sensors.Position()
            if self._flight_mode == SUBMODE_SWOOP_DOWN:
                if self.completed_course():
                    self._flight_mode = SUBMODE_SWOOP_UP
                    self.DesiredAirSpeed = self._swoop_min_as
                    self.DesiredAltitude = self._swoop_high_alt
                elif self.CurrentAirSpeed > self.DesiredAirSpeed:
                    # Going too fast. Better throttle down
                    self._throttlePID.SetMode (PID.AUTOMATIC, self.CurrentAirSpeed,
                            self._throttle_control.GetCurrent())
            else:
                if self._flight_mode != SUBMODE_STRAIGHT and self.completed_course():
                    if self._flight_mode == SUBMODE_SWOOP_UP:
                        # Finished a swoop up. Restore full automatic PID operation and limits
                        self._throttlePID.SetMode (PID.AUTOMATIC, self.CurrentAirSpeed,
                                self._throttle_control.GetCurrent())
                    self.get_next_directive()
                    return
                if self._flight_mode == SUBMODE_SWOOP_UP and self.CurrentAirSpeed < self.DesiredAirSpeed:
                    # Going too slow on a swoop up. Have to increase throttle
                    self._throttlePID.SetMode (PID.AUTOMATIC, self.CurrentAirSpeed,
                            self._throttle_control.GetCurrent())

            if self._flight_mode != SUBMODE_COURSE:
                roll = self.compute_roll(self.DesiredTrueHeading)
            else:
                roll = self.compute_roll_from_course()
            if abs(roll) > self.MaxRoll:
                roll = self.MaxRoll if roll > 0 else -self.MaxRoll
            self._desired_roll = roll

        if self._journal_file:
            self._journal_file.write(str(ms))
        if self._throttlePID.GetMode() == PID.AUTOMATIC:
            if self._pid_optimization_goal != "throttle":
                self._throttlePID.SetSetPoint (self.DesiredAirSpeed)
            th = self._throttlePID.Compute (self.CurrentAirSpeed, ms)
        if self._in_pid_optimization == "throttle":
            self._pid_optimization_scoring.IncrementScore(self.CurrentAirSpeed, th, self._pid_optimization_goal, self._journal_file)
        if self._journal_file and self.JournalThrottle:
            self._journal_file.write(",%g,%g,%g"%(self.DesiredAirSpeed, self.CurrentAirSpeed, th))
        self._throttle_control.Set(th)

        self.CurrentAltitude = self._sensors.Altitude()
        self._desired_climb_rate = self.get_climb_rate()

        self.CurrentClimbRate = self._sensors.ClimbRate()
        self._desired_pitch = self.SetPitch(ms)

        if self._journal_file:
            self._journal_file.write("\n")
            self._journal_flush_count += 1
            if self._journal_flush_count >= 10:
                self._journal_file.flush()
                self._journal_flush_count = 0

        self._attitude_control.UpdateControls (self._desired_pitch, self._desired_roll, 0)

    def get_climb_rate(self):
        altitude_error = self.DesiredAltitude - self.CurrentAltitude
        if self.ClimbRateCurve != None:
            ret = util.rate_curve(altitude_error, self.ClimbRateCurve)
            logger.log (5, "Climb rate curve %g ==> %g", altitude_error, ret)
        else:
            climb_rate = altitude_error / self.AltitudeAchievementMinutes
            if self._flight_mode == SUBMODE_SWOOP_DOWN:
                limits = self.SwoopClimbRateLimits
            else:
                limits = self.ClimbRateLimits
            if climb_rate < limits[0]:
                climb_rate = limits[0]
            elif climb_rate > limits[1]:
                climb_rate = limits[1]
            ret = climb_rate
        return ret

    # Compute a good self._desired_heading_rate_change based on the course, speed, heading and position
    def compute_roll_from_course(self):
        pos = self._sensors.Position()
        side, forward, course_heading, heading_to_dest = util.CourseDeviation(pos, self.DesiredCourse)
        turn_radius = (360.0 * (self.DesiredAirSpeed / 60.0) / self.TurnRate) / (4 * util.M_PI)
        turn_radius *= self.InterceptMultiplier
        if abs(side) > turn_radius:
            diff = heading_to_dest - course_heading
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
            if diff > 0:
                intercept_heading = course_heading + 90
            else:
                intercept_heading = course_heading - 90
        else:
            d1 = turn_radius - abs(side)
            # cos(a1) = d1 / turn_radius
            a1 = math.acos (d1 / turn_radius) * util.DEG_RAD
            if side > 0:
                intercept_heading = course_heading - a1
            else:
                intercept_heading = course_heading + a1
            # Example 1: course_heading = 0, side = .1, turn_radius = .5
            #   d1 = .4
            #   a1 = acos (.4 / .5) = 37 degrees
            #   ih = -37
            # Example 2: course_heading = 0, side = 0, turn_radius = .5
            #   d1 = .5
            #   a1 = acos (.5 / .5) = 0 degrees
            #   ih = 0
            # Example 3: course_heading = 0, side = -.1, turn_radius = .5
            #   d1 = .4
            #   a1 = acos (.4 / .5) = 37 degrees
            #   ih = 37
        #util.log_occasional_info("compute_roll_from_course",
        #        "cur_pos = (%g,%g), to=(%g,%g), course_h=%g, h_dest=%g, side = %g, forward=%g, intercept=%g"%(
        #            pos[0], pos[1],
        #            self.DesiredCourse[1][0], self.DesiredCourse[1][1],
        #            course_heading, heading_to_dest, side, forward, intercept_heading), 100)
        return self.compute_roll(intercept_heading)

    def compute_roll(self, intercept_heading):
        heading_error = intercept_heading - self.CurrentTrueHeading
        # Handle wrap around
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        if self._force_turn_direction < 0:
            if heading_error <= 0:
                self._force_turn_direction = 0
            else:
                while heading_error >= 0:
                    heading_error -= 360
        elif self._force_turn_direction > 0:
            if heading_error >= 0:
                self._force_turn_direction = 0
            else:
                while heading_error <= 0:
                    heading_error += 360
        # heading rate change in degrees per second
        ret = util.rate_curve (heading_error, self.RollCurve)
        return ret

    def FollowCourse(self, course, alt):
        self._flight_mode = SUBMODE_COURSE
        self.DesiredCourse = course
        self.DesiredAltitude = alt

    def TurnTo(self, heading, roll_angle = 0, alt = 0):
        logger.debug ("Flight control turning to %g, roll %g, alt %d", heading, roll_angle, alt)
        self._flight_mode = SUBMODE_TURN
        self._force_turn_direction = roll_angle
        self.DesiredTrueHeading = heading
        if alt > 0:
            self.DesiredAltitude = alt
        logger.debug("starting turn at %g", self._sensors.Heading())

    def Turn(self, degrees, roll_angle=0, alt=0):
        logger.debug ("Flight control turning %g, roll %g, alt %d", degrees, roll_angle, alt)
        self._flight_mode = SUBMODE_TURN
        self._force_turn_direction = roll_angle
        if alt > 0:
            self.DesiredAltitude = alt
        turn_start = self._sensors.Heading()
        self.DesiredTrueHeading = turn_start + degrees
        logger.debug("starting turn at %g", turn_start)

    def Swoop(self, course, low_alt, high_alt, min_airspeed=0):
        current_altitude = self._sensors.Altitude()
        if current_altitude <= low_alt:
            self._flight_mode = SUBMODE_COURSE
            self.DesiredAltitude = high_alt
            self.DesiredCourse = course
            self.DesiredAirSpeed = min_airspeed
        else:
            self._flight_mode = SUBMODE_SWOOP_DOWN
            # set altitude PID limits and desired speed higher for a swoop
            self.DesiredAirSpeed = self._callback.MaxAirSpeed
            # use a constant throttle setting to start
            self._throttlePID.SetMode (PID.MANUAL, self.CurrentAirSpeed, self._throttle_control.GetCurrent())
            self._throttle_control.Set (.75)

            self.DesiredCourse = course
            self.DesiredCourse, self._swoop_up_course = self.divide_swoop(course, low_alt,
                                                high_alt, current_altitude)
            self._swoop_high_alt = high_alt
            if min_airspeed > self.MinClimbAirSpeed:
                self._swoop_min_as = min_airspeed
            else:
                self._swoop_min_as = self.MinClimbAirSpeed
            self.DesiredAltitude = low_alt

    def FlyTo(self, coordinate, desired_altitude=0, desired_airspeed=0):
        self.DesiredCourse = [self._sensors.Position(), coordinate]
        if desired_altitude:
            self.DesiredAltitude = desired_altitude
        if desired_airspeed:
            self.DesiredAirSpeed = desired_airspeed
        self._flight_mode = SUBMODE_COURSE

    def FlyCourse(self, course, desired_altitude=0, desired_airspeed=0):
        self.DesiredCourse = course
        if desired_altitude:
            self.DesiredAltitude = desired_altitude
        if desired_airspeed:
            self.DesiredAirSpeed = desired_airspeed
        self._flight_mode = SUBMODE_COURSE

    def StraightAndLevel(self, desired_altitude=0, desired_airspeed=0, desired_heading = None):
        self._flight_mode = SUBMODE_STRAIGHT
        if desired_altitude:
            self.DesiredAltitude = desired_altitude
        else:
            self.DesiredAltitude = self._sensors.Altitude()
        if desired_airspeed:
            self.DesiredAirSpeed = desired_airspeed
        if desired_heading == None:
            self.DesiredTrueHeading = self._sensors.TrueHeading()
        else:
            self.DesiredTrueHeading = desired_heading

    def PIDOptimizationStart(self, which_pid, params, scoring_object, outfile):
        self._in_pid_optimization = which_pid
        self._pid_optimization_scoring = scoring_object
        routing_to = self._in_pid_optimization.split('.')[0]
        if routing_to == "attitude":
            return self._attitude_control.PIDOptimizationStart (which_pid[9:], params, scoring_object, outfile)
        else:
            step = self._pid_optimization_scoring.InitializeScoring()
            self._pid_optimization_goal = step.input_value[2]
            self._journal_file = outfile
            self._journal_flush_count = 0
            self.JournalPitch = False
            self.JournalThrottle = False
            if self._in_pid_optimization == "climb_pitch":
                # TODO: PID tuning unimplemented for controller complex
                self._climbPitchController.SetTunings (params['P'], params['I'], params['D'])
                self._climbPitchController.SetSetPoint (self._pid_optimization_goal)
            elif self._in_pid_optimization == "airspeed_pitch":
                self._airspeedPitchPID.SetTunings (params['P'], params['I'], params['D'])
                self._airspeedPitchPID.SetSetPoint (self._pid_optimization_goal)
            elif self._in_pid_optimization == "throttle":
                self._throttlePID.SetTunings (params['P'], params['I'], params['D'])
                self._throttlePID.SetSetPoint (self._pid_optimization_goal)
            else:
                raise RuntimeError ("Unknown PID optimization target: %s"%which_pid)
            return step

    def PIDOptimizationNext(self):
        routing_to = self._in_pid_optimization.split('.')[0]
        if routing_to == "attitude":
            return self._attitude_control.PIDOptimizationNext()
        else:
            step = self._pid_optimization_scoring.GetNextStep()
            if not step:
                self.PIDOptimizationStop()
                return self._pid_optimization_scoring.GetScore()
            self._pid_optimization_goal = step.input_value
            if self._in_pid_optimization == "climb_pitch":
                self._climbPitchController.SetSetPoint (self._pid_optimization_goal)
            elif self._in_pid_optimization == "airspeed_pitch":
                self._airspeedPitchPID.SetSetPoint (self._pid_optimization_goal)
            elif self._in_pid_optimization == "throttle":
                self._throttlePID.SetSetPoint (self._pid_optimization_goal)
            else:
                raise RuntimeError ("Unknown PID optimization target: %s"%which_pid)
            return step

    def PIDOptimizationStop(self):
        if self._in_pid_optimization == "climb_pitch":
            kp,ki,kd = self.ClimbPitchPIDTuningParams
            self._climbPitchController.SetTunings (kp,ki,kd)
        elif self._in_pid_optimization == "airspeed_pitch":
            kp,ki,kd = self.AirspeedPitchPIDTuningParams
            self._airspeedPitchPID.SetTunings (kp,ki,kd)
        elif self._in_pid_optimization == "throttle":
            kp,ki,kd = self.ThrottlePIDTuningParams
            self._throttlePID.SetTunings (kp,ki,kd)
        else:
            raise RuntimeError ("Unknown PID optimization target: %s"%which_pid)
        self._in_pid_optimization = ""
        if self._journal_file:
            self._journal_file.close()
            self._journal_file = None

    def completed_turn(self):
        diff = self.CurrentTrueHeading - self.DesiredTrueHeading
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return (abs(diff) < 3.0)

    def completed_course(self):
        # The course is considered completed if the aircraft has crossed the line
        # perpendicular to the course and intersecting the terminal waypoint

        # Translate the current position to a coordinate space with the terminal point at origin
        angle,distance,rel_lng = util.TrueHeadingAndDistance(self.DesiredCourse)
        lng,lat = self.CurrentPosition
        tlng,tlat = self.DesiredCourse[1]
        dlng = lng-tlng
        dlat = lat-tlat
        dlng *= rel_lng
        x,y = util.rotate2d (angle * util.M_PI / 180, dlng, dlat)
        #util.log_occasional_info("completed_course",
        #        "cur_pos = (%g,%g), to=(%g,%g), dlng=%g, dlat=%g, angle=%g, xy=%g,%g"%(
        #            self.CurrentPosition[0], self.CurrentPosition[1],
        #            self.DesiredCourse[1][0], self.DesiredCourse[1][1],
        #            dlng, dlat, angle, x, y))

        # turn_circumference = airspeed(nm / minute) / turn_rate (degrees / minute) * 360 degrees
        # turn_diameter = turn_circumference / (2 * PI)
        # turn_radius = turn_diameter / 2

        turn_radius = (360.0 * (self.DesiredAirSpeed / 60.0) / self.TurnRate) / (4 * util.M_PI)
        if y >= -turn_radius / 60.0:
            return True
        else:
            return False

    def get_next_directive(self):
        self._callback.GetNextDirective()

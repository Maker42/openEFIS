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

import time, logging

import PID
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
        self.DesiredHeading = 0
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
        self._turn_start = sensors.Heading()
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
        self._turn_degrees = 0
        self._desired_roll = 0
        self._swoop_low_alt = sensors.Altitude()

        # Configuration properties
        self.AltitudeAchievementMinutes = 1.2
        self.ClimbRateLimits = (-1000.0, 1000.0)        # feet / minute
        self.SwoopClimbRateLimits = (-5000.0, 5000.0)   # feet / minute
        self.PitchPIDLimits = (-20.0, 20.0)         # degrees

        self.PitchPIDSampleTime = 1000
        self.ThrottlePIDSampleTime = 1000

        self.ClimbPitchPIDTuningParams = None
        self.AirspeedPitchPIDTuningParams = None
        self.ThrottlePIDTuningParams = None

        self.HsiFactor = 10.0
        # RollFactor: degrees roll per (degrees heading change / second)
        # 30 degrees roll standard turn is about 180 degrees / minute, or 3 degrees per second
        self.RollFactor = 10.0
        self.MaxRoll = 30.0
        # SecondsHeadingCorrection: How many seconds do we want to consume to achieve a desired heading
        # correction (within the limits of roll)
        self.SecondsHeadingCorrection = 2.0

        self.MaxPitchChangePerSample = 1.0

        # Will Use one of the following 2 PIDS to request pitch angle:
        # Input: Current Climb Rate; SetPoint: Desired Climb Rate; Output: Desired Pitch Attitude
        self._climbPitchPID = None
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
        ms = util.millis()

        kp,ki,kd = self.ClimbPitchPIDTuningParams
        self._climbPitchPID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)
        mn,mx = self.PitchPIDLimits
        self._climbPitchPID.SetOutputLimits (mn, mx)

        kp,ki,kd = self.AirspeedPitchPIDTuningParams
        self._airspeedPitchPID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)
        mn,mx = self.PitchPIDLimits
        self._airspeedPitchPID.SetOutputLimits (mn, mx)

        kp,ki,kd = self.ThrottlePIDTuningParams
        self._throttlePID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)

        self._climbPitchPID.SetSampleTime (self.PitchPIDSampleTime)
        self._airspeedPitchPID.SetSampleTime (self.PitchPIDSampleTime)
        self._throttlePID.SetSampleTime (self.ThrottlePIDSampleTime)

    # Notifies that the take off roll has accompished enough speed that flight controls
    # have responsibility and authority. Activates PIDs.
    def Start(self, last_desired_pitch):
        logger.debug ("Flight Control Starting")
        self._attitude_control.StartFlight()
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
            if self._current_pitch_mode == "climb":
                # Turn off climb pitch PID
                self._climbPitchPID.SetMode (PID.MANUAL, self.CurrentClimbRate, self._desired_pitch)
            # Turn on climb rate Pitch PID
            if self._in_pid_optimization != "airspeed_pitch":
                self._airspeedPitchPID.SetSetPoint (self.MinClimbAirSpeed)
            self._airspeedPitchPID.SetMode (PID.AUTOMATIC, self.CurrentAirSpeed, self._desired_pitch)
            self._current_pitch_mode = "airspeed"

    def SelectClimbratePitch(self):
        if self._current_pitch_mode != "climb":
            if self._current_pitch_mode == "airspeed":
                # Turn off airspeed pitch PID
                self._airspeedPitchPID.SetMode (PID.MANUAL, self.CurrentClimbRate, self._desired_pitch)
            # Turn on climb rate Pitch PID
            if self._in_pid_optimization != "airspeed_pitch":
                self._climbPitchPID.SetSetPoint (self._desired_climb_rate)
            self._climbPitchPID.SetMode (PID.AUTOMATIC, self.CurrentClimbRate, self._desired_pitch)
            self._current_pitch_mode = "climb"

    def UpdateAirspeedPitch(self, ms):
        self._set_pitch_pid_limits(self.PitchPIDLimits, self._airspeedPitchPID)
        desired_pitch = self._airspeedPitchPID.Compute (self.CurrentAirSpeed, ms)
        if self._in_pid_optimization == "airspeed_pitch":
            self._pid_optimization_scoring.IncrementScore(self.CurrentAirSpeed, desired_pitch, self._pid_optimization_goal, self._journal_file)
        if self._journal_file and self.JournalPitch:
            self._journal_file.write(",%g,%g,%g"%(self.MinClimbAirSpeed, self.CurrentAirSpeed, desired_pitch))
        return desired_pitch

    def UpdateClimbratePitch(self, ms):
        self._set_pitch_pid_limits(self.PitchPIDLimits, self._climbPitchPID)
        if self._in_pid_optimization != "climb_pitch":
            self._climbPitchPID.SetSetPoint (self._desired_climb_rate)
        desired_pitch = self._climbPitchPID.Compute (self.CurrentClimbRate, ms)
        if self._in_pid_optimization == "climb_pitch":
            self._pid_optimization_scoring.IncrementScore(self.CurrentClimbRate, desired_pitch, self._pid_optimization_goal, self._journal_file)
        if self._journal_file and self.JournalPitch:
            self._journal_file.write(",%g,%g,%g"%(self._desired_climb_rate, self.CurrentClimbRate, desired_pitch))
        return desired_pitch

    def _set_pitch_pid_limits(self, absolute_limits, pid):
        amn,amx = absolute_limits
        rmn = self._desired_pitch - self.MaxPitchChangePerSample
        rmx = self._desired_pitch + self.MaxPitchChangePerSample
        if rmn < amn:
            rmn = amn
        if rmx > amx:
            rmx = amx
        pid.SetOutputLimits(rmn, rmx)

    def Stop(self):
        self._throttlePID.SetMode (PID.MANUAL, self.CurrentAirSpeed, self._throttle_control.GetCurrent())
        self._climbPitchPID.SetMode (PID.MANUAL, self.CurrentAirSpeed, self._desired_pitch)
        self._airspeedPitchPID.SetMode (PID.MANUAL, self.CurrentClimbRate, self.DesiredAirSpeed)
        self._attitude_control.StopFlight()
        if self._journal_file:
            self._journal_file.close()
        return self._desired_roll, self._desired_climb_rate, self._desired_pitch

    def Update(self):
        ms = util.millis()
        self.CurrentAirSpeed = self._sensors.AirSpeed()
        self.CurrentHeading = self._sensors.Heading()
        if self._flight_mode == SUBMODE_TURN:
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

            if self._flight_mode == SUBMODE_STRAIGHT:
                self._desired_roll = 0.0
            else:
                self.compute_heading_from_course()
                roll = self._desired_heading_rate_change * self.RollFactor
                if abs(roll) > self.MaxRoll:
                    roll = self.MaxRoll if roll > 0 else -self.MaxRoll
                self._desired_roll = roll

        if self._journal_file:
            self._journal_file.write(str(ms))
        if self._throttlePID.GetMode() == PID.AUTOMATIC:
            if self._pid_optimization_goal != "throtle":
                self._throttlePID.SetSetPoint (self.DesiredAirSpeed)
            th = self._throttlePID.Compute (self.CurrentAirSpeed, ms)
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
        climb_rate = altitude_error / self.AltitudeAchievementMinutes
        if self._flight_mode == SUBMODE_SWOOP_DOWN:
            limits = self.SwoopClimbRateLimits
        else:
            limits = self.ClimbRateLimits
        if climb_rate < limits[0]:
            climb_rate = limits[0]
        elif climb_rate > limits[1]:
            climb_rate = limits[1]
        return climb_rate

    # Compute a good self._desired_heading_rate_change based on the course, speed, heading and position
    def compute_heading_from_course(self):
        angle = util.TrueHeading(self.DesiredCourse)
        pos = self._sensors.Position()
        course_heading = util.MagneticHeading(angle, pos)
        heading_to_dest = util.MagneticHeading(util.TrueHeading ((pos, self.DesiredCourse[1]), pos))
        heading_error = (heading_to_dest - course_heading)
        # Handle wrap around
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        # TODO: Limit correction heading as we get close to the destination (sigmoid function?)
        correction_heading = heading_error * self.HsiFactor
        if abs(correction_heading) > 90:
            # TODO: Signal back to flight director of way off course
            correction_heading = 90 if correction_heading > 0 else -90
        intercept_heading = course_heading - correction_heading
        self.compute_heading_rate(intercept_heading)

    def compute_heading_rate(self, intercept_heading):
        heading_error = intercept_heading - self.CurrentHeading
        # Handle wrap around
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        # heading rate change in degrees per second
        self._desired_heading_rate_change = heading_error / self.SecondsHeadingCorrection

    def compute_heading_straight(self):
        self.compute_heading_rate(self.DesiredHeading)

    def FollowCourse(self, course, alt):
        self._flight_mode = SUBMODE_COURSE
        self.DesiredCourse = course
        self.DesiredAltitude = alt

    def Turn(self, degrees, roll_angle, alt):
        logger.debug ("Flight control turning %g, roll %g, alt %d", degrees, roll_angle, alt)
        self._flight_mode = SUBMODE_TURN
        self._turn_degrees = degrees
        self._desired_roll = roll_angle
        self.DesiredAltitude = alt
        self._turn_start = self._sensors.Heading()

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
            self._throttle_control.Set (75)

            self.DesiredCourse = course
            self.DesiredCourse, self._swoop_up_course = self.divide_swoop(course, low_alt,
                                                high_alt, current_altitude)
            self._swoop_high_alt = high_alt
            if min_airspeed > self.MinClimbAirSpeed:
                self._swoop_min_as = min_airspeed
            else:
                self._swoop_min_as = self.MinClimbAirSpeed
            self.DesiredAltitude = low_alt

    def FlyTo(self, coordinate, alt):
        self.DesiredCourse = [self._sensors.Position(), coordinate]
        self.DesiredAltitude = alt
        self._flight_mode = SUBMODE_COURSE

    def StraightAndLevel(self):
        self._flight_mode = SUBMODE_STRAIGHT
        self.DesiredAltitude = self._sensors.Altitude()
        self.DesiredHeading = self._sensors.Heading()

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
                self._climbPitchPID.SetTunings (params['P'], params['I'], params['D'])
                self._climbPitchPID.SetSetPoint (self._pid_optimization_goal)
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
                self._climbPitchPID.SetSetPoint (self._pid_optimization_goal)
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
            self._climbPitchPID.SetTunings (kp,ki,kd)
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
        turn_sign = 1 if self._turn_degrees > 0 else -1
        turn_termination = (self._turn_start + self._turn_degrees) % 360
        diff = self.CurrentHeading - turn_termination
        if turn_sign == 1:
            if diff < self._turn_degrees and diff >= 0:
                return True
        else:
            if diff > self._turn_degrees and diff <= 0:
                return True
        return False

    def completed_course(self):
        # The course is considered completed if the aircraft has crossed the line
        # perpendicular to the course and intersecting the terminal waypoint

        # Translate the current position to a coordinate space with the terminal point at origin
        lng,lat = self.CurrentPosition
        tlng,tlat = self.DesiredCourse[1]
        dlng = lng-tlng
        dlat = lat-tlat
        angle = util.TrueHeading(self.DesiredCourse)
        x,y = util.rotate2d (angle * util.M_PI / 180, dlng, dlat)
        if y >= 0:
            return True
        else:
            return False

    def get_next_directive(self):
        self._callback.GetNextDirective()


def mac(h,f):
    if h != None:
        return h*f
    else:
        return 0
    

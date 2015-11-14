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

import time, logging, math

import PID
import FileConfig
import util

logger=logging.getLogger(__name__)

SUBMODE_STATIONARY="stationary"
SUBMODE_ASCEND="ascend"
SUBMODE_ROTATE="rotate"
SUBMODE_TRANSITION_PRIME="transition_prime"
SUBMODE_TRANSITION_SECONDARY="transition_secondary"

class TakeoffControlVTOL(FileConfig.FileConfig):
    def __init__(self, att_cont, middle_engine_tilt, vtol_engine_release, sensors, callback):
        # Dynamic Input parameters
        self.DesiredAltitude = 0
        self.DesiredAirSpeed = 0
        self.DesiredGroundSpeed = 0
        self.DesiredTrueHeading = 0
        self.DesiredCourse = None
        self.DesiredPosition = None
        self.MagneticDeclination = sensors.MagneticDeclination()
        self.JournalFileName = ''

        # Current State properties
        self.CurrentAltitude = sensors.Altitude()
        self.CurrentAirSpeed = sensors.AirSpeed()
        self.CurrentHeading = sensors.Heading()
        self.CurrentGroundSpeed = sensors.GroundSpeed()

        self._journal_file = None
        self._journal_flush_count = 0
        self._in_pid_optimization = ''
        self._pid_optimization_goal = 0
        self._pid_optimization_scoring = None
        self._lifted_off = False

        # Computed private operating parameters
        self._desired_climb_rate = 0
        self._desired_pitch = 0.0
        self._desired_roll = 0.0
        self._desired_heading_rate_change = 0.0

        self._flight_mode = SUBMODE_STATIONARY
        self._callback = callback

        # Configuration properties
        self.AltitudeAchievementMinutes = .2
        self.ClimbRateLimits = (-100.0, 1000.0)        # feet / minute

        # SecondsHeadingCorrection: How many seconds do we want to consume to achieve a desired heading
        # correction (within the limits of roll)
        self.SecondsHeadingCorrection = 2.0
        self.MaxAttitudeChangePerSample = 1.0
        self.MaxGroundSpeed = 5.0              # Max speed to correct position
        self._PitchPID = None
        self._RollPID = None
        self.AttPIDLimits = (-20.0, 20.0)         # degrees
        self.AttPIDTuningParams = None
        self.AttPIDSampleTime = 1000

        self.TransitionAGL = 700.0
        self.MinutesPositionCorrection = 0.2
        self.TransitionSteps = list()           # List of forward thrust vector angles (0 = up, 90 = forward) and minimum airspeeds
        self.HeadingAchievementSeconds = 2.0
        self.MaxHeadingRate = 15.0

        self._attitude_control = att_cont
        self._sensors = sensors
        self._engine_tilt = middle_engine_tilt
        self._vtol_engine_release = vtol_engine_release
        self._transition_step = 0               # Current transition step number

        FileConfig.FileConfig.__init__(self)

    def initialize(self, filelines):
        self.InitializeFromFileLines(filelines)
        kp,ki,kd = self.AttPIDTuningParams
        self._PitchPID = PID.PID(0, kp, ki, kd, PID.DIRECT, util.millis(self._sensors.Time()))
        self._RollPID = PID.PID(0, kp, ki, kd, PID.DIRECT, util.millis(self._sensors.Time()))
        mn,mx = self.AttPIDLimits
        self._PitchPID.SetOutputLimits (mn, mx)
        self._RollPID.SetOutputLimits (mn, mx)
        self._PitchPID.SetSampleTime (self.AttPIDSampleTime)
        self._RollPID.SetSampleTime (self.AttPIDSampleTime)

    # Notifies that the take off roll has accompished enough speed that flight controls
    # have responsibility and authority. Activates PIDs.
    def Start(self):
        logger.debug ("Takeoff Control Starting")

    def Takeoff(self, desired_course):
        self.DesiredCourse = desired_course
        self.DesiredPosition = self._sensors.Position()

        self._flight_mode = SUBMODE_ASCEND
        self.RunwayAltitude = self._sensors.Altitude()
        self.DesiredAltitude = self.RunwayAltitude + self.TransitionAGL
        self._attitude_control.StartFlight(0.9)
        self._lifted_off = False

    def Stop(self):
        self._PitchPID.SetMode (PID.MANUAL, self.CurrentAirSpeed, self._sensors.AirSpeed())
        self._RollPID.SetMode (PID.MANUAL, self.CurrentAirSpeed, self._sensors.AirSpeed())
        self._attitude_control.StopFlight()
        if self._journal_file:
            self._journal_file.close()
        return self._desired_pitch

    def Update(self):
        ms = util.millis(self._sensors.Time())
        self.CurrentPosition = self._sensors.Position()
        self.CurrentAltitude = self._sensors.Altitude()
        self._desired_climb_rate = self.get_climb_rate()
        self.CurrentHeading = self._sensors.Heading()
        middle_engine_roll_contribution = 1.0
        outer_engine_movement = 0
        if (self._flight_mode == SUBMODE_ASCEND or
                (self._flight_mode == SUBMODE_STATIONARY and
                    self._sensors.Altitude() > self.RunwayAltitude + self.callback.GroundEffectHeight)):
            # If not following a course, keep the starting GPS position
            if self._sensors.ClimbRate() > 0.0 and (not self._lifted_off):
                self._lifted_off = True
                self._PitchPID.SetMode (PID.AUTOMATIC, 0.0 ,0.0)
                self._RollPID.SetMode (PID.AUTOMATIC, 0.0 ,0.0)
            desired_heading,distance,_ = util.TrueHeadingAndDistance ([self.CurrentPosition, self.DesiredPosition])
        
            desired_groundspeed = util.get_rate(0.0, distance * 60.0, self.MinutesPositionCorrection, self.MaxGroundSpeed)
            self.CurrentGroundSpeed = self._sensors.GroundSpeed()
            ground_track = self._sensors.GroundTrack()
            relative_heading = ground_track - self.CurrentHeading
            current_forward_speed = self.CurrentGroundSpeed * math.cos(relative_heading*util.M_PI/180.0)
            current_side_speed = self.CurrentGroundSpeed * math.sin(relative_heading*util.M_PI/180.0)

            current_true_heading = self.CurrentHeading + self.MagneticDeclination
            desired_relative_heading = desired_heading - current_true_heading
            desired_relative_heading *= util.M_PI / 180.0
            desired_forward_speed = desired_groundspeed * math.cos(desired_relative_heading)
            desired_side_speed = desired_groundspeed * math.sin(desired_relative_heading)
            desired_relative_heading /= util.M_PI / 180.0
            logger.log (3, "Ascend relative heading = %g(%g-%g)/%g(%g-%g), forward = %g/%g, side=%g/%g",
                    relative_heading, ground_track, self.CurrentHeading,
                    desired_relative_heading, desired_heading, current_true_heading,
                    current_forward_speed, desired_forward_speed,
                    current_side_speed,desired_side_speed)

            self._PitchPID.SetSetPoint(desired_forward_speed)
            self._RollPID.SetSetPoint(desired_side_speed)
            self._desired_pitch = -self._PitchPID.Compute(current_forward_speed, ms)
            self._desired_roll = self._RollPID.Compute(current_side_speed, ms)

            if self.CurrentAltitude >= self.DesiredAltitude:
                self._desired_roll = 0.0
                self._desired_pitch = 0.0
                self.get_next_directive() # Inserted for xplane testing. Delete me!
                if isinstance(self.DesiredCourse, tuple):
                    self.DesiredTrueHeading = util.TrueHeading (self.DesiredCourse)
                else:
                    self.DesiredTrueHeading = self.DesiredCourse
                hd = self.compute_heading_difference()
                if abs(hd) > 2.0:
                    self._flight_mode = SUBMODE_ROTATE
                    self._rotate_direction = 1 if hd > 0 else -1
                    self.compute_heading_rate(hd)
                else:
                    self._flight_mode = SUBMODE_TRANSITION_PRIME
        elif self._flight_mode == SUBMODE_ROTATE:
            self.compute_heading_rate()
            if self._desired_heading_rate_change * self._rotate_direction <= 0:
                self._desired_heading_rate_change = 0.0
                self._flight_mode = SUBMODE_TRANSITION_PRIME
        elif self._flight_mode == SUBMODE_TRANSITION_PRIME:
            # TODO: slowly move the middle engines into forward thrust, following self.TransitionSteps
            middle_engine_roll_contribution = math.sin(self.TransitionSteps[self._transition_step][0] * util.M_PI / 180.0)
            if self._sensors.AirSpeed() >= self.TransitionSteps[self._transition_step][1]:
                self._transition_step += 1
                if self._transition_step >= len(self.TransitionSteps):
                    self._flight_mode = SUBMODE_TRANSITION_SECONDARY
                    self._vtol_engine_release.Set(1)
                    outer_engine_movement = 1
                else:
                    self._engine_tilt.Set(self.TransitionSteps[self._transition_step][0])
        elif self._flight_mode == SUBMODE_TRANSITION_SECONDARY:
            # Release tilt locks, apply movement slowing brakes, add power to rear engine, and
            # apply downward elevator to compensate for rear engine upward thrust.
            # Adding power to the rear engines will cause the front and back engines to tilt
            # into forward flight mode. The movement brakes will force the transition to be
            # gradual enough for a smooth transition.
            outer_engine_movement = 1
            if self._sensors.OuterEnginePosition() == "forward":
                self._vtol_engine_release.Set(0)
                self.get_next_directive()

        if self._journal_file:
            self._journal_file.write(str(ms))

        if self._journal_file:
            self._journal_file.write("\n")
            self._journal_flush_count += 1
            if self._journal_flush_count >= 10:
                self._journal_file.flush()
                self._journal_flush_count = 0

        logger.log (3, "Ascend desired pitch = %g, roll=%g"%(self._desired_pitch, self._desired_roll))
        self._attitude_control.UpdateControls (self._desired_pitch, self._desired_roll,
                self._desired_heading_rate_change, self._desired_climb_rate, middle_engine_roll_contribution,
                outer_engine_movement, True)

    def get_climb_rate(self):
        return util.get_rate (self.CurrentAltitude, self.DesiredAltitude, self.AltitudeAchievementMinutes, self.ClimbRateLimits)


    def compute_heading_rate(self, heading_difference=None):
        if heading_difference == None:
            heading_difference = self.compute_heading_difference()
        self._desired_heading_rate_change = util.get_rate (0.0, heading_difference,
                self.HeadingAchievementSeconds, self.MaxHeadingRate)

    def compute_heading_difference(self):
        heading_difference = self.DesiredTrueHeading - (self.CurrentHeading + self.MagneticDeclination)
        if heading_difference > 180.0:
            heading_difference -= 360.0
        elif heading_difference < -180.0:
            heading_difference += 360.0
        return heading_difference

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
            self.JournalAtt = False
            self.JournalThrottle = False
            if self._in_pid_optimization == "pitch":
                self._PitchPID.SetTunings (params['P'], params['I'], params['D'])
                self._PitchPID.SetSetPoint (self._pid_optimization_goal)
            if self._in_pid_optimization == "roll":
                self._RollPID.SetTunings (params['P'], params['I'], params['D'])
                self._RollPID.SetSetPoint (self._pid_optimization_goal)
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
            if self._in_pid_optimization == "pitch":
                self._PitchPID.SetSetPoint (self._pid_optimization_goal)
            if self._in_pid_optimization == "roll":
                self._RollPID.SetSetPoint (self._pid_optimization_goal)
            else:
                raise RuntimeError ("Unknown PID optimization target: %s"%which_pid)
            return step

    def PIDOptimizationStop(self):
        if self._in_pid_optimization == "pitch":
            kp,ki,kd = self.AttPIDTuningParams
            self._PitchPID.SetTunings (kp,ki,kd)
        if self._in_pid_optimization == "roll":
            kp,ki,kd = self.AttPIDTuningParams
            self._RollPID.SetTunings (kp,ki,kd)
        else:
            raise RuntimeError ("Unknown PID optimization target: %s"%which_pid)
        self._in_pid_optimization = ""
        if self._journal_file:
            self._journal_file.close()
            self._journal_file = None

    def get_next_directive(self):
        self._callback.GetNextDirective()


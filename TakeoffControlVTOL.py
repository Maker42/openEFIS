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

import PID
import FileConfig, Globals
import util

logger=logging.getLogger(__name__)

SUBMODE_ASCEND="ascend"
SUBMODE_TRANSITION_PRIME="transition_prime"
SUBMODE_TRANSITION_SECONDARY="transition_secondary"

class TakeoffControlVTOL(FileConfig.FileConfig):
    def __init__(self, att_cont, middle_engine_tilt, vtol_engine_release, sensors,
            attitude_vtol_estimation, callback):
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

        # Computed private operating parameters
        self._desired_climb_rate = 0
        self._desired_pitch = 0.0
        self._desired_roll = 0.0
        self._desired_heading_rate_change = 0.0
        self._last_side_speed = 0.0
        self._last_forward_speed = 0.0

        self._flight_mode = SUBMODE_ASCEND
        self._callback = callback

        # Configuration properties
        self.AltitudeAchievementMinutes = .2
        self.ClimbRateLimits = (-100.0, 1000.0)        # feet / minute

        # SecondsHeadingCorrection: How many seconds do we want to consume to achieve a desired heading
        # correction (within the limits of roll)
        self.SecondsHeadingCorrection = 2.0
        self.MaxAttitudeChangePerSample = 1.0

        self.TransitionAGL = 700.0
        self.CorrectionCurve = [(0.0, 0.0), (5.0, 0.2), (10.0, 0.5), (40.0, 1.0)]
        self.TransitionSteps = list()           # List of forward thrust vector angles (0 = up, 90 = forward) and minimum airspeeds
        self.HeadingAchievementSeconds = 2.0
        self.MaxHeadingRate = 15.0
        self.PitchPIDParameters = (.005, .005, 0.0)
        self.PitchPIDSamplePeriod = 1000
        self.PitchPIDLimits = (-0.2, 0.2)

        self.LandAfterReachingState = None      # Name a substate which, after achievement,
                                                # trigger a command to reverse and land.

        self._attitude_control = att_cont
        self._sensors = sensors
        self._engine_tilt = middle_engine_tilt
        self._vtol_engine_release = vtol_engine_release
        self._attitude_vtol_estimation = attitude_vtol_estimation
        self._transition_step = 0               # Current transition step number

        FileConfig.FileConfig.__init__(self)

    def initialize(self, filelines):
        self.InitializeFromFileLines(filelines)
        self._attitude_vtol_estimation.initialize(self.PitchPIDParameters, self.PitchPIDSamplePeriod,
                self.PitchPIDLimits, util.millis(self._sensors.Time()))

    # Notifies that the take off roll has accompished enough speed that flight controls
    # have responsibility and authority. Activates PIDs.
    def Start(self):
        logger.debug ("Takeoff Control Starting")
        self._attitude_vtol_estimation.Start(self.JournalFileName)

    def Takeoff(self, desired_course):
        self.DesiredCourse = desired_course
        self.DesiredPosition = self._sensors.Position()

        self._flight_mode = SUBMODE_ASCEND
        self.RunwayAltitude = self._sensors.Altitude()
        self.DesiredAltitude = self.RunwayAltitude + self.TransitionAGL
        self._transition_step = 0               # Current transition step number
        self._attitude_control.StartFlight(0.9)

    def Stop(self):
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
        throttle_overrides = None
        self._desired_heading_rate_change = 0.0
        use_control_surfaces = False
        # If not following a course, keep the starting GPS position
        self._desired_pitch, self._desired_roll = (
                self._attitude_vtol_estimation.EstimateNextAttitude(self.DesiredPosition,
                    self.CorrectionCurve, self._sensors)
                )

        if self._flight_mode == SUBMODE_ASCEND:
            if self.CurrentAltitude >= self.DesiredAltitude:
                self._desired_roll = 0.0
                self._desired_pitch = 0.0
                if isinstance(self.DesiredCourse, tuple):
                    self.DesiredTrueHeading = util.TrueHeading (self.DesiredCourse)
                else:
                    self.DesiredTrueHeading = self.DesiredCourse
                self._flight_mode = SUBMODE_TRANSITION_PRIME
        elif self._flight_mode == SUBMODE_TRANSITION_PRIME:
            aborted = False
            if self.LandAfterReachingState.startswith (SUBMODE_TRANSITION_PRIME):
                state_length = 0
                if self.LandAfterReachingState.find ('.') > 0:
                    try:
                        state_length = int(self.LandAfterReachingState.rsplit('.', 1)[1])
                    except Exception,e:
                        logger.error ("%s: Unable to read land command substate penetration", str(e))
                if self._transition_step >= state_length:
                    aborted = True
                    self.get_next_directive()

            if not aborted:
                self._desired_pitch = 0.0
                self._desired_roll = 0.0
                if self.TransitionSteps[self._transition_step][0] > 10:
                    # Engines are now facing foward enough that a differential would cause unwanted yaw.
                    # Just thrust forward
                    throttle_overrides = [None, None, 0.7, 0.7, None, None]
                    use_control_surfaces = True
                    self._sensors.FlightMode(Globals.FLIGHT_MODE_AIRBORN, True)
                if self._sensors.AirSpeed() >= self.TransitionSteps[self._transition_step][1]:
                    self._transition_step += 1
                    if self._transition_step >= len(self.TransitionSteps):
                        self._flight_mode = SUBMODE_TRANSITION_SECONDARY
                        self._vtol_engine_release.Set(1)
                        throttle_overrides = [.1, .1, 1.0, 1.0, .3, .3]
                        self._transition_step = self._sensors.Time()
                    else:
                        self._engine_tilt.Set(self.TransitionSteps[self._transition_step][0])
        elif self._flight_mode == SUBMODE_TRANSITION_SECONDARY:
            # Release tilt locks, apply movement slowing brakes, add power to rear engine, and
            # apply downward elevator to compensate for rear engine upward thrust.
            # Adding power to the rear engines will cause the front and back engines to tilt
            # into forward flight mode. The movement brakes will force the transition to be
            # gradual enough for a smooth transition.
            if self.LandAfterReachingState.startswith (SUBMODE_TRANSITION_SECONDARY):
                state_length = 0
                if self.LandAfterReachingState.find ('.') > 0:
                    try:
                        state_length = float(self.LandAfterReachingState.rsplit('.', 1)[1])
                    except Exception,e:
                        logger.error ("%s: Unable to read land command substate penetration", str(e))
                if self._transition_step + state_length < self._sensors.Time():
                    self.get_next_directive()

            use_control_surfaces = True
            throttle_overrides = [.1, .1, 1.0, 1.0, .3, .3]
            self._desired_pitch = 0.0
            self._desired_roll = 0.0
            if self._sensors.OuterEnginePosition() == "forward":
                if self.LandAfterReachingState.startswith (SUBMODE_TRANSITION_SECONDARY):
                    logger.info ("Reached end of secondary transition")
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

        self.compute_heading_rate()
        logger.log (3, "Ascend desired pitch = %g, roll=%g, heading_rate=%g",
                self._desired_pitch, self._desired_roll, self._desired_heading_rate_change)
        self._attitude_control.UpdateControls (self._desired_pitch, self._desired_roll,
                self._desired_heading_rate_change, self._desired_climb_rate, 
                use_control_surfaces, throttle_overrides)

    def get_climb_rate(self):
        return util.get_rate (self.CurrentAltitude, self.DesiredAltitude, self.AltitudeAchievementMinutes, self.ClimbRateLimits)


    def compute_heading_rate(self, heading_difference=None):
        if heading_difference == None:
            heading_difference = self.compute_heading_difference()
        self._desired_heading_rate_change = util.get_rate (0.0, heading_difference,
                self.HeadingAchievementSeconds, self.MaxHeadingRate)

    def compute_heading_difference(self):
        heading_difference = self.DesiredTrueHeading - (self.CurrentHeading - self.MagneticDeclination)
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
            raise RuntimeError ("Unknown PID optimization target: %s"%which_pid)
            return step

    def PIDOptimizationStop(self):
        raise RuntimeError ("Unknown PID optimization target: %s"%which_pid)
        self._in_pid_optimization = ""
        if self._journal_file:
            self._journal_file.close()
            self._journal_file = None

    def get_next_directive(self):
        self._callback.GetNextDirective()

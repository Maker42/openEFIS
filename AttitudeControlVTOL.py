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

import util, FileConfig

logger=logging.getLogger(__name__)

class AttitudeControlVTOL(FileConfig.FileConfig):
    def __init__(self, throttles, yawvtol, elevator, flight_attitude_control, sensors):
        # Dynamic Input properties
        self.DesiredPitch = 0.0
        self.DesiredRoll = 0.0
        self.DesiredHeadingRate = 0.0
        self.DesiredClimbRate = 0.0

        # For recording PID performance
        self.JournalPitch = False
        self.JournalRoll = False
        self.JournalYaw = False
        self.JournalClimb = False
        self.JournalFileName = ''

        # Current State properties
        self.CurrentPitch = 0
        self.CurrentRoll = 0
        self.CurrentHeadingRate = 0
        self.CurrentClimbRate = 0

        self.CurrentRollRate = 0
        self.CurrentPitchRate = 0

        self._journal_file = None
        self._journal_flush_count = 0

        self._in_pid_optimization = ''
        self._pid_optimization_goal = 0
        self._pid_optimization_scoring = None

        self._last_surface_control = False
        self._baseline_offset = 0.0     # Goes negative when the throttle is forced down

        # Configuration properties

        # For each range of airspeeds, there are different PID tuning parameters.
        # That is because as the airspeed gets lower, controls get "mushy", or need more
        # deflection to effect the same response.
        # The AirSpeedCategories is a list of 2-tuples containing the airspeed min and max for that
        # category index. The index of the airspeed is found by walking through the list.
        # The airspeed index is then used to index into the PID tuning parameters to find which
        # set should be used.
        # In order to avoid rapid oscillation between parameter sets, a hysteresis is employed.

        self.PitchPIDSampleTime = 10
        self.RollRatePIDSampleTime = 10
        self.ClimbRatePIDSampleTime = 1000
        self.YawPIDSampleTime = 10

        self.PitchRatePIDTuningParams = None
        self.RollRatePIDTuningParams = None
        self.YawPIDTuningParams = None
        self.ClimbRatePIDTuningParams = None

        self.AttitudePIDLimits = (-.3,.3)
        self.ClimbRatePIDLimits = (0.0,0.8)

        self.AttitudeAchievementSeconds = 1.0
        self.MaxAttitudeRate = 7.0 # degrees per second

        self.NumberEngines = 6
        self.RightEngines = list()
        self.LeftEngines = list()
        self.FrontEngines = list()
        self.RearEngines = list()

        self.ThrottleDownIncrement = .001

        # Working Objects

        # pitch and yaw PIDs have the input of current attitude, set point of desired attitude,
        # and output of desired thrust vector angle
        self._yawPID = None
        # Pitch Rate PID has input of current rate, set point of desired rate,
        #  and output of desired throttle offset
        self._pitchRatePID = None

        # Roll Rate PID has input of current rate, set point of desired rate,
        #  and output of desired throttle offset
        self._rollRatePID = None

        self._climbRatePID = None

        self._throttle_controls = throttles
        self._pitch_throttle_offset = 0.0
        self._roll_throttle_offset = 0.0
        self._throttle_baseline = 0.0
        self._yaw_control = yawvtol
        self._flight_attitude_control = flight_attitude_control
        self._sensors = sensors

        FileConfig.FileConfig.__init__(self, None, None)

    def initialize(self, filelines):
        self.InitializeFromFileLines(filelines)

        kp,ki,kd = self.PitchRatePIDTuningParams
        ms = util.millis(self._sensors.Time())
        self._pitchRatePID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)
        kp,ki,kd = self.RollRatePIDTuningParams
        self._rollRatePID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)

        kp,ki,kd = self.YawPIDTuningParams
        self._yawPID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)


        kp,ki,kd = self.ClimbRatePIDTuningParams
        self._climbRatePID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)

        self._pitchRatePID.SetSampleTime (self.PitchPIDSampleTime)
        self._yawPID.SetSampleTime (self.YawPIDSampleTime)
        self._rollRatePID.SetSampleTime (self.RollRatePIDSampleTime)
        self._climbRatePID.SetSampleTime (self.ClimbRatePIDSampleTime)


    # Notifies that the take off roll has accompished enough speed that flight controls
    # have responsibility and authority. Activates PIDs.
    def StartFlight(self, throttle_baseline):
        self._last_surface_control = False
        mn,mx = self.AttitudePIDLimits
        self._pitchRatePID.SetOutputLimits (mn, mx)
        self._rollRatePID.SetOutputLimits (mn, mx)
        mn,mx = self.ClimbRatePIDLimits
        self._climbRatePID.SetOutputLimits (mn, mx)

        self.CurrentPitchRate = self._sensors.PitchRate()
        self._pitchRatePID.SetSetPoint (util.get_rate(self.CurrentPitchRate, 0.0, self.AttitudeAchievementSeconds, self.MaxAttitudeRate))
        self._pitchRatePID.SetMode (PID.AUTOMATIC, self.CurrentPitchRate, 0.0)

        if self._yaw_control:
            self._yawPID.SetMode (PID.AUTOMATIC, self._sensors.HeadingRateChange(), 0.0)
            mn,mx = self._yaw_control.GetLimits()
            self._yawPID.SetOutputLimits (mn, mx)

        self.CurrentRollRate = self._sensors.RollRate()
        self._rollRatePID.SetSetPoint (util.get_rate(self.CurrentRollRate, 0.0, self.AttitudeAchievementSeconds, self.MaxAttitudeRate))
        self._rollRatePID.SetMode (PID.AUTOMATIC, self.CurrentRollRate, 0.0)

        self._climbRatePID.SetSetPoint (self.DesiredClimbRate)
        self._throttle_baseline = throttle_baseline
        self._baseline_offset = 0.0
        self._climbRatePID.SetMode (PID.AUTOMATIC, self.CurrentClimbRate, self._throttle_baseline)
        self._throttle_controls.SetThrottles([self._throttle_baseline for i in range(6)])

        if self.JournalFileName and (not self._journal_file):
            self._journal_file = open(self.JournalFileName, 'w+')
            if self._journal_file:
                self._journal_file.write("Time")
                if self.JournalRoll:
                    self._journal_file.write(",RollSet,RollCurrent,RollOffset")
                if self.JournalPitch:
                    self._journal_file.write(",PitchSet,PitchCurrent,PitchOffset")
                if self.JournalYaw:
                    self._journal_file.write(",YawSet,YawCurrent,Rudder")
                if self.JournalClimb:
                    self._journal_file.write(",ClimbSet,ClimbCurrent,Throttle")
                self._journal_file.write("\n")

    def StopFlight(self):
        self._pitchRatePID.SetMode (PID.MANUAL, self.CurrentPitchRate, self._pitch_throttle_offset)
        self._rollRatePID.SetMode (PID.MANUAL, self.CurrentRollRate, self._roll_throttle_offset)
        self._climbRatePID.SetMode (PID.MANUAL, self.CurrentClimbRate, self._throttle_baseline)
        if self._yaw_control:
            self._yawPID.SetMode (PID.MANUAL, self._sensors.HeadingRateChange(), 0.0)
        if self._journal_file:
            self._journal_file.close()

    def UpdateControls (self, desired_pitch, desired_roll, desired_heading_rate, desired_climb_rate,
                        use_control_surfaces, throttle_overrides=None):
        self.DesiredPitch = desired_pitch
        self.DesiredRoll = desired_roll
        self.DesiredHeadingRate = desired_heading_rate
        self.DesiredClimbRate = desired_climb_rate

        if use_control_surfaces != self._last_surface_control:
            self._last_surface_control = use_control_surfaces
            if use_control_surfaces:
                self._flight_attitude_control.StartFlight()
            else:
                self._flight_attitude_control.StopFlight()
        if use_control_surfaces:
            self._flight_attitude_control.UpdateControls (self.DesiredPitch, desired_roll, 0.0)

        self.CurrentPitch = self._sensors.Pitch()
        self.CurrentPitchRate = self._sensors.PitchRate()
        self.CurrentRoll = self._sensors.Roll()
        self.CurrentRollRate = self._sensors.RollRate()

        ms = util.millis(self._sensors.Time())
        if self._journal_file:
            self._journal_file.write(str(ms))
        if self._in_pid_optimization != "roll":
            self.DesiredRollRate = util.get_rate(self.CurrentRoll, self.DesiredRoll, self.AttitudeAchievementSeconds, self.MaxAttitudeRate)
            self._rollRatePID.SetSetPoint (self.DesiredRollRate)
        self.CurrentRollRate = self._sensors.RollRate()
        self._roll_throttle_offset = self._rollRatePID.Compute (self.CurrentRollRate, ms)
        if self._in_pid_optimization == "roll":
            self._pid_optimization_scoring.IncrementScore(self.CurrentRollRate, self._roll_throttle_offset, self._pid_optimization_goal, self._journal_file)
        if self._journal_file and self.JournalRoll:
            self._journal_file.write(",%g,%g,%g"%(self.DesiredRollRate, self.CurrentRollRate, self._roll_throttle_offset))

        if self._in_pid_optimization != "pitch":
            self.DesiredPitchRate = util.get_rate(self.CurrentPitch, self.DesiredPitch, self.AttitudeAchievementSeconds, self.MaxAttitudeRate)
            self._pitchRatePID.SetSetPoint (self.DesiredPitchRate)
        self.CurrentPitchRate = self._sensors.PitchRate()
        self._pitch_throttle_offset = self._pitchRatePID.Compute(self.CurrentPitchRate, ms)
        if self._in_pid_optimization == "pitch":
            self._pid_optimization_scoring.IncrementScore(self.CurrentPitchRate, self._pitch_throttle_offset, self._pid_optimization_goal, self._journal_file)
        if self._journal_file and self.JournalPitch:
            self._journal_file.write(",%g,%g,%g"%(self.DesiredPitch, self.CurrentPitchRate, self._pitch_throttle_offset))

        if self._in_pid_optimization != "climb":
            self._climbRatePID.SetSetPoint (self.DesiredClimbRate)
        self.CurrentClimbRate = self._sensors.ClimbRate()
        self._throttle_baseline = self._climbRatePID.Compute (self.CurrentClimbRate, ms)
        if self._in_pid_optimization == "climb":
            self._pid_optimization_scoring.IncrementScore(self.CurrentClimbRate, self._throttle_baseline, self._pid_optimization_goal, self._journal_file)
        if self._journal_file and self.JournalClimb:
            self._journal_file.write(",%g,%g,%g"%(self.DesiredClimbRate, self.CurrentClimbRate, self._throttle_baseline))

        if self._yaw_control:
            self.CurrentHeadingRate = self._sensors.HeadingRateChange()
            if self._in_pid_optimization != "yaw":
                self._yawPID.SetSetPoint (self.DesiredHeadingRate)
            tilt_delta = self._yawPID.Compute(self.CurrentHeadingRate, ms)
            # TODO: Set vectored thrust for yaw
            if self._in_pid_optimization == "yaw":
                self._pid_optimization_scoring.IncrementScore(self.CurrentHeadingRate, tilt_delta, self._pid_optimization_goal, self._journal_file)
            if self._journal_file and self.JournalYaw:
                self._journal_file.write(",%g,%g,%g"%(self.DesiredHeadingRate, self.CurrentHeadingRate, tilt_delta))
            self._yaw_control.Set(tilt_delta)
        if self._journal_file and (not self._in_pid_optimization):
            self._journal_file.write("\n")
            self._journal_flush_count += 1
            if self._journal_flush_count >= 10:
                self._journal_file.flush()
                self._journal_flush_count = 0

        throttle_values = [self._throttle_baseline + self._baseline_offset for i in range(self.NumberEngines)]
        # Add pitch throttle offsets to front and rear engines
        for engine_number in self.FrontEngines:
            throttle_values [engine_number] += self._pitch_throttle_offset
        for engine_number in self.RearEngines:
            throttle_values [engine_number] -= self._pitch_throttle_offset
        # Add roll throttle offsets to right and left engines
        for engine_number in self.RightEngines:
            throttle_values [engine_number] -= self._roll_throttle_offset
        for engine_number in self.LeftEngines:
            throttle_values [engine_number] += self._roll_throttle_offset
        throttle_values = [tv if tv >= 0.0 else 0.0 for tv in throttle_values]
        throttle_values = [tv if tv <= 1.0 else 1.0 for tv in throttle_values]
        if throttle_overrides != None:
            throttle_values = map(lambda ct,ot: ot if ot!=None else ct, throttle_values, throttle_overrides)
        # Set the throttles
        self._throttle_controls.SetThrottles(throttle_values)

    def ForceThrottleDown (self):
        self._baseline_offset -= self.ThrottleDownIncrement
        # Allow engine balancing to continue all the way until all engines are produce no
        # power by forcing a greater degree of attitude PID limits
        self.AttitudePIDLimits = (-1.0,1.0)
        if self._baseline_offset <= -1.0:
            return True
        else:
            return False

    def PIDOptimizationStart(self, which_pid, params, scoring_object, outfile):
        self._in_pid_optimization = which_pid
        self._pid_optimization_scoring = scoring_object
        step = self._pid_optimization_scoring.InitializeScoring()
        self._pid_optimization_goal = step.input_value[2]
        self._journal_file = outfile
        self._journal_flush_count = 0
        self.JournalRoll = False
        self.JournalClimb = False
        self.JournalPitch = False
        self.JournalYaw = False
        if which_pid == "roll":
            self._rollRatePID.SetTunings (params['P'], params['I'], params['D'])
            self._rollRatePID.SetSetPoint (self._pid_optimization_goal)
        elif which_pid == "pitch":
            self._pitchRatePID.SetTunings (params['P'], params['I'], params['D'])
            self._pitchRatePID.SetSetPoint (self._pid_optimization_goal)
        elif which_pid == "yaw":
            self._yawPID.SetTunings (params['P'], params['I'], params['D'])
            self._yawPID.SetSetPoint (self._pid_optimization_goal)
        elif self._in_pid_optimization == "climb":
            self._climbRatePID.SetTunings (params['P'], params['I'], params['D'])
            self._climbRatePID.SetSetPoint (self._pid_optimization_goal)
        else:
            raise RuntimeError("Attitude Control: PID Optimization Target %s not recognized"%which_pid)
        return step

    def PIDOptimizationNext(self):
        step = self._pid_optimization_scoring.GetNextStep()
        if not step:
            self.PIDOptimizationStop()
            self._current_airspeed_index = -1       # Force a restoration of original PID values
            return self._pid_optimization_scoring.GetScore()
        self._pid_optimization_goal = step.input_value
        if self._in_pid_optimization == "roll":
            self._rollRatePID.SetSetPoint (self._pid_optimization_goal)
        elif self._in_pid_optimization == "pitch":
            self._pitchRatePID.SetSetPoint (self._pid_optimization_goal)
        elif self._in_pid_optimization == "yaw":
            self._yawPID.SetSetPoint (self._pid_optimization_goal)
        elif self._in_pid_optimization == "climb":
            self._climbRatePID.SetSetPoint (self._pid_optimization_goal)
        else:
            raise RuntimeError("Attitude Control: PID Optimization Target %s not recognized"%self._in_pid_optimization)
        return step

    def PIDOptimizationStop(self):
        if self._in_pid_optimization == "roll":
            kp,ki,kd = self.RollRatePIDTuningParams
            self._rollRatePID.SetTunings (kp, ki, kd)
        elif self._in_pid_optimization == "pitch":
            kp,ki,kd = self.PitchRatePIDTuningParams
            self._pitchRatePID.SetTunings (kp, ki, kd)
        elif self._in_pid_optimization == "yaw":
            kp,ki,kd = self.YawRatePIDTuningParams
            self._yawRatePID.SetTunings (kp, ki, kd)
        elif self._in_pid_optimization == "climb":
            kp,ki,kd = self.ClimbRatePIDTuningParams
            self._climbRatePID.SetTunings (kp, ki, kd)
        else:
            raise RuntimeError("Attitude Control: PID Optimization Target %s not recognized"%self._in_pid_optimization)
        self._in_pid_optimization = ""
        if self._journal_file:
            self._journal_file.close()
            self._journal_file = None

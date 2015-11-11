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

class AttitudeControl(FileConfig.FileConfig):
    def __init__(self, ac, ec, rc, sensors):
        # Dynamic Input properties
        self.DesiredPitch = 0
        self.DesiredRoll = 0
        self.DesiredYaw = 0

        # For recording PID performance
        self.JournalPitch = False
        self.JournalRoll = False
        self.JournalYaw = False
        self.JournalFileName = ''

        # Current State properties
        self.CurrentPitch = 0
        self.CurrentRoll = 0
        self.CurrentYaw = 0

        self.CurrentRollRate = 0
        self.CurrentYawRate = 0

        self.CurrentAirspeed = 0
        self._current_airspeed_index = 0

        self._journal_file = None
        self._journal_flush_count = 0

        self._in_pid_optimization = ''
        self._pid_optimization_goal = 0
        self._pid_optimization_scoring = None

        # Configuration properties

        # For each range of airspeeds, there are different PID tuning parameters.
        # That is because as the airspeed gets lower, controls get "mushy", or need more
        # deflection to effect the same response.
        # The AirSpeedCategories is a list of 2-tuples containing the airspeed min and max for that
        # category index. The index of the airspeed is found by walking through the list.
        # The airspeed index is then used to index into the PID tuning parameters to find which
        # set should be used.
        # In order to avoid rapid oscillation between parameter sets, a hysteresis is employed.
        self.AirSpeedCategories = list()
        self.HysteresisMargin = .05     # 5% band size for hysteresis

        self.PitchPIDSampleTime = 10
        self.RollRatePIDSampleTime = 10
        self.YawPIDSampleTime = 10

        self.PitchPIDTuningParams = list()
        self.YawPIDTuningParams = list()
        self.RollRatePIDTuningParams = list()

        self.AttitudeAchievementSeconds = 3.0
        self.MaxRollRate = 7.0 # degrees per second

        self.RudderAileronRatio = .2
        self.RollPitchRatio = .025

        # Working Objects

        # pitch and yaw PIDs have the input of current attitude, set point of desired attitude,
        # and output of desired control surface deflection
        self._yawPID = None
        # Pitch Rate PID has input of current rate, set point of desired rate (from the attitude PIDS),
        #  and output of desired control surface deflection
        self._pitchPID = None

        # Roll Rate PID has input of current rate, set point of desired rate (from the attitude PIDS),
        #  and output of desired control surface deflection
        self._rollRatePID = None

        self._aileron_control = ac
        self._elevator_control = ec
        self._rudder_control = rc
        self._sensors = sensors

        FileConfig.FileConfig.__init__(self, None, None)

    def initialize(self, filelines):
        self.InitializeFromFileLines(filelines)

        self._current_airspeed_index = self.get_airspeed_index()
        kp,ki,kd = self.GetTunings (self.PitchPIDTuningParams)
        ms = util.millis(self._sensors.Time())
        self._pitchPID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)

        kp,ki,kd = self.GetTunings (self.YawPIDTuningParams)
        self._yawPID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)

        kp,ki,kd = self.GetTunings (self.RollRatePIDTuningParams)
        self._rollRatePID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)

        self._pitchPID.SetSampleTime (self.PitchPIDSampleTime)
        self._yawPID.SetSampleTime (self.YawPIDSampleTime)
        self._rollRatePID.SetSampleTime (self.RollRatePIDSampleTime)


    # Notifies that the take off roll has accompished enough speed that flight controls
    # have responsibility and authority. Activates PIDs.
    def StartFlight(self):
        mn,mx = self._elevator_control.GetLimits()
        self._pitchPID.SetOutputLimits (mn, mx)
        mn,mx = self._aileron_control.GetLimits()
        self._rollRatePID.SetOutputLimits (mn, mx)

        self.CurrentPitch = self._sensors.Pitch()
        self._pitchPID.SetSetPoint (self.CurrentPitch)
        self._pitchPID.SetMode (PID.AUTOMATIC, self.CurrentPitch, self._elevator_control.GetCurrent())

        self.CurrentYaw = self._sensors.Yaw()
        self._yawPID.SetSetPoint (self.DesiredYaw)
        self._yawPID.SetMode (PID.AUTOMATIC, self.CurrentYaw, self._rudder_control.GetCurrent())
        mn,mx = self._rudder_control.GetLimits()
        self._yawPID.SetOutputLimits (mn, mx)

        self.CurrentRollRate = self._sensors.RollRate()
        self._rollRatePID.SetSetPoint (self.CurrentRollRate)
        self._rollRatePID.SetMode (PID.AUTOMATIC, self.CurrentRollRate, self._aileron_control.GetCurrent())

        if self.JournalFileName and (not self._journal_file):
            self._journal_file = open(self.JournalFileName, 'w+')
            if self._journal_file:
                self._journal_file.write("Time")
                if self.JournalRoll:
                    self._journal_file.write(",RollSet,RollCurrent,Aileron")
                if self.JournalPitch:
                    self._journal_file.write(",PitchSet,PitchCurrent,Elevator")
                if self.JournalYaw:
                    self._journal_file.write(",YawSet,YawCurrent,Rudder")
                self._journal_file.write("\n")

    def StopFlight(self):
        self._pitchPID.SetMode (PID.MANUAL, self.CurrentPitch, self._elevator_control.GetCurrent())
        self._rollRatePID.SetMode (PID.MANUAL, self.CurrentRollRate, self._aileron_control.GetCurrent())
        self._yawPID.SetMode (PID.MANUAL, self.CurrentYaw, self._rudder_control.GetCurrent())
        if self._journal_file:
            self._journal_file.close()

    def UpdateControls (self, desired_pitch, desired_roll, desired_yaw):
        self.DesiredPitch = desired_pitch
        self.DesiredRoll = desired_roll
        self.DesiredYaw = desired_yaw
        asidx = self.get_new_airspeed_index()
        if asidx != self._current_airspeed_index:
            # Update the tuning parameters for the new airspeed category
            self._current_airspeed_index = asidx

            if self._in_pid_optimization != "yaw":
                kp,ki,kd = self.GetTunings (self.YawPIDTuningParams)
                self._yawPID.SetTunings (kp, ki, kd)

            if self._in_pid_optimization != "roll":
                kp,ki,kd = self.GetTunings (self.RollRatePIDTuningParams)
                self._rollRatePID.SetTunings (kp, ki, kd)

            if self._in_pid_optimization != "pitch":
                kp,ki,kd = self.GetTunings (self.PitchPIDTuningParams)
                self._pitchPID.SetTunings (kp, ki, kd)

        ms = util.millis(self._sensors.Time())
        if self._journal_file:
            self._journal_file.write(str(ms))
        if self._in_pid_optimization != "roll":
            desired_roll_rate = self.get_roll_rate()
            self._rollRatePID.SetSetPoint (desired_roll_rate)
        self.CurrentRollRate = self._sensors.RollRate()
        self.aileron_deflection = self._rollRatePID.Compute (self.CurrentRollRate, ms)
        self._aileron_control.Set (self.aileron_deflection)
        if self._in_pid_optimization == "roll":
            self._pid_optimization_scoring.IncrementScore(self.CurrentRollRate, self.aileron_deflection, self._pid_optimization_goal, self._journal_file)
        if self._journal_file and self.JournalRoll:
            self._journal_file.write(",%g,%g,%g"%(desired_roll_rate, self.CurrentRollRate, self.aileron_deflection))

        if self._in_pid_optimization != "pitch":
            desired_pitch = self.DesiredPitch + self.CurrentRoll * self.RollPitchRatio
            self._pitchPID.SetSetPoint (desired_pitch)
        self.CurrentPitch = self._sensors.Pitch()
        elevator_value = self._pitchPID.Compute(self.CurrentPitch, ms)
        self._elevator_control.Set (elevator_value)
        if self._in_pid_optimization == "pitch":
            self._pid_optimization_scoring.IncrementScore(self.CurrentPitch, elevator_value, self._pid_optimization_goal, self._journal_file)
        if self._journal_file and self.JournalPitch:
            self._journal_file.write(",%g,%g,%g"%(desired_pitch, self.CurrentPitch, elevator_value))

        self.CurrentYaw = self._sensors.Yaw()
        if self._in_pid_optimization != "yaw":
            self._yawPID.SetSetPoint (self.DesiredYaw)
        rudder_value = self._yawPID.Compute(self.CurrentYaw, ms)
        self._rudder_control.Set (rudder_value + self.rudder_offset())
        if self._in_pid_optimization == "yaw":
            self._pid_optimization_scoring.IncrementScore(self.CurrentYaw, rudder_value, self._pid_optimization_goal, self._journal_file)
        if self._journal_file and self.JournalRoll:
            self._journal_file.write(",%g,%g,%g"%(self.DesiredYaw, self.CurrentYaw, rudder_value))
        if self._journal_file and (not self._in_pid_optimization):
            self._journal_file.write("\n")
            self._journal_flush_count += 1
            if self._journal_flush_count >= 10:
                self._journal_file.flush()
                self._journal_flush_count = 0

    def get_roll_rate(self):
        self.CurrentRoll = self._sensors.Roll()
        return util.get_rate(self.CurrentRoll, self.DesiredRoll, self.AttitudeAchievementSeconds, self.MaxRollRate)

    # Returns how much the rudders should be offset to approximate coordinated flight,
    #  given the current aileron deflection
    def rudder_offset(self):
        # TODO: this is probably a non-linear function in reality
        return self.RudderAileronRatio * self.aileron_deflection

    def GetTunings (self, params):
        try:
            return params [self._current_airspeed_index]
        except Exception, e:
            raise RuntimeError ("GetTunings[%d] got error %s"%(self._current_airspeed_index, str(e)))

    def get_new_airspeed_index (self):
        assert(len(self.AirSpeedCategories) == len(self.RollRatePIDTuningParams))
        # First see if we just stay with the current index
        if self._current_airspeed_index < 0:
            return self.get_airspeed_index()
        mn,mx = self.AirSpeedCategories[self._current_airspeed_index]
        mn -= (mn * self.HysteresisMargin)
        mx += (mx * self.HysteresisMargin)
        self.CurrentAirspeed = self._sensors.AirSpeed()
        if self.CurrentAirspeed >= mn and self.CurrentAirspeed <= mx:
            # No change
            return self._current_airspeed_index

        for i in range(len(self.AirSpeedCategories)):
            mn,mx = self.AirSpeedCategories[i]
            if self.CurrentAirspeed >= mn and self.CurrentAirspeed <= mx:
                return i
        else:
            if self.CurrentAirspeed < self.AirSpeedCategories[0][0]:
                return 0
        return len(self.AirSpeedCategories)-1

    def get_airspeed_index(self):
        self.CurrentAirspeed = self._sensors.AirSpeed()
        for i in range(len(self.AirSpeedCategories)):
            mn,mx = self.AirSpeedCategories[i]
            if self.CurrentAirspeed >= mn and self.CurrentAirspeed <= mx:
                return i
        else:
            if self.CurrentAirspeed < self.AirSpeedCategories[0][0]:
                return 0
        return len(self.AirSpeedCategories)-1

    def PIDOptimizationStart(self, which_pid, params, scoring_object, outfile):
        self._in_pid_optimization = which_pid
        self._pid_optimization_scoring = scoring_object
        step = self._pid_optimization_scoring.InitializeScoring()
        self._pid_optimization_goal = step.input_value[2]
        self._journal_file = outfile
        self._journal_flush_count = 0
        self.JournalRoll = False
        self.JournalPitch = False
        self.JournalYaw = False
        if which_pid == "roll":
            self._rollRatePID.SetTunings (params['P'], params['I'], params['D'])
            self._rollRatePID.SetSetPoint (self._pid_optimization_goal)
        elif which_pid == "pitch":
            self._pitchPID.SetTunings (params['P'], params['I'], params['D'])
            self._pitchPID.SetSetPoint (self._pid_optimization_goal)
        elif which_pid == "yaw":
            self._yawPID.SetTunings (params['P'], params['I'], params['D'])
            self._yawPID.SetSetPoint (self._pid_optimization_goal)
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
            self._pitchPID.SetSetPoint (self._pid_optimization_goal)
        elif self._in_pid_optimization == "yaw":
            self._yawPID.SetSetPoint (self._pid_optimization_goal)
        else:
            raise RuntimeError("Attitude Control: PID Optimization Target %s not recognized"%which_pid)
        return step

    def PIDOptimizationStop(self):
        self._in_pid_optimization = ""
        if self._journal_file:
            self._journal_file.close()
            self._journal_file = None

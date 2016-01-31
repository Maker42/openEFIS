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
import FileConfig
import util

logger=logging.getLogger(__name__)

SUBMODE_ACCELERATE="accelerate"
SUBMODE_ROTATE="rotate"
SUBMODE_POSITIVE_CLIMB="positive_climb"
SUBMODE_GEAR_UP="gear_up"
SUBMODE_FLAPS_UP="flaps_up"

class TakeoffControl(FileConfig.FileConfig):
    def __init__(self, att_cont, throttle_control, rudder_control, gear_control, flap_control, sensors, callback):
        # Dynamic Input parameters
        self.DesiredCourse = [(0,0), (0,0)]
        self.DesiredTrueHeading = 0
        self.JournalPitch = False
        self.JournalFileName = ''

        # Current State properties
        self.CurrentAirSpeed = sensors.AirSpeed()
        self.CurrentClimbRate = sensors.ClimbRate()
        self.CurrentTrueHeading = sensors.TrueHeading()
        self.CurrentHeadingRate = sensors.HeadingRateChange()

        self._journal_file = None
        self._journal_flush_count = 0
        self._in_pid_optimization = ''
        self._pid_optimization_goal = 0
        self._pid_optimization_scoring = None

        # Computed private operating parameters
        self._desired_pitch = 0
        self._last_pitch = 0
        self._desired_heading_rate = 0

        self._flight_mode = SUBMODE_ACCELERATE
        self._callback = callback

        # Configuration properties
        self.MaxPitchChangePerSample = 1.0
        self.PitchPIDSampleTime = 1000
        self.RudderPIDSampleTime = 10
        self.RudderPIDTuningParams = None
        self.HeadingRateCurve = [(0, 0), (5.0, 0.1), (10.0, 0.3)]
        self.TakeoffPitch = 15.0
        self.InitialRudder = .4
        self.TakeoffFlaps = 0.2

        # Operational properties
        self._attitude_control = att_cont
        self._sensors = sensors
        self._throttle_control = throttle_control
        self._rudder_control = rudder_control
        self._gear_control = gear_control
        self._flap_control = flap_control
        self._RudderPID = None

        FileConfig.FileConfig.__init__(self)

    def initialize(self, filelines):
        self.InitializeFromFileLines(filelines)
        ms = util.millis(self._sensors.Time())
        self._last_update_time = ms
        kp,ki,kd = self.RudderPIDTuningParams
        self._RudderPID = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)
        self._RudderPID.SetSampleTime (self.RudderPIDSampleTime)


    # Notifies that the take off roll has accompished enough speed that flight controls
    # have responsibility and authority. Activates PIDs.
    def Start(self):
        logger.debug ("Takeoff Control Starting")

        mn,mx = self._rudder_control.GetLimits()
        self._RudderPID.SetOutputLimits (mn, mx)
        self.CurrentHeadingRate = self._sensors.HeadingRateChange()
        self.CurrentTrueHeading = self._sensors.TrueHeading()
        self._RudderPID.SetSetPoint (self.ComputeHeadingRate())
        if self._flap_control:
            self._flap_control.Set(self.TakeoffFlaps)

        if self.JournalFileName and (not self._journal_file):
            self._journal_file = open(self.JournalFileName, 'w+')
            if self._journal_file:
                self._journal_file.write("Time")
                self._journal_file.write(",AirSpeed,Pitch")
                self._journal_file.write("\n")

    def Takeoff(self, approach_endpoints, soft_field):
        self._flight_mode = SUBMODE_ACCELERATE
        self._abort = False
        if soft_field:
            self._desired_pitch = self.TakeoffPitch
            self._last_pitch = self._sensors.Pitch()
            self._attitude_control.StartFlight()
        else:
            self._desired_pitch = None
            self._last_pitch = None
        self._throttle_control.Set(1.0)
        logger.debug ("Throttle up")
        self.DesiredCourse = approach_endpoints
        self.DesiredTrueHeading,_,__ = util.TrueHeadingAndDistance(self.DesiredCourse)
        self.CurrentHeadingRate = self._sensors.HeadingRateChange()
        self._RudderPID.SetMode (PID.AUTOMATIC, self.CurrentHeadingRate, self.InitialRudder)

    def Stop(self):
        self._attitude_control.StopFlight()
        self._RudderPID.SetMode (PID.MANUAL, self.CurrentHeadingRate, self._rudder_control.GetCurrent())
        if self._journal_file:
            self._journal_file.close()
        return self._desired_pitch

    def Update(self):
        ms = util.millis(self._sensors.Time())
        self.CurrentAirSpeed = self._sensors.AirSpeed()
        self.CurrentTrueHeading = self._sensors.TrueHeading()
        self.CurrentClimbRate = self._sensors.ClimbRate()
        self.CurrentHeadingRate = self._sensors.HeadingRateChange()

        if self._flight_mode == SUBMODE_ACCELERATE:
            self.DesiredHeadingRate = self.ComputeHeadingRate()
            self._RudderPID.SetSetPoint(self.DesiredHeadingRate)
            rudder = self._RudderPID.Compute(self.CurrentHeadingRate, ms)
            #logger.debug ("Takeoff Set Rudder %g/%g ==> %g", self.CurrentHeadingRate, self.DesiredHeadingRate, rudder)
            self._rudder_control.Set(rudder)
            if self._sensors.EnginesOut() > 0:
                self._abort = True
                self._throttle_control.Set(0)
                self._brake_control.Set(.75)
            if (not self._abort) and self.CurrentAirSpeed >= self._callback.StallSpeed * 1.2:
                self._flight_mode = SUBMODE_ROTATE
                self._attitude_control.StartFlight()
                self._desired_pitch = self.TakeoffPitch
                logger.debug ("Takeoff rotating to pitch %g", self._desired_pitch)
        elif self._flight_mode == SUBMODE_ROTATE:
            if abs(self._sensors.Pitch() - self._desired_pitch) < 2.0:
                self._RudderPID.SetMode (PID.MANUAL, self.CurrentHeadingRate, self._rudder_control.GetCurrent())
                self._flight_mode = SUBMODE_POSITIVE_CLIMB
                logger.debug ("Takeoff looking for positive climb")
        elif self._flight_mode == SUBMODE_POSITIVE_CLIMB:
            if self.CurrentClimbRate >= 100.0 and self._sensors.AGL() > 100.0:
                if self._gear_control != None:
                    self._gear_control.Set(0)
                    self._flight_mode = SUBMODE_GEAR_UP
                else:
                    if self._flap_control != None:
                        self._flap_control.Set(0)
                        self._flight_mode = SUBMODE_FLAPS_UP
                    else:
                        self.get_next_directive()
        elif self._flight_mode == SUBMODE_GEAR_UP:
            if self._sensors.GearUpLocked():
                self._flight_mode = SUBMODE_FLAPS_UP
                self._flap_control.Set(0)
                if self._flap_control != None:
                    self._flap_control.Set(0)
                    self._flight_mode = SUBMODE_FLAPS_UP
                else:
                    self.get_next_directive()
        elif self._flight_mode == SUBMODE_FLAPS_UP:
            if self._sensors.FlapPosition() == 0:
                self.get_next_directive()

        if self._journal_file:
            self._journal_file.write("\n")
            self._journal_flush_count += 1
            if self._journal_flush_count >= 10:
                self._journal_file.flush()
                self._journal_flush_count = 0

        if self._desired_pitch != None:
            if self._last_pitch == None:
                self._last_pitch = self._sensors.Pitch()
            pitch_diff = self._desired_pitch - self._last_pitch
            time_diff = ms - self._last_update_time
            if abs(pitch_diff) > self.MaxPitchChangePerSample * time_diff / self.PitchPIDSampleTime:
                pitch_diff = self.MaxPitchChangePerSample * time_diff / self.PitchPIDSampleTime
                if self._desired_pitch < self._last_pitch:
                    pitch_diff *= -1
                self._last_pitch += pitch_diff
            desired_yaw = None if self._flight_mode == SUBMODE_ACCELERATE else 0
            self._attitude_control.UpdateControls (self._last_pitch, 0.0, desired_yaw)
        self._last_update_time = ms

    def ComputeHeadingRate(self):
        heading_error = self.DesiredTrueHeading - self.CurrentTrueHeading
        if heading_error < -300:
            heading_error += 360
        elif heading_error > 300:
            heading_error -= 360
        ret = util.rate_curve(abs(heading_error), self.HeadingRateCurve)
        ret = ret if heading_error >= 0 else -ret
        #logger.debug ("Desired Heading Rate %g/%g==>%g", self.CurrentTrueHeading, self.DesiredTrueHeading, ret)
        return ret

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

    def get_next_directive(self):
        self._callback.GetNextDirective()

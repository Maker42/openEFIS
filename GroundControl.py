# Copyright (C) 2015-2018  Garrett Herschleb
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

from Common.FileConfig import FileConfig
import Common.util as util

import PID

class GroundControl(FileConfig):
    def __init__(self, throttle_control, sensors, callback):
        self._callback = callback
        self._sensors = sensors
        self.DesiredHeading = 0
        self.DesiredGroundSpeed = 0

        self.CurrentHeading = sensors.Heading()
        self.CurrentGroundSpeed = sensors.GroundSpeed()

        self.ThrottlePIDLimits = (0, 100)
        self.RudderPIDLimits = (-20, 20)

        self.ThrottlePIDSampleTime = 1000
        self.RudderPIDSampleTime = 100

        self.ThrottlePIDTuningParams = None
        self.RudderPIDTuningParams = None

        self._yawPID = None
        self._throttlePID = None

        self.rudder_control = None
        self.throttle_control = throttle_control

        self._current_airspeed = 0
        self._delta_airspeed = 0

    def initialize(self, filelines):
        self.InitalizeFromFileLines(filelines)

        kp,ki,kd = self.GetTunings (self.RudderPIDTuningParams)
        ms = util.millis(self._sensors.Time())
        self._yawPID = PID.PID(kp, ki, kd, PID.DIRECT, ms)
        mn,mx = self.RudderPIDLimits
        self._yawPID.SetOutputLimits (mn, mx)

        self.aileron_control = eval(self.aileron_control)
        self.elevator_control = eval(self.elevator_control)
        self.rudder_control = eval(self.rudder_control)

    # Notifies that the landing roll is complete, or the system has just awoken on the ground, and
    # have responsibility and authority. Activates PIDs.
    def Start(self, lo_yaw):
        self._yawPID.SetSetPoint (self.DesiredHeading)
        self._yawPID.SetMode (PID.AUTOMATIC, self.CurrentHeading, self.rudder_control.GetCurrent())

    def Stop(self):
        self._yawPID.SetMode (PID.MANUAL, self.CurrentHeading, self.rudder_control.GetCurrent())

    def Taxi(self, desired_course):
        pass

    def UpdateControls (self, desired_yaw):
        self.DesiredYaw = desired_yaw
        ms = util.millis(self._sensors.Time())

        self._yawPID.SetSetPoint (self.DesiredYaw)
        self.rudder_control.Set (self._yawPID.Compute(self.CurrentYaw, ms))

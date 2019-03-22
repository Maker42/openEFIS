# Copyright (C) 2018-2019  Garrett Herschleb
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

import math

from Common.util import DEG_RAD, rate_curve
from MicroServerComs import MicroServerComs

class GroundRoll(MicroServerComs):
    def __init__(self, accelerometer_calibration):
        self.accelerometer_calibration = accelerometer_calibration
        MicroServerComs.__init__(self, "GroundRoll")
        self.xoffset = 0

    def updated(self, channel):
        if channel == 'accelerometers':
            if self.a_z != 0:
                if isinstance(self.accelerometer_calibration, dict):
                    self.a_x = rate_curve (self.a_x,
                            self.accelerometer_calibration['x'], False)
                    self.a_z = rate_curve (self.a_z,
                            self.accelerometer_calibration['z'], False)
                self.a_x += self.xoffset
                self.ground_roll = math.atan2(-self.a_x, self.a_z) * DEG_RAD
                self.timestamp = self.accelerometers_updated
                self.publish ()
                print ("GroundRoll: %.2f,%.2f => %.1f"%(self.a_z, self.a_x, self.ground_roll))
        elif channel == 'systemcommand' and hasattr(self, 'a_x'):
            if self.command.startswith( b'0att'):
                print ("Attitude: Received command to 0 out")
                self.xoffset = -self.a_x


if __name__ == "__main__":
    gr = GroundRoll()
    gr.listen()

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

import Common.util as util
from MicroServerComs import MicroServerComs

class PitchEstimate(MicroServerComs):
    def __init__(self, accelerometer_calibration):
        self.accelerometer_calibration = accelerometer_calibration
        MicroServerComs.__init__(self, "PitchEstimate")
        self.yoffset = 0
        self.roll = None

    def updated(self, channel):
        if channel == 'accelerometers':
            if self.a_z != 0:
                if isinstance(self.accelerometer_calibration, dict):
                    self.a_y = util.rate_curve (self.a_y,
                            self.accelerometer_calibration['y'], False)
                    self.a_z = util.rate_curve (self.a_z,
                            self.accelerometer_calibration['z'], False)
                self.a_y += self.yoffset
                roll = 0
                if self.roll is not None and \
                        self.Roll_updated - self.accelerometers_updated < .4:
                    roll = self.roll * util.RAD_DEG
                self.pitch_estimate = math.atan(self.a_y /
                        (self.a_z*math.cos(roll)))
                self.pitch_estimate *= util.DEG_RAD
                self.timestamp = self.accelerometers_updated
                self.publish ()
                print ("PitchEstimate: roll(%.1f) %.2f,%.2f,%.2f => %.2f"%(roll, 
                    self.a_x, self.a_y, self.a_z, self.pitch_estimate))
        elif channel == 'systemcommand' and hasattr(self, 'a_y'):
            if self.command.startswith( b'0att'):
                self.yoffset = -self.a_y

if __name__ == "__main__":
    pe = PitchEstimate()
    pe.listen()

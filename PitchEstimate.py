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

    def updated(self, channel):
        if channel == 'accelerometers':
            if self.a_z != 0:
                if isinstance(self.accelerometer_calibration, dict):
                    self.a_y = util.rate_curve (self.a_y,
                            self.accelerometer_calibration['y'])
                    self.a_z = util.rate_curve (self.a_z,
                            self.accelerometer_calibration['z'])
                self.a_y += self.yoffset
                self.pitch_estimate = math.atan(float(self.a_y) / float(self.a_z)) * \
                        util.DEG_RAD
                self.timestamp = self.accelerometers_updated
                self.publish ()
                print ("PitchEstimate: %d,%d => %g"%(self.a_z, self.a_y, self.pitch_estimate))
        elif channel == 'systemcommand':
            if self.command.startswith( b'0att'):
                self.yoffset = -self.a_y

if __name__ == "__main__":
    pe = PitchEstimate()
    pe.listen()

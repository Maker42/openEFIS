# Copyright (C) 2018  Garrett Herschleb
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

import Common.util as util
from Common.Spatial import Vector

from MicroServerComs import MicroServerComs

class HeadingComputed(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "HeadingComputed")

    def updated(self, channel):
        mag_vector = Vector(self.m_x, self.m_z, self.m_y)
        mag_polar = mag_vector.to_polar(limit_phi = True, robot_coordinates=False)
        mag_heading = self._calibrated_heading (mag_polar.theta * util.DEG_RAD)
        self.heading_computed = int(round(mag_heading))
        self.timestamp = self.magneticsensors_updated

        self.publish ()
        print ("HeadingComputed: %g,%g,%g => %g"%(self.m_z, self.m_y, self.m_x, self.heading_computed))

    def _calibrated_heading(self, heading):
        # Find the bracketing throttle tables
        # In each table, estimate the calibrated heading
        # estimate calibrated heading PWL from 2 throttle brackets
        # for test:
        return heading
        raise RuntimeError("Magnetic calibration unimplemented")

if __name__ == "__main__":
    hc = HeadingComputed()
    hc.listen()

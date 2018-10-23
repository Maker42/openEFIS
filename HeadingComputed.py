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

from Common.util import DEG_RAD, rate_curve
import math

from MicroServerComs import MicroServerComs

class HeadingComputed(MicroServerComs):
    def __init__(self, heading_calibration):
        self.heading_calibration = heading_calibration
        MicroServerComs.__init__(self, "HeadingComputed")

    def updated(self, channel):
        theta = math.atan2 (self.m_y, self.m_x) - math.pi/2
        mag_heading = theta * DEG_RAD
        while mag_heading < 0:
            mag_heading += 360
        while mag_heading >= 360:
            mag_heading -= 360
        self.heading_computed = self._calibrated_heading (mag_heading)
        self.timestamp = self.magneticsensors_updated

        self.publish ()
        print ("HeadingComputed: %g,%g,%g => %g"%(self.m_z, self.m_y, self.m_x, self.heading_computed))

    def _calibrated_heading(self, heading):
        # In each table, estimate the calibrated heading
        # estimate calibrated heading through a piece wise linear function
        if isinstance(self.heading_calibration, dict):
            heading = rate_curve (heading, self.heading_calibration['compass_correction'])
        return heading

if __name__ == "__main__":
    hc = HeadingComputed()
    hc.listen()

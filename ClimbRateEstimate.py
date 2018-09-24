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
from MicroServerComs import MicroServerComs

class ClimbRateEstimate(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "ClimbRateEstimate")
        self.last_altitude = None
        self.last_time = None
        self.climb_rate_estimate = None
        self._raw_alt_rate = list()

    def updated(self, channel):
        if self.last_time is not None:
            timediff = self.gps_utc - self.last_time
            timediff /= 60.0        # Convert rate to per minute
            altdiff = self.gps_altitude - self.last_altitude
            current_alt_rate = altdiff / timediff
            self.climb_rate_estimate = util.LowPassFilter (current_alt_rate, self._raw_alt_rate)
            self.climb_rate_estimate = int(round(self.climb_rate_estimate))
            self.publish ()
            print ("ClimbRateEstimate: %d/%g => %g"%(altdiff, timediff, self.climb_rate_estimate))
        self.last_time = self.gps_utc
        self.last_altitude = self.gps_altitude


if __name__ == "__main__":
    cre = ClimbRateEstimate()
    cre.listen()

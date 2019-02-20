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

class ClimbRate(MicroServerComs):
    def __init__(self, conf_mult=0.01):
        MicroServerComs.__init__(self, "ClimbRate")
        self.last_time = None
        self.last_altitude = None
        self.climb_rate = None
        self.climb_rate_estimate = None
        self._raw_alt_rate = list()
        self.climb_rate_confidence = 0.0
        # Default 1 degree per minute
        self.confidence_multiplier = conf_mult

    def updated(self, channel):
        if channel == 'AltitudeComputed':
            self.timestamp = self.AltitudeComputed_updated
            if self.last_time is not None:
                timediff = self.timestamp - self.last_time
                timediff /= 60.0        # Convert rate to per minute
                altdiff = self.altitude_computed - self.last_altitude
                current_alt_rate = altdiff / timediff
                self.climb_rate = util.LowPassFilter (current_alt_rate, self._raw_alt_rate)
                if self.climb_rate_estimate is not None:
                    variance = abs(self.climb_rate - self.climb_rate_estimate)
                    self.climb_rate_confidence = 10.0 - variance * self.confidence_multiplier
                else:
                    self.climb_rate_confidence = 9.0
                self.publish ()
                print ("ClimbRate: %d/%d => %g(%g)"%(self.altitude_computed, altdiff,
                    self.climb_rate, self.climb_rate_confidence))
            self.last_altitude = self.altitude_computed
            self.last_time = self.timestamp


if __name__ == "__main__":
    climb_rate = ClimbRate()
    climb_rate.listen()

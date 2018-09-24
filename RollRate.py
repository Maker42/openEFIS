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

import math

import Globals
import Common.util as util
from MicroServerComs import MicroServerComs

class RollRate(MicroServerComs):
    def __init__(self, cor_rate=1.0, conf_mult=1.0):
        MicroServerComs.__init__(self, "RollRate")
        self.roll_rate = None
        self.roll_rate_estimate = None
        self.flight_mode = Globals.FLIGHT_MODE_GROUND
        self.roll_rate_confidence = 0.0
        # Default 1 degree per minute
        self.confidence_multiplier = conf_mult

    def updated(self, channel):
        if channel == 'rotationsensors':
            self.roll_rate = self.r_y
            if self.roll_rate_estimate is not None and \
                            self.flight_mode != Globals.FLIGHT_MODE_GROUND:
                variance = abs(self.roll_rate - self.roll_rate_estimate)
                self.roll_rate_confidence = 10.0 - variance * self.confidence_multiplier
            else:
                self.roll_rate = self.r_y
                self.roll_rate_confidence = 10.0
            self.timestamp = self.rotationsensors_updated
            self.publish ()
            print ("roll_rate: %g => %g(%g)"%(self.r_y, self.roll_rate, self.roll_rate_confidence))


if __name__ == "__main__":
    rr = RollRate()
    rr.listen()

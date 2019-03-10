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

from MicroServerComs import MicroServerComs
from Common.util import DEG_RAD, RAD_DEG

class RollEstimate(MicroServerComs):
    def __init__(self):
        # tcf default: 5 degree roll will make 1 degree / s heading change
        MicroServerComs.__init__(self, "RollEstimate")
        self.estimated_tas = None
        self.airspeed_computed = None
        self.cas2tas = None

    def updated(self, channel):
        if channel == 'TurnRateComputed':
            if self.airspeed_computed is not None and self.cas2tas is not None:
                tas = self.airspeed_computed * self.cas2tas
            elif self.estimated_tas is not None:
                tas = self.estimated_tas
            else:
                print ("RollEstimate: No TAS available")
                return      # No tas, no roll estimate
            self.timestamp = self.TurnRateComputed_updated
            #math.tan(_bank_angle_) = (rate_of_turn * tas_knots * 493/900) / 9.8(m/s)
            rate_of_turn = self.turn_rate_computed * RAD_DEG
            bank_radians = math.atan((rate_of_turn * tas * 493/900) / 9.8) #(m/s)
            self.roll_estimate = bank_radians * DEG_RAD
            self.publish ()
            print ("RollEstimate: %.1f => %.1f"%(self.turn_rate_computed, self.roll_estimate))


if __name__ == "__main__":
    re = RollEstimate()
    re.listen()

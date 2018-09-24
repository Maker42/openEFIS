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

import Globals
from MicroServerComs import MicroServerComs

class Roll(MicroServerComs):
    def __init__(self, cor_rate=1.0, conf_mult=1.0):
        MicroServerComs.__init__(self, "Roll")
        self.last_time = None
        self.roll = None
        self.ground_roll = None
        self.roll_estimate = None
        self.flight_mode = Globals.FLIGHT_MODE_GROUND
        self.roll_confidence = 0.0
        # Default 1 degree per minute
        self.correction_rate = cor_rate / 60.0
        self.confidence_multiplier = conf_mult

    def updated(self, channel):
        if channel == 'rotationsensors':
            if self.last_time and self.flight_mode != Globals.FLIGHT_MODE_GROUND:
                timediff = self.rotationsensors_updated - self.last_time
                correction_factor = self.correction_rate * timediff
                if self.roll_estimate is not None:
                    self.roll += (self.r_y * timediff -
                                (self.roll_estimate - self.roll) * correction_factor)
                    variance = abs(self.roll - self.roll_estimate)
                    self.roll_confidence = 10.0 - variance * self.confidence_multiplier
                else:
                    self.roll += (self.r_y * timediff)
                    self.roll_confidence = 5.0
                self.timestamp = self.rotationsensors_updated
                self.publish ()
                print ("roll: %g => %g(%g)"%(self.r_y, self.roll, self.roll_confidence))
            self.last_time = self.rotationsensors_updated
        elif channel == 'GroundRoll':
            if self.flight_mode == Globals.FLIGHT_MODE_GROUND:
                self.roll = self.ground_roll
                self.roll_confidence = 10.0 - self.roll * 0.5
                self.timestamp = self.GroundRoll_updated
                self.publish ()
                print ("roll: (from estimate) %g => %g(%g)"%(self.ground_roll, self.roll, self.roll_confidence))


if __name__ == "__main__":
    roll = Roll()
    roll.listen()

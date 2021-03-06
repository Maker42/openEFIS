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

class RollRateEstimate(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "RollRateEstimate")
        self._roll = list()
        self.last_roll = None
        self.last_time = None

    def updated(self, channel):
        self.timestamp = self.RollEstimate_updated
        if self.last_roll is not None:
            timediff = self.timestamp - self.last_time
            current_roll_rate = (self.roll_estimate - self.last_roll) / timediff
            self.roll_rate_estimate = util.LowPassFilter (current_roll_rate, self._roll)
            self.publish ()
            print ("RollRateEstimate: %.1f => %.2f"%(self.roll_estimate, self.roll_rate_estimate))
        self.last_time = self.timestamp
        self.last_roll = self.roll_estimate

if __name__ == "__main__":
    rre = RollRateEstimate()
    rre.listen()

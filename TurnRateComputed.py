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


from MicroServerComs import MicroServerComs

class TurnRateComputed(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "TurnRateComputed")
        self._raw_heading_rate = list()
        self.last_heading = None
        self.last_time = None
        self.filter_coefficient = .2
        self.turn_rate_computed = 0.0

    def updated(self, channel):
        self.timestamp = self.HeadingComputed_updated
        if self.last_heading is not None:
            timediff = self.timestamp - self.last_time
            diff = (self.heading_computed - self.last_heading)
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
            current_heading_rate = diff / timediff
            self.turn_rate_computed += self.filter_coefficient * (current_heading_rate - self.turn_rate_computed)
            self.publish ()
            print ("TurnRateComputed: %g => %g"%(self.heading_computed, self.turn_rate_computed))
        self.last_time = self.timestamp
        self.last_heading = self.heading_computed

if __name__ == "__main__":
    pe = TurnRateComputed()
    pe.listen()

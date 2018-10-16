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


from MicroServerComs import MicroServerComs

class RollEstimate(MicroServerComs):
    def __init__(self, tcf=5.0):
        # tcf default: 5 degree roll will make 1 degree / s heading change
        MicroServerComs.__init__(self, "RollEstimate")
        self.TurnCoordinatorFactor = tcf

    def updated(self, channel):
        self.timestamp = self.TurnRateComputed_updated
        self.roll_estimate = self.turn_rate_computed * self.TurnCoordinatorFactor
        self.publish ()
        print ("RollEstimate: %g => %g"%(self.turn_rate_computed, self.roll_estimate))


if __name__ == "__main__":
    re = RollEstimate()
    re.listen()

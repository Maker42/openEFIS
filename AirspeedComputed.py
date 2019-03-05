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

import yaml

import Common.util as util

from MicroServerComs import MicroServerComs

class AirspeedComputed(MicroServerComs):
    def __init__(self, airspeed_config):
        MicroServerComs.__init__(self, "AirspeedComputed")
        self.pitot = None
        self.airspeed_computed = None
        self.ascurve = None
        if isinstance(airspeed_config,dict):
            self.ascurve = airspeed_config['airspeed_pressure_curve']

    def updated(self, channel):
        if self.ascurve is not None:
            if self.pitot > 0:
                # Only output if we have 2 valid pressures to compare
                self.timestamp = self.pitotsensor_updated
                self.airspeed_computed = util.rate_curve (self.pitot, self.ascurve)
                self.airspeed_computed = self.airspeed_computed
                self.publish ()
                print ("AirspeedComputed: %.2f ==> %.2f"%(self.pitot, self.airspeed_computed))
            else:
                print ("AirspeedComputed: no pitot")
        else:
            print ("AirspeedComputed: don't have curve")

if __name__ == "__main__":
    ae = AirspeedComputed()
    ae.listen()

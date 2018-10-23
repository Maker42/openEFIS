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

KELVIN_OFFSET=273.15
PRESSURE_DIVISOR=.0065
PRESSURE_POWER=5.25588

class AltitudeComputed(MicroServerComs):
    def __init__(self, pressure_calibration):
        MicroServerComs.__init__(self, "AltitudeComputed")
        self.altitude_computed = None
        self.static_pressure = None
        self.sea_level_pressure = None
        self.temperature = None
        self.pressure_calibration = pressure_calibration
        # TODO: publish confidence level based on recency and relevance of inputs

    def updated(self, channel):
        if channel == 'pressuresensors':
            self.timestamp = self.pressuresensors_updated
            if self.sea_level_pressure is not None and self.temperature is not None:
                if isinstance(self.pressure_calibration,dict):
                    self.static_pressure = util.rate_curve (self.static_pressure,
                            self.pressure_calibration['pressure_calibration'])
                self.altitude_computed = ((pow(
                    self.sea_level_pressure / self.static_pressure, 1/PRESSURE_POWER)-1) *
                        (self.temperature + KELVIN_OFFSET)) / PRESSURE_DIVISOR
                self.altitude_computed *= util.FEET_METER
                self.altitude_computed = self.altitude_computed
                self.publish ()
                print ("AltitudeComputed: %d"%self.altitude_computed)
            else:
                if self.temperature is None:
                    print ("AltitudeComputed: No temp data")
                if self.sea_level_pressure is None:
                    print ("AltitudeComputed: No sea level pressure data")

if __name__ == "__main__":
    ac = AltitudeComputed()
    ac.listen()

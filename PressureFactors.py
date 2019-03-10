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

import Common.util as util

from MicroServerComs import MicroServerComs

KPA_INHG = 100.0 / 29.53
KELVIN_OFFSET=273.15
PRESSURE_DIVISOR=.0065
PRESSURE_POWER=5.25588

class PressureFactors(MicroServerComs):
    def __init__(self, pressure_calibration):
        MicroServerComs.__init__(self, "PressureFactors")
        self.known_altitude = None
        self.static_pressure = None
        self.sea_level_pressure = None
        self.temperature = None
        self.static_pressure = None
        self.standard_sea_level_temp = None
        self.pressure_calibration = pressure_calibration
        # TODO: publish confidence level based on recency and relevance of inputs

    def updated(self, channel):
        if channel == 'knownaltitude':
            if self.temperature is not None:
                self.standard_sea_level_temp = self.temperature + 1.98 * self.known_altitude / 1000.0
        elif channel == 'systemcommand':
            print ("Got systemcommand: %s"%str(self.command))
            if self.command.startswith(b'baroinhg'):
                print ("Got new baro setting: %.2f"%float(self.args.strip(b'\x00')))
                self.sea_level_pressure = (float(self.args.strip(b'\x00')) * KPA_INHG)
            elif self.command.startswith( b'barompa'):
                self.sea_level_pressure = (float(self.args.strip(b'\x00')) * 1000)

        if self.static_pressure is not None:
            if isinstance(self.pressure_calibration,dict):
                self.static_pressure = util.rate_curve (self.static_pressure,
                        self.pressure_calibration['pressure_calibration'])

        if self.known_altitude is not None and self.static_pressure is not None and \
                self.temperature is not None:
            self.sea_level_pressure = (self.static_pressure /
                pow(1.0 - (self.known_altitude * util.METERS_FOOT * PRESSURE_DIVISOR /
                    (self.temperature + KELVIN_OFFSET)), PRESSURE_POWER))
            self.known_altitude = None

        if self.sea_level_pressure is not None and self.static_pressure is not None and \
                self.temperature is not None:
            if self.standard_sea_level_temp is None:
                altitude_computed = ((pow(self.sea_level_pressure / self.static_pressure, 1/PRESSURE_POWER)-1) *
                        (self.temperature + KELVIN_OFFSET)) / PRESSURE_DIVISOR
                altitude_computed *= util.FEET_METER
                self.standard_sea_level_temp = self.temperature + 1.98 * altitude_computed / 1000.0

            self.cas2tas = math.sqrt(
                self.AirDensity (self.sea_level_pressure, self.standard_sea_level_temp) /
                self.AirDensity(self.static_pressure, self.temperature)
               )
            self.publish ()
            print ("PressureFactors: %g, %g"%(self.sea_level_pressure, self.cas2tas))

    def AirDensity(self, pressure, temp):
        temp += KELVIN_OFFSET
        pressure *= 1000.0       # 1 kPa = 1000 Pa
        return pressure / (temp * 287.058)

if __name__ == "__main__":
    ac = PressureFactors()
    ac.listen()

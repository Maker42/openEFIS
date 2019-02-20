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

class Altitude(MicroServerComs):
    def __init__(self, conf_mult=0.01):
        MicroServerComs.__init__(self, "Altitude")
        self.altitude = None
        self.gps_altitude = None
        self.altitude_confidence = 0.0
        # Default 1 degree per minute
        self.confidence_multiplier = conf_mult

    def updated(self, channel):
        if channel == 'AltitudeComputed':
            self.timestamp = self.AltitudeComputed_updated
            self.altitude = self.altitude_computed
            if self.gps_altitude is not None:
                variance = abs(self.altitude - self.gps_altitude)
                self.altitude_confidence = 10.0 - variance * self.confidence_multiplier
            else:
                self.altitude_confidence = 9.0
            self.publish ()
            print ("Altitude: %g => %g(%g)"%(self.altitude_computed, self.altitude, self.altitude_confidence))


if __name__ == "__main__":
    altitude = Altitude()
    altitude.listen()

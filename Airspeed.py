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

class Airspeed(MicroServerComs):
    def __init__(self, conf_mult=0.05):
        MicroServerComs.__init__(self, "Airspeed")
        self.airspeed = None
        self.airspeed_estimate = None
        self.airspeed_computed = None
        self.airspeed_is_estimated = True
        self.airspeed_confidence = 0.0
        # Default 1 degree per minute
        self.confidence_multiplier = conf_mult

    def updated(self, channel):
        update = False
        if channel == 'AirspeedComputed':
            self.timestamp = self.AirspeedComputed_updated
            self.airspeed = self.airspeed_computed
            self.airspeed_is_estimated = False
            if self.airspeed_estimate is not None:
                variance = abs(self.airspeed - self.airspeed_estimate)
                self.airspeed_confidence = 10.0 - variance * self.confidence_multiplier
            else:
                self.airspeed_confidence = 9.0
            update = True
        elif channel == 'AirspeedEstimate':
            if self.airspeed_computed is None:
                self.timestamp = self.AirspeedEstimate_updated
                self.airspeed = self.airspeed_estimate
                self.airspeed_is_estimated = True
                update = True
        if update:
            self.publish ()
            print ("Airspeed: %.1f(%.1f)"%(self.airspeed, self.airspeed_confidence))


if __name__ == "__main__":
    airspeed = Airspeed()
    airspeed.listen()

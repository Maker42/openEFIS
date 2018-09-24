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

class Heading(MicroServerComs):
    def __init__(self, conf_mult=0.1):
        MicroServerComs.__init__(self, "Heading")
        self.heading = None
        self.heading_estimate = None
        self.magnetic_variation = None
        self.flight_mode = Globals.FLIGHT_MODE_GROUND
        self.heading_confidence = 0.0
        self.gps_magnetic_variation = None
        # Default 1 degree per minute
        self.confidence_multiplier = conf_mult

    def updated(self, channel):
        if channel == 'HeadingComputed':
            self.timestamp = self.HeadingComputed_updated
            self.heading = self.heading_computed
            if self.heading_estimate is not None and self.flight_mode != Globals.FLIGHT_MODE_GROUND:
                variance = abs(self.heading - self.heading_estimate)
                self.heading_confidence = 10.0 - variance * self.confidence_multiplier
            else:
                self.heading_confidence = 9.0
            if self.gps_magnetic_variation is not None:
                self.publish ()
                print ("Heading: %g(%g)"%(self.heading, self.heading_confidence))


if __name__ == "__main__":
    heading = Heading()
    heading.listen()

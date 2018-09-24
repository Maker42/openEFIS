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

class AirspeedEstimate(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "AirspeedEstimate")
        self.estimated_tas = None
        self.airspeed_estimate = None
        self.cas2tas = None
        # TODO: publish confidence level based on recency and relevance of inputs

    def updated(self, channel):
        if channel == 'HeadingTasEstimate':
            if self.estimated_tas is not None and \
                            self.cas2tas is not None:

                self.airspeed_estimate = self.estimated_tas / self.cas2tas
                self.airspeed_estimate = int(round(self.airspeed_estimate))
                self.publish ()

                print ("AirspeedEstimate: %d"%self.airspeed_estimate)

if __name__ == "__main__":
    ae = AirspeedEstimate()
    ae.listen()

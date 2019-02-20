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
from Common.Spatial import Polar

from MicroServerComs import MicroServerComs

class HeadingTasEstimate(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "HeadingTasEstimate")
        self.wind_heading = 0
        self.wind_speed = 0
        self.gps_ground_track = None
        self.gps_ground_speed = None
        self.estimated_tas = None
        self.estimated_heading_true = None
        self.gps_magnetic_variation = None
        # TODO: publish confidence level based on recency and relevance of inputs

    def updated(self, channel):
        update = False
        if channel == 'gpsfeed':
            update = True
        elif channel == 'WindEstimate':
            # only update and publish from the GPS feed.
            # If you updated from a new wind estimate, you would cause a
            # recursive feedback cascade since wind estimates come from
            # finalized readings
            pass

        if update and self.gps_ground_track is not None and \
                self.wind_speed is not None:
            ground_polar = Polar (self.gps_ground_speed, self.gps_ground_track * util.RAD_DEG, 0)
            wind_polar = Polar (self.wind_speed, self.wind_heading * util.RAD_DEG, 0)
            ground_vector = ground_polar.to3(robot_coordinates=False)
            wind_vector = wind_polar.to3(robot_coordinates=False)
            ground_vector.sub(wind_vector)       # ground vector now holds wind vector
            air_polar = Polar()
            air_polar.from3 (ground_vector, limit_phi=True, robot_coordinates=False)
            self.estimated_heading_true = int(round(air_polar.theta * util.RAD_DEG))
            self.estimated_tas = int(round(air_polar.rad))
            while self.estimated_heading_true < 0:
                self.estimated_heading_true += 360
            self.publish ()

            print ("HeadingTasEstimate: %d at %d degrees"%(
                self.estimated_tas, self.estimated_heading_true))

if __name__ == "__main__":
    hte = HeadingTasEstimate()
    hte.listen()

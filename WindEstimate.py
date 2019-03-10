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


import Common.util as util
from Common.Spatial import Polar

from MicroServerComs import MicroServerComs

class WindEstimate(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "WindEstimate")
        self.airspeed_is_estimated = False
        self.winds_aloft_reports = dict()
        self.gps_ground_track = None
        self.gps_ground_speed = None
        self.airspeed = None
        self.true_airspeed = None
        self.cas2tas = None
        self.heading = None
        self.wind_heading = 0
        self.wind_speed = 0
        # TODO: publish confidence level based on recency and relevance of inputs

    def updated(self, channel):
        update = False
        if channel == 'Airspeed':
            if self.cas2tas is not None:
                self.true_airspeed = self.airspeed * self.cas2tas
                update = not self.airspeed_is_estimated
        elif channel == 'PressureFactors':
            if self.airspeed is not None:
                self.true_airspeed = self.airspeed * self.cas2tas
        elif channel == 'systemcommand':
            if self.command == b'atis':
                args = self.args.strip(b'\x00').split()
                self.wind_speed = float(args[0])
                self.wind_heading = float(args[1])
                print ("WindEstimate from atis: %.1f at %.1f degrees"%(self.wind_speed, self.wind_heading))
                self.publish()
            elif self.command == b'windsalft':
                args = self.args.strip(b'\x00').split()
                args = [float(a) for a in args]
                self.wa_lat, self.wa_lng, self.wa_altitude, self.wa_time, \
                            self.wa_heading, self.wa_speed = args
                position=(round(self.wa_lat,2),round(self.wa_lng,2))
                if position in self.winds_aloft_reports:
                    if self.wa_altitude in self.winds_aloft_reports[position]:
                        self.winds_aloft_reports[position][self.wa_altitude][self.wa_time] = ( \
                                self.wa_heading,self.wa_speed)
                        # TODO: trim out old reports
                    else:
                        self.winds_aloft_reports[position][self.wa_altitude] = {
                                self.wa_time: (self.wa_heading,self.wa_speed)}
                else:
                    self.winds_aloft_reports[position] = {
                            self.wa_altitude: {self.wa_time: (self.wa_heading,self.wa_speed)}}
                if self.airspeed_is_estimated:
                    # TODO: Create wind estimate from reports
                    self.wind_heading = self.wa_heading
                    self.wind_speed = self.wa_speed
                    self.publish()  # Publish the last reported for now

        if update and self.gps_ground_track is not None and \
                self.gps_ground_speed is not None and \
                self.true_airspeed is not None and \
                self.heading is not None and \
                self.magnetic_declination is not None:
            ground_polar = Polar (self.gps_ground_speed, self.gps_ground_track * util.RAD_DEG, 0)
            air_polar = Polar (self.true_airspeed,
                    (self.heading+self.magnetic_declination) * util.RAD_DEG, 0)
            ground_vector = ground_polar.to3(robot_coordinates=False)
            air_vector = air_polar.to3(robot_coordinates=False)
            ground_vector.sub(air_vector)       # ground vector now holds wind vector
            wind_polar = Polar()
            wind_polar.from3 (ground_vector, limit_phi=True, robot_coordinates=False)
            self.wind_heading = wind_polar.theta * util.RAD_DEG
            self.wind_speed = wind_polar.rad
            while self.wind_heading < 0:
                self.wind_heading += 360
            self.publish ()
            print ("WindEstimate: %.1f at %.1f degrees"%(self.wind_speed, self.wind_heading))
        else:
            # TODO: estimate wind from aloft reports
            pass


if __name__ == "__main__":
    we = WindEstimate()
    we.listen()

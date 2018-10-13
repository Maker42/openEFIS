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

import time

import yaml

from MicroServerComs import MicroServerComs

param_map = {
 "Roll" : ("ROLL", "roll")
,"Pitch" : ("PITCH", "pitch")
,"Heading" : ("HEAD", "heading")
,"Altitude" : ("ALT", "altitude")
,"ClimbRate" : ("VS", "climb_rate")
,"Airspeed" : ("IAS", "airspeed")
,"Yaw" : ("YAW", "yaw")
}

class RAIS(MicroServerComs):
    def __init__(self, config_file=None):
        if config_file is not None:
            with open(config_file, 'r') as yml:
                cfg = yaml.load(yml)
                yml.close()
        else:
            cfg = None
        MicroServerComs.__init__(self, "Display", config=cfg)
        self.altitude = None
        self.airspeed = None
        self.heading = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.climb_rate = None
        self.turn_rate = None
        self.gps_utc = None
        self.gps_lat = None
        self.gps_lng = None
        self.gps_ground_speed = None
        self.gps_ground_track = None
        self.gps_signal_quality = None
        self.gps_magnetic_variation = None
        self.update_period = 1.0/30.0
        self.callback = None

    def setParameterCallback(self, cb):
        self.callback = cb

    def updated(self, channel):
        if self.callback is not None:
            if channel in param_map:
                dbkey,property_string = param_map[channel]
                self.callback (dbkey, getattr(self, property_string))
            elif channel == "GroundVector":     # Handle odd case of GPS output
                self.callback ("GS", self.gps_ground_speed)
                self.callback ("TRACK", self.gps_ground_track)
                self.callback ("LAT", self.gps_lat)
                self.callback ("LONG", self.gps_lng)
                self.callback ("TIMEZ", time.asctime(time.gmtime(self.gps_utc)))
                self.callback ("TRACKM", self.gps_ground_track + self.gps_magnetic_variation)

    def AreSensorsGreen(self):
        ret = not (self.altitude is None or 
                self.airspeed is None or 
                self.heading is None or 
                self.roll is None or 
                self.pitch is None or 
                self.yaw is None or 
                self.turn_rate is None or 
                self.climb_rate is None)
        if not ret:
            print ("-")
        return ret

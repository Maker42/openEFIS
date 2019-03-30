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

import time

import yaml

from MicroServerComs import MicroServerComs
from PubSub import assign_all_ports
from SenseControl import SystemCommand

# TODO: Need to transition between flight modes

param_map = {
 "Roll" : ("ROLL", "roll", "roll_confidence")
,"Pitch" : ("PITCH", "pitch", "pitch_confidence")
,"Heading" : ("HEAD", "heading", "heading_confidence")
,"Altitude" : ("ALT", "altitude", "altitude_confidence")
,"ClimbRate" : ("VS", "climb_rate", "climb_rate_confidence")
,"Airspeed" : [("IAS", "airspeed", "airspeed_confidence"), ("TAS", "tas", "airspeed_confidence")]
,"Yaw" : ("ALAT", "yaw", "yaw_confidence")
,"TurnRate" : ("ROT", "turn_rate", "turn_rate_confidence")
}

class RAIS(MicroServerComs):
    def __init__(self, starting_port, config_file=None):
        if config_file is not None:
            with open(config_file, 'r') as yml:
                cfg = yaml.load(yml, Loader=yaml.SafeLoader)
                yml.close()
                assign_all_ports (cfg, starting_port)
        else:
            cfg = None
        self.pubsub_config = cfg
        MicroServerComs.__init__(self, "Display", config=cfg, timeout=0)
        self._system_command = SystemCommand(cfg)
        self.altitude = None
        self.airspeed = None
        self.tas = None
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
        self.ground_vector_confidence = None
        self.magnetic_declination = None
        self.update_period = 1.0/30.0
        self.callback = None

    def setParameterCallback(self, cb):
        self.callback = cb

    def updated(self, channel):
        if self.callback is not None:
            if channel in param_map:
                params = param_map[channel]
                if isinstance(params, tuple):
                    dbkey,property_string,conf = params
                    self.callback (dbkey, getattr(self, property_string),
                            getattr(self, conf))
                elif isinstance(params, list):
                    for dbkey,property_string,conf in params:
                        self.callback (dbkey, getattr(self, property_string),
                                getattr(self, conf))
            elif channel == "GroundVector":     # Handle odd case of GPS output
                self.callback ("GS", self.gps_ground_speed,
                        self.ground_vector_confidence)
                self.callback ("TRACK", self.gps_ground_track,
                        self.ground_vector_confidence)
                self.callback ("LAT", self.gps_lat, self.ground_vector_confidence)
                self.callback ("LONG", self.gps_lng, self.ground_vector_confidence)
                self.callback ("TIMEZ", time.asctime(time.gmtime(self.gps_utc)),
                        self.ground_vector_confidence)
                if self.magnetic_declination is not None:
                    self.callback ("TRACKM",
                           self.gps_ground_track - self.magnetic_declination,
                           self.ground_vector_confidence)

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

    def GivenBarometer(self, b):
        self._system_command.send ('baroinhg', str(b))

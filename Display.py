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

import sys
import socket
import struct

from Common.util import METERS_FOOT

from MicroServerComs import MicroServerComs

class Display(MicroServerComs):
    def __init__(self, localportno, xplane_host, xplane_port=49000):
        MicroServerComs.__init__(self, "Display")
        self.localport = localportno
        self.xplane_host = xplane_host
        self.xplane_port = xplane_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind (("", localportno))
        self.sock.setblocking (0)
        self.gps_magnetic_variation = None
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
        self.dvis_struct_preamble = struct.pack("5s", b"DVIS")
        self.dvis_struct_body = struct.Struct("dddddd")
        self.dref_struct_preamble = struct.pack("5s", b"DREF")
        self.dref_struct_body = struct.Struct("f500s")
        self.additional_data = {
                 "airspeed":    b"sim/cockpit2/gauges/indicators/airspeed_kts_pilot"
                ,"climb_rate":   b"sim/cockpit2/gauges/indicators/vvi_fpm_pilot"
                #,"Compass":     b"sim/cockpit2/gauges/indicators/compass_heading_deg_mag"
                ,"yaw":         b"sim/cockpit2/gauges/indicators/slip_deg"
                ,"altitude":    b"sim/cockpit2/gauges/indicators/altitude_ft_pilot"
                ,"turn_rate":    b"sim/cockpit2/gauges/indicators/turn_rate_heading_deg_pilot"
                ,"heading":     b"sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot"
                ,"pitch":       b"sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot"
                ,"roll":        b"sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot"
                }

    def updated(self, channel):
        if channel == 'Roll' and self.AreSensorsGreen():
            # Send new visual data to Xplane
            dvis = self.dvis_struct_preamble + \
                    self.dvis_struct_body.pack(
                            self.gps_lat,
                            self.gps_lng,
                            self.altitude * METERS_FOOT,
                            round(self.heading - self.gps_magnetic_variation),
                            round(self.pitch),
                            round(self.roll))
            self.sock.sendto (dvis, (self.xplane_host, self.xplane_port))

            for attr,dest in self.additional_data.items():
                pack = self.dref_struct_preamble + \
                    self.dref_struct_body.pack (getattr(self, attr), dest)
                self.sock.sendto (pack, (self.xplane_host, self.xplane_port))

    def AreSensorsGreen(self):
        ret = not (self.gps_ground_speed is None or 
                self.altitude is None or 
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


if __name__ == "__main__":
    if len(sys.argv) > 1:
        xplanehost = sys.argv[1]
    else:
        xplanehost = 'localhost'
    if len(sys.argv) > 2:
        localport = sys.argv[2]
    else:
        localport = 47951
    display = Display(localport, xplanehost)
    display.listen()

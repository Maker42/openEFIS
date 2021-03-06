#  Copyright (c) 2018-2019 Garrett Herschleb
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

# This is a module that injects the computed magnetic declination given
# lat, long, alt, and date


from MicroServerComs import MicroServerComs

class GroundVector(MicroServerComs):
    def __init__(self, conf_mult=1.0):
        MicroServerComs.__init__(self, "GroundVector")
        self.gps_lat = None
        self.gps_lng = None
        self.gps_ground_speed = None
        self.gps_ground_track = None
        self.gps_signal_quality = None
        self.confidence_multiplier = conf_mult
        self.magnetic_declination = None

    def updated(self, channel):
        if channel == 'gpsfeed':
            self.ground_vector_confidence = self.gps_signal_quality * \
                            self.confidence_multiplier
        if self.magnetic_declination is not None and self.gps_lat is not None:
            self.publish ()

if __name__ == "__main__":
    gv = GroundVector()
    gv.listen()

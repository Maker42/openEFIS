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

from geomag import declination

from MicroServerComs import MicroServerComs

class MagneticDeclination(MicroServerComs):
    def __init__(self, conf_mult=0.1):
        MicroServerComs.__init__(self, "MagneticDeclination")
        self.gps_lat = None
        self.gps_lng = None
        self.gps_altitude = None
        self.last_time = 0
        self.magnetic_declination = None

    def updated(self, channel):
        if self.gps_utc - self.last_time > 10:
            self.last_time = self.gps_utc
            self.magnetic_declination = declination (
                    self.gps_lat, self.gps_lng, self.gps_altitude)
            self.publish ()
            print ("Magnetic Declination: %.1f"%(self.magnetic_declination,))


if __name__ == "__main__":
    magdcl = MagneticDeclination()
    magdcl.listen()

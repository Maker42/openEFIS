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

from Common.util import DEG_RAD, rate_curve
import Common.Spatial as Spatial
import math

from MicroServerComs import MicroServerComs

class HeadingComputed(MicroServerComs):
    def __init__(self, heading_calibration, rais_id):
        self.heading_calibration = heading_calibration
        MicroServerComs.__init__(self, "HeadingComputed")
        self.rais_id = rais_id
        self.roll = None
        self.pitch = None

    def updated(self, channel):
        if channel == 'magneticsensors':
            # Assumes Y axis is pointing forward, X axis is pointing to starboard wing/side
            compensated = True
            if self.roll is None or self.pitch is None or \
                   self.magneticsensors_updated - self.Roll_updated > .3 or \
                   self.magneticsensors_updated - self.Pitch_updated > .3:
                # Assume we're about level for uncompensated heading
                x = self.m_x
                y = self.m_y
                compensated = False
            if compensated:
                mag_vector = Spatial.Vector (self.m_x, self.m_y, self.m_z)
                # Project mag_vector onto aircraft plane
                aircraft_normal = Spatial.Vector(0,0,1)
                aircraft_xvec = Spatial.Vector(1,0,0)
                aircraft_yvec = Spatial.Vector(0,1,0)
                pitch = self.pitch*math.pi/180
                roll = self.roll*math.pi/180
                aircraft_normal.rotate_xaxis(pitch)
                aircraft_normal.rotate_yaxis(roll)
                aircraft_xvec.rotate_xaxis(pitch)
                aircraft_xvec.rotate_yaxis(roll)
                aircraft_yvec.rotate_xaxis(pitch)
                aircraft_yvec.rotate_yaxis(roll)
                origin = Spatial.Point3()
                aircraft_plane = Spatial.Plane(p1=origin, normal=aircraft_normal)
                mag_projection = aircraft_plane.project(mag_vector)
                ac_screen = Spatial.Screen(aircraft_plane,
                        origin, aircraft_xvec, aircraft_yvec)
                x,y = ac_screen.point2D (mag_projection)
            if x == 0 and y == 0:
                # If both x and y are 0, it means the aircraft is pointed
                # straight up or straight down with respect to the magnetic field,
                # so no direction can be determined.
                return  # Don't publish an indeterminate result
                # Next sample probably the alignment will be off, so we'll have a real
                # reading next time.
            theta = math.atan2 (y, x) - math.pi/2
            mag_heading = theta * DEG_RAD
            while mag_heading < 0:
                mag_heading += 360
            while mag_heading >= 360:
                mag_heading -= 360
            self.heading_computed = self._calibrated_heading (mag_heading)
            self.timestamp = self.magneticsensors_updated

            self.publish ()
            print ("HeadingComputed(%s): %g,%g,%g => %g"%('Compensated' if compensated else \
                    'uncompensated', self.m_x, self.m_y, self.m_z, self.heading_computed))
        elif channel == 'admincommand' and self.heading_computed is not None:
            if self.command == b'heading':
                given_file = 'given_heading%d.csv'%self.rais_id
                f = open(given_file, 'a+')
                f.write ('%.2f,%.1f\n'%(self.heading_computed, float(self.args)))
                f.close()

    def _calibrated_heading(self, heading):
        # In each table, estimate the calibrated heading
        # estimate calibrated heading through a piece wise linear function
        if isinstance(self.heading_calibration, dict):
            heading = rate_curve (heading, self.heading_calibration['compass_correction'], False)
        return heading

if __name__ == "__main__":
    hc = HeadingComputed()
    hc.listen()

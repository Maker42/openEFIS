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


from MicroServerComs import MicroServerComs

class Yaw(MicroServerComs):
    def __init__(self, accelerometer_calibration, accel_factor):
        self.accelerometer_calibration = accelerometer_calibration
        MicroServerComs.__init__(self, "Yaw")
        self.accel_factor = accel_factor
        self.xoffset = 0

    def updated(self, channel):
        if channel == 'accelerometers':
            if isinstance(self.accelerometer_calibration, dict):
                self.a_x = util.rate_curve (self.a_x,
                        self.accelerometer_calibration['x'])
            self.a_x += self.xoffset
            self.yaw = self.a_x * self.accel_factor
            self.timestamp = self.accelerometers_updated
            self.yaw_confidence = 10.0
            self.publish ()
            print ("Yaw: %g => %g"%(self.a_x, self.yaw))
        elif channel == 'systemcommand':
            if self.command.startswith( b'0att'):
                self.xoffset = -self.a_x


if __name__ == "__main__":
    yaw = Yaw()
    yaw.listen()

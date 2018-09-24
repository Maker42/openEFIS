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

class Yaw(MicroServerComs):
    def __init__(self, accel_factor=1.0):
        MicroServerComs.__init__(self, "Yaw")
        self.accel_factor = accel_factor

    def updated(self, channel):
        self.yaw = self.a_x * self.accel_factor
        self.timestamp = self.accelerometers_updated
        self.yaw_confidence = 10.0
        self.publish ()
        print ("Yaw: %g => %g"%(self.a_x, self.yaw))


if __name__ == "__main__":
    yaw = Yaw()
    yaw.listen()

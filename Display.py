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

from MicroServerComs import MicroServerComs

from TkDisplay import StartDisplayWindow

class Display(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "Display")
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
        self.display_window = StartDisplayWindow()
        self.update_period = 1.0/30.0
        self.next_update_time = time.time() + self.update_period

    def updated(self, channel):
        tm = time.time()
        if channel == 'Roll' and self.AreSensorsGreen() and tm >= self.next_update_time:
            self.display_window.display_horizon (self.pitch, self.roll)
            self.display_window.display_airspeed (self.airspeed)
            self.display_window.display_altitude (self.altitude)
            self.display_window.display_climb_rate (int(round(self.climb_rate/10)*10))
            self.display_window.display_instruments (self.heading, None, self.turn_rate, self.yaw)
            while self.next_update_time < tm:
                self.next_update_time += self.update_period
            self.display_window.update()

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


if __name__ == "__main__":
    display = Display()
    display.listen()

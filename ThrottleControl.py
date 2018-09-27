# Copyright (C) 2015-2018  Garrett Herschleb
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

class ThrottleControl:
    def __init__(self, throttle_table, throttle_channels):
        self.controller = None
        self.last_val = 0.0
        self._throttle_table = throttle_table
        self._throttle_channels = throttle_channels

    def SetServoController(self, c):
        self.controller = c
        self.controller.SetThrottleTable(self._throttle_table)
        self.controller.SetThrottleChannels(self._throttle_channels)

    def Set(self, val):
        self.last_val = val
        self.controller.SetThrottles (val)

    def GetCurrent(self):
        return self.last_val

    def GetLimits(self):
        mn = 9999999
        mx = -9999999
        for k,v in self._throttle_table:
            mn = mn if mn < k else k
            mx = mx if mx > k else k
        return (mn,mx)

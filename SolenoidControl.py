# Copyright (C) 2015  Garrett Herschleb
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

class SolenoidControl:
    def __init__(self, pin):
        self.pin = pin
        self._current_val = 0
        self._controller = None

    def Set(self, val):
        self._current_val = val
        self._controller.SetPin(self.pin, val)
        # TODO: connect interface to control a pin output

    def SetServoController(self, sc):
        self._controller = sc
        self._controller.SetPin(self.pin, self._current_val)

    def GetCurrent(self):
        return self._current_val

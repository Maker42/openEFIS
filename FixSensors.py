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

import logging, time

import fix

logger=logging.getLogger(__name__)

class FixSensors:
    def __init__(self):
        self.AltitudeDB = fix.db.get_item("ALT")
        self.HeadingDB = fix.db.get_item("HEAD")
        self.RollDB = fix.db.get_item("ROLL")
        self.RollRateDB = fix.db.get_item("ROLLRATE")
        self.PitchDB = fix.db.get_item("PITCH")
        self.PitchRateDB = fix.db.get_item("PITCHRATE")
        self.TurnRateDB = fix.db.get_item("TURNRATE")
        self.YawDB = fix.db.get_item("YAW")
        self.AirSpeedDB = fix.db.get_item("IAS")
        self.GroundSpeedDB = fix.db.get_item("GS")
        self.ClimbRateDB = fix.db.get_item("VS")
        self.LongitudeDB = fix.db.get_item("LONG")
        self.LatitudeDB = fix.db.get_item("LAT")
        self.MagneticDeclinationDB = fix.db.get_item("MAGDCL")
        self.GroundTrackDB = fix.db.get_item("TRACK")

    def initialize(self, filelines):
        pass

    def Altitude(self):
        return self.AltitudeDB.value

    def Heading(self):
        return self.HeadingDB.value

    def Roll(self):
        return self.RollDB.value

    def RollRate(self):
        return self.RollRateDB.value

    def Pitch(self):
        return self.PitchDB.value

    def PitchRate(self):
        return self.PitchRateDB.value

    def Yaw(self):
        return self.YawDB.value

    def AirSpeed(self):
        return self.AirSpeedDB.value

    def GroundSpeed(self):
        return self.GroundSpeedDB.value

    def ClimbRate(self):
        return self.ClimbRateDB.value

    def Position(self):
        return (self.LongitudeDB.value, self.LatitudeDB.value)

    def HeadingRateChange(self):
        return self.TurnRateDB.value

    def TrueHeading(self):
        return self.HeadingDB.value - self.MagneticDeclinationDB.value

    def MagneticDeclination(self):
        return self.MagneticDeclinationDB.value

    def Time(self):
        return time.time()

    # Actual flight path in true coordinates
    def GroundTrack(self):
        return self.GroundTrackDB.value

    def WindSpeed(self):
        return 0

    def WindDirection(self):
        return 0

    def AGL(self):
        return 0

    def GearUpLocked(self):
        return True

    def FlapPosition(self):
        return 0.0

    def OuterEnginePosition(self):
        return 0

    def Snapshot(self):
        pass

    def Battery(self):
        return 100

    def EnginesOut(self):
        return 0

    def KnownAltitude(self, alt):
        pass

    def KnownMagneticVariation(self, v):
        pass

    def FlightMode (self, mode, vertical=True):
        pass

    def WaitSensorsGreen(self):
        pass

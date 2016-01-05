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

import time, math, logging

import serial

import Globals, Spatial, SenseControlRemote

logger=logging.getLogger(__name__)

scmaster = None

class Control:
    def __init__(self, remotehost="localhost", remoteport=49001, localport=49000):
        global scmaster
        scmaster = SenseControlRemote.SenseControlMaster(localport, remotehost, remoteport)
        self._lookup_tables = list()
        self._throttle_table = None

    def SetAnalogChannel(self, channel, val):
        channel = int(channel)
        if channel < 0 or channel >= len(self._lookup_tables) or self._lookup_tables[channel] == None:
            raise RuntimeError ("Invalid channel set (%d)"%channel)
        scaled_val = look_up (val, self._lookup_tables[channel])
        scaled_val = int(round(scaled_val))
        scmaster.SetAnalogChannel (channel, scaled_val)

    def SetDigitalChannel(self, channel, val):
        channel = int(channel)
        scaled_val = int(val)
        scmaster.SetDigitalChannel (channel, scaled_val)

    def SetThrottles(self, throttles, nthrottles):
        if isinstance(throttles,list):
            throttle_values = [int(round(look_up (t, self._throttle_table))) for t in throttles]
        else:
            value = int(round(look_up (throttles, self._throttle_table)))
            throttle_values = [value for i in range(nthrottles)]

        args = tuple(throttle_values)
        scmaster.SetThrottles (*throttle_values)
        logger.log (3, "Setting throttles to %s", str(throttles))

    def SetThrottleTable(self, table):
        self._throttle_table = table

    def SetLookupTable(self, channel, table):
        while len(self._lookup_tables) <= channel:
            self._lookup_tables.append(None)
        self._lookup_tables[channel] = table

    def initialize(self, filelines):
        return

    def GetLimits(self, channel):
        if channel < 0 or channel >= len(self._lookup_tables) or self._lookup_tables[channel] == None:
            raise RuntimeError ("Invalid channel set (%d)"%channel)
        mn = 9999999
        mx = -9999999
        for k,v in self._lookup_tables[channel]:
            mn = mn if mn < k else k
            mx = mx if mx > k else k
        return (mn,mx)


class Sensors:
    def __init__(self):
        self._magnetic_variation = None
        self._WindDirection = None
        self._WindSpeed = None


    def initialize(self, filelines):
        pass

    def FlightMode(self, mode, vertical = True):
        scmaster.FlightMode (mode, vertical)

    def KnownAltitude(self, alt):
        scmaster.KnownAltitude(alt)

    def KnownMagneticVariation(self, v):
        self._magnetic_variation = v
        scmaster.MagneticVariation(v)

    def WindVector(self, v):
        self._corrected_sensors.WindVector (v)
        self._WindDirection = math.atan2 (v.x, v.y)      # X & Y switched because aviation compass is shifted 90 degrees
        self._WindSpeed = v.norm()
        scmaster.WindVector(v)

    def FlightParameters(self, *args):
        scmaster.FlightParameters (*args)

    def ThrottleLevel (self, l):
        scmaster.ThrottleLevel(l)

    def Altitude(self):
        scmaster.Update()
        return scmaster.Sensors['Altitude'][0]

    def Heading(self):
        scmaster.Update()
        return scmaster.Sensors['Heading'][0]

    def Roll(self):
        scmaster.Update()
        return scmaster.Sensors['Roll'][0]

    def RollRate(self):
        scmaster.Update()
        return scmaster.Sensors['RollRate'][0]

    def Pitch(self):
        scmaster.Update()
        return scmaster.Sensors['Pitch'][0]

    def PitchRate(self):
        scmaster.Update()
        return scmaster.Sensors['PitchRate'][0]

    def Yaw(self):
        scmaster.Update()
        return scmaster.Sensors['Yaw'][0]

    def AirSpeed(self):
        scmaster.Update()
        return scmaster.Sensors['AirSpeed'][0]

    def ClimbRate(self):
        scmaster.Update()
        return scmaster.Sensors['ClimbRate'][0]

    def Position(self):
        scmaster.Update()
        return scmaster.Sensors['Position'][0]

    def HeadingRateChange(self):
        scmaster.Update()
        return scmaster.Sensors['HeadingRate'][0]

    def TrueHeading(self):
        scmaster.Update()
        if self._magnetic_variation != None:
            return scmaster.Sensors['Heading'][0] - self._magnetic_variation
        else:
            return scmaster.Sensors['Heading'][0]

    def MagneticDeclination(self):
        if self._magnetic_variation != None:
            return self._magnetic_variation
        else:
            return 0.0

    def Time(self):
        return time.time()

    # Actual flight path in true coordinates
    def GroundTrack(self):
        scmaster.Update()
        return scmaster.Sensors['GroundTrack'][0]

    def GroundSpeed(self):
        scmaster.Update()
        return scmaster.Sensors['GroundSpeed'][0]

    def WindSpeed(self):
        return self._WindSpeed

    def WindDirection(self):
        return self._WindDirection

    def AGL(self):
        scmaster.Update()
        raise RuntimeError("AGL sensor Not implemented")

    def Battery(self):
        return 100

def look_up(keyin, lookup_table, continuous=False, reverse=False):
    last_keypoint = None
    last_val = None

    if reverse:
        KEYINDEX = 1
        VALINDEX = 0
    else:
        KEYINDEX = 0
        VALINDEX = 1

    # Check if we're off the piece-wise linear portion of the lookup table
    l1index = 0     # Index containing the lowest key value
    l2index = 1     # Index containing the 2nd lowest key value
    h1index = -1    # Index containing the highest key value
    h2index = -2    # Index containing the 2nd highest key value
    if lookup_table[0][KEYINDEX] > lookup_table[-1][KEYINDEX]:    # Inverted slope
        h1index = 0
        h2index = 1
        l1index = -1
        l2index = -2

    if keyin < lookup_table[l1index][KEYINDEX]:      # If we're off the range of lookup table on the negative side
        if continuous:    # If this supports continuous range of values,
            # calculate the value based on the last slope of the curve.
            slope = (float(lookup_table[l2index][VALINDEX] - lookup_table[l1index][VALINDEX]) /
                    float(lookup_table[l2index][KEYINDEX] - lookup_table[l1index][KEYINDEX]))
            dist = keyin - lookup_table[l1index][KEYINDEX]
            val = lookup_table[l1index][VALINDEX] + dist * slope
            return val
        else:
            return lookup_table[l1index][VALINDEX]
    if keyin > lookup_table[h1index][KEYINDEX]:      # If we're off the range of lookup table on the positive side
        if continuous:    # If this supports continuous range of values,
            # calculate the value based on the last slope of the curve.
            slope = (float(lookup_table[h1index][VALINDEX] - lookup_table[h2index][VALINDEX]) /
                float(lookup_table[h1index][KEYINDEX] - lookup_table[h2index][KEYINDEX]))
            dist = keyin - lookup_table[h1index][KEYINDEX]
            val = lookup_table[h1index][VALINDEX] + dist * slope
            return val
        else:
            return lookup_table[h1index][VALINDEX]

    # We're in the middle of the lookup table, so search through the piece-wise linear graph for the right value.
    for entry in lookup_table:
        val = entry[VALINDEX]
        keypoint = entry[KEYINDEX]
        if last_keypoint != None:
            if keyin >= last_keypoint and keyin <= keypoint:
                newval = (val - float(last_val)) * (float(keyin) - float(last_keypoint)) / (float(keypoint) - float(last_keypoint)) + float(last_val)
                return newval
            elif keyin >= keypoint and keyin <= last_keypoint:
                newval = (last_val - val) * (keyin - keypoint) / (last_keypoint - keypoint) + val
                return newval
        last_keypoint = keypoint
        last_val = val
    return None


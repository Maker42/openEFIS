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

import time, math, logging

from MicroServerComs import MicroServerComs
from PubSub import assign_all_ports

import yaml

logger=logging.getLogger(__name__)

class Control(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "Control")
        self._lookup_tables = dict()
        self._throttle_table = None
        self.engaged = False

    def SetAnalogChannel(self, channel, val):
        if not channel in self._lookup_tables:
            raise RuntimeError ("Invalid channel set (%s)"%channel)
        scaled_val = look_up (val, self._lookup_tables[channel])
        scaled_val = int(round(scaled_val))
        self.channel = channel
        self.value = scaled_val
        self.publish()

    def SetDigitalChannel(self, channel, val):
        self.channel = channel
        self.value = int(val)
        self.publish()

    def SetThrottles(self, throttles):
        if isinstance(throttles,list):
            throttle_values = [int(round(look_up (t, self._throttle_table))) for t in throttles]
        else:
            value = int(round(look_up (throttles, self._throttle_table)))
            throttle_values = [value for i in range(len(self._throttle_channels))]

        if len(self._throttle_channels) != len(throttle_values):
            raise RuntimeError ("SetThrottles: value list length does not match number of engines")
        for ch,val in zip(self._throttle_channels,throttle_values):
            self.channel = ch
            self.value = val
            self.publish()
        logger.log (3, "Setting throttles to %s", str(throttles))

    def SetThrottleTable(self, table):
        self._throttle_table = table

    def SetThrottleChannels(self, chs):
        self._throttle_channels = chs

    def SetLookupTable(self, channel, table):
        self._lookup_tables[channel] = table

    def initialize(self, filelines):
        return

    def GetLimits(self, channel):
        if not channel in self._lookup_tables:
            raise RuntimeError ("Invalid channel set (%s)"%channel)
        mn = 9999999
        mx = -9999999
        for k,v in self._lookup_tables[channel]:
            mn = mn if mn < k else k
            mx = mx if mx > k else k
        return (mn,mx)

    def SetEngagedState(self, engage):
        self.engaged = engage
        self.publish()


class Sensors(MicroServerComs):
    def __init__(self, pubsub_cfg, starting_port):
        if isinstance(pubsub_cfg, str):
            with open (pubsub_cfg, 'r') as yml:
                cfg = yaml.load (yml, Loader=yaml.SafeLoader)
                yml.close()
        else:
            cfg = pubsub_cfg
        assign_all_ports (cfg, starting_port)
        MicroServerComs.__init__(self, "Autopilot", config=cfg)
        self.magnetic_declination = None
        self._system_command = SystemCommand(cfg)
        self.altitude = None
        self.altitude_confidence = 0
        self.airspeed = None
        self.airspeed_confidence = 0
        self.heading = None
        self.heading_confidence = 0
        self.roll = None
        self.roll_confidence = 0
        self.roll_rate = None
        self.roll_rate_confidence = 0
        self.pitch = None
        self.pitch_confidence = 0
        self.pitch_rate = None
        self.pitch_rate_confidence = 0
        self.yaw = None
        self.yaw_confidence = 0
        self.climb_rate = None
        self.climb_rate_confidence = 0
        self.turn_rate = None
        self.turn_rate_confidence = 0
        self.gps_utc = None
        self.gps_lat = None
        self.gps_lng = None
        self.gps_ground_speed = None
        self.gps_ground_track = None
        self.gps_signal_quality = 0
        self.last_update_time = 0

    def initialize(self, fl):
        pass

    def FlightMode(self, mode, vertical = 1):
        self._system_command.send ('fmode', ' '.join([mode, str(vertical)]))

    def SendBarometer(self, b):
        self._system_command.send ('baroinhg', str(b))

    def WindsAloftReport(self, lat, lng, altitude, timestamp, direction, speed):
        self._system_command.send ('windsalft',
                ' '.join([lat, lng, altitude, timestamp, direction, speed]))

    def AtisWinds(self, speed, direction):
        self._system_command.send ("atis", ' '.join([str(speed), str(direction)]))

    def Altitude(self):
        self.listen (timeout=0, loop=False)
        return self.altitude

    def AltitudeConfidence(self):
        return self.altitude_confidence

    def Heading(self):
        self.listen (timeout=0, loop=False)
        return self.heading

    def HeadingConfidence(self):
        return self.heading_confidence

    def Roll(self):
        self.listen (timeout=0, loop=False)
        return self.roll

    def RollConfidence(self):
        return self.roll_confidence

    def RollRate(self):
        self.listen (timeout=0, loop=False)
        return self.roll_rate

    def RollRateConfidence(self):
        return self.roll_rate_confidence

    def Pitch(self):
        self.listen (timeout=0, loop=False)
        return self.pitch

    def PitchConfidence(self):
        return self.pitch_confidence

    def PitchRate(self):
        self.listen (timeout=0, loop=False)
        return self.pitch_rate

    def PitchRateConfidence(self):
        return self.pitch_rate_confidence

    def Yaw(self):
        self.listen (timeout=0, loop=False)
        return self.yaw

    def YawConfidence(self):
        return self.yaw_confidence

    def AirSpeed(self):
        self.listen (timeout=0, loop=False)
        return self.airspeed

    def AirSpeedConfidence(self):
        return self.airspeed_confidence

    def ClimbRate(self):
        self.listen (timeout=0, loop=False)
        return self.climb_rate

    def ClimbRateConfidence(self):
        return self.climb_rate_confidence

    def Position(self):
        self.listen (timeout=0, loop=False)
        return (self.gps_lng, self.gps_lat)

    def GPSSignalQuality(self):
        return self.gps_signal_quality

    def HeadingRateChange(self):
        self.listen (timeout=0, loop=False)
        return self.turn_rate

    def HeadingRateChangeConfidence(self):
        return self.turn_rate_confidence

    def TrueHeading(self):
        self.listen (timeout=0, loop=False)
        if self.magnetic_declination is not None:
            return self.heading + self.magnetic_declination
        else:
            return self.heading

    def MagneticDeclination(self):
        if self.magnetic_declination != None:
            return self.magnetic_declination
        else:
            return 0.0

    def OuterEnginePosition(self):
        # TODO: Complete sensor loop
        return "vertical"

    def Time(self):
        return self.last_update_time

    # Actual flight path in true coordinates
    def GroundTrack(self):
        self.listen (timeout=0, loop=False)
        return self.gps_ground_track

    def GroundSpeed(self):
        self.listen (timeout=0, loop=False)
        return self.gps_ground_speed

    def AGL(self):
        self.listen (timeout=0, loop=False)
        raise RuntimeError("AGL sensor Not implemented")

    def Battery(self):
        return 100

    def EnginesOut(self):
        return 0

    def Snapshot(self):
        return str(dir(self))
    
    def updated(self, channel):
        return

    def WaitSensorsGreen(self):
        while not self.Ready():
            time.sleep (.1)

    def Set0AirSpeed (self):
        self._system_command.send ('0airspeed', '')

    def Set0Attitude (self):
        self._system_command.send ('0attitude', '')

    def Ready(self):
        self.listen (timeout=0, loop=False)
        return not (
                self.altitude is None or \
                #self.airspeed is None or \
                self.heading is None or \
                self.roll is None or \
                self.pitch is None or \
                self.yaw is None or \
                self.turn_rate is None or \
                self.pitch_rate is None or \
                self.roll_rate is None or \
                self.climb_rate is None)

class KnownAltitude(MicroServerComs):
    def __init__(self, cfg):
        self.known_altitude = None
        MicroServerComs.__init__(self, "KnownAltitude", channel='knownaltitude', config=cfg)

    def send(self, alt):
        self.known_altitude = alt
        self.publish()

class WindsAloftReport(MicroServerComs):
    def __init__(self, cfg):
        MicroServerComs.__init__(self, "WindsAloftReport", channel='windsaloftreport', config=cfg)

    def send (self, lat, lng, altitude, timestamp, direction, speed):
        self.wa_lat = lat
        self.wa_lng = lng
        self.wa_altitude = altitude
        self.wa_time = timestamp
        self.wa_heading = direction
        self.wa_speed = speed
        self.publish()

class SystemCommand(MicroServerComs):
    def __init__(self, cfg=None):
        self.command = None
        self.args = None
        MicroServerComs.__init__(self, "SystemCommand", channel='systemcommand', config=cfg)

    def send(self, c, a):
        self.command = bytes(c, encoding='utf8')
        self.args = bytes(a, encoding='utf8')
        # print ("System Command %s(%s) published %s"%(c, a, str(self.pubchannel.getpeername())))
        self.publish()


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


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

import NMEAParser, Globals, Spatial, ArduinoCmdMessenger

logger=logging.getLogger(__name__)

METERS_FOOT = 0.3048
FEET_METER = 1.0 / METERS_FOOT

CMD_ANALOG_SET_CHANNEL =1
CMD_DIGITAL_SET_CHANNEL=2
CMD_10DOF_SENSOR_DATA  =3
CMD_GPS_DATA           =4
CMD_LOG                =250
CMD_SET_THROTTLES      =6

class ArduinoControl:
    def __init__(self, portname):
        global control
        control = self
        self._lookup_tables = list()
        self._throttle_table = None
        try:
            self._main_port = serial.Serial(portname, 115200, timeout=0)
            self._mainCmd = ArduinoCmdMessenger.CmdMessenger(self._main_port, arg_delimiter='^')
            self._mainCmd.attach (CMD_LOG, self._logMainCmd)
        except:
            raise RuntimeError("Cannot open control/sensor port %s"%portname)

    def _logMainCmd(self, args):
        try:
            level,string,checksum = args
            level = int(level)
            checksum = int(checksum)
            if ArduinoCmdMessenger.ComputeCheckSum (level, string, checksum) == 0:
                logger.log (level, "Main Arduino: %s", string)
            else:
                logger.warning ("Main Arduino log command bad checksum")
        except:
            logger.warning("Can't log main Arduino log")

    def SetAnalogChannel(self, channel, val):
        channel = int(channel)
        if channel < 0 or channel >= len(self._lookup_tables) or self._lookup_tables[channel] == None:
            raise RuntimeError ("Invalid channel set (%d)"%channel)
        scaled_val = look_up (val, self._lookup_tables[channel])
        scaled_val = int(round(scaled_val))
        for retry in range(5):
            logger.log (3, "Setting analog channel %d to %d", channel, scaled_val)
            try:
                self._mainCmd.sendValidatedCmd(CMD_ANALOG_SET_CHANNEL, channel, scaled_val)
                break
            except:
                time.sleep(.01)

    def SetDigitalChannel(self, channel, val):
        channel = int(channel)
        scaled_val = int(val)
        for retry in range(5):
            logger.log (3, "Setting digital channel %d to %d", channel, scaled_val)
            try:
                self._mainCmd.sendValidatedCmd(CMD_DIGITAL_SET_CHANNEL, channel, scaled_val)
                break
            except:
                time.sleep(.01)

    def SetThrottles(self, throttles, nthrottles):
        if isinstance(throttles,list):
            throttle_values = [int(round(look_up (t, self._throttle_table))) for t in throttles]
        else:
            value = int(round(look_up (throttles, self._throttle_table)))
            throttle_values = [value for i in range(nthrottles)]

        args = tuple(throttle_values)
        self._mainCmd.sendValidatedCmd(CMD_SET_THROTTLES, *args)
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


class ArduinoSensors:
    def __init__(self):
        global control
        self.SamplesPerSecond = 10
        self.residual_input = ""
        self.reading_time = 0.0
        self.climb_rate_history = list()
        self.heading_rate_history = list()
        self._sea_level_pressure = 101.325      # 1 Standard atm

        self._Altitude = None
        self._Heading = None
        self._HeadingRate = None
        self._AirSpeed = None
        self._ClimbRate = None
        self._Roll = None
        self._Pitch = None
        self._Yaw = None

        self._flight_mode = Globals.FLIGHT_MODE_GROUND
        self._altitude_valid = False
        self._known_altitude = None
        self._magnetic_variation = None
        self._gps_altitude_variance = None

        # Readings from GPS
        self.utc = None
        self.latitude = None
        self.longitude = None
        self.gps_altitude = None
        self.ground_speed = None
        self.ground_track = None
        self.magnetic_variation = 0.0

        # Readings from 10DOF sensors
        self.pressure = None
        self.temp = None
        self.a_forward = None
        self.a_side = None
        self.a_up = None
        self.r_pitch = None
        self.r_roll = None
        self.r_yaw = None
        self.m_forward = None
        self.m_side = None
        self.m_up = None
        control._mainCmd.attach(CMD_10DOF_SENSOR_DATA, self.Get10DOFSensorData)
        control._mainCmd.attach(CMD_GPS_DATA, self.GetGPSData)

    def initialize(self, filelines):
        global control
        self.all_sensors_go = False
        while not self.all_sensors_go:
            self.ProcessIncoming()
            time.sleep(.01)
            break
        return

    def FlightMode(self, mode, vertical = True):
        self._flight_mode = mode
        self._vertical = vertical

    def KnownAltitude(self, alt):
        self._known_altitude = alt

    def KnownMagneticVariation(self, v):
        self._magnetic_variation = v

    def Altitude(self):
        self.ProcessIncoming()
        return self._Altitude

    def Heading(self):
        self.ProcessIncoming()
        return self._Heading

    def Roll(self):
        self.ProcessIncoming()
        return self._Roll

    def RollRate(self):
        self.ProcessIncoming()
        return self.r_roll

    def Pitch(self):
        self.ProcessIncoming()
        return self._Pitch

    def PitchRate(self):
        self.ProcessIncoming()
        return self.r_pitch

    def Yaw(self):
        self.ProcessIncoming()
        return self._Yaw

    def AirSpeed(self):
        self.ProcessIncoming()
        return self._AirSpeed

    def GroundSpeed(self):
        self.ProcessIncoming()
        return self.ground_speed

    def ClimbRate(self):
        self.ProcessIncoming()
        return self._ClimbRate

    def Position(self):
        self.ProcessIncoming()
        return (self.longitude, self.latitude)

    def HeadingRateChange(self):
        self.ProcessIncoming()
        return self._HeadingRate

    def TrueHeading(self):
        self.ProcessIncoming()
        if self._magnetic_variation != None:
            return self._Heading - self._magnetic_variation
        elif self.ground_track != None:
            return self.ground_track
        else:
            return self._Heading

    def MagneticDeclination(self):
        self.ProcessIncoming()
        if self._magnetic_variation != None:
            return self._magnetic_variation
        else:
            return 0.0

    def Time(self):
        return time.time()

    # Actual flight path in true coordinates
    def GroundTrack(self):
        self.ProcessIncoming()
        return self.ground_track

    def WindSpeed(self):
        self.ProcessIncoming()
        return self._WindSpeed

    def WindDirection(self):
        self.ProcessIncoming()
        return self._WindDirection

    def AGL(self):
        self.ProcessIncoming()
        raise RuntimeError("AGL sensor Not implemented")

    def ProcessIncoming(self):
        global control
        control._mainCmd.feedinSerialData()

    def Battery(self):
        return 100

    def Get10DOFSensorData(self, args):
        line,checksum = args
        checksum = int(checksum)
        if 0 == ArduinoCmdMessenger.ComputeCheckSum(line, checksum):
            line = line.strip()
            fields = line.split(',')
            if len(fields) < 12:
                logger.warning("Invalid sensor string: %s", line)
            else:
                try:
                    self.pressure = float(fields[0])
                except:
                    self.pressure = None
                    logger.debug("10DOF sensors are not reading. Do not fly")
                try:
                    self.temp = float(fields[1])
                except:
                    self.temp = None
                try:
                    self.r_pitch = float(fields[2])
                except:
                    self.r_pitch = None
                try:
                    self.r_roll = float(fields[3])
                except:
                    self.r_roll = None
                try:
                    self.r_yaw = float(fields[4])
                except:
                    self.r_yaw = None
                try:
                    self.a_forward = float(fields[5])
                except:
                    self.a_forward = None
                try:
                    self.a_side = float(fields[6])
                except:
                    self.a_side = None
                try:
                    self.a_up = float(fields[7])
                except:
                    self.a_up = None
                try:
                    self.m_forward = float(fields[8])
                except:
                    self.m_forward = None
                try:
                    self.m_side = float(fields[9])
                except:
                    self.m_side = None
                try:
                    self.m_up = float(fields[10])
                except:
                    self.m_up = None
                self.last_reading_time = self.reading_time
                try:
                    self.reading_time = int(fields[11])
                except:
                    return
                logger.log(1, "Got sensor readings 10DOF: pressure = %s, temp=%s, a=%s,%s,%s, r=%s,%s,%s, m=%s,%s,%s",
                self.pressure,
                self.temp,
                self.a_forward,
                self.a_side,
                self.a_up,
                self.r_pitch,
                self.r_roll,
                self.r_yaw,
                self.m_forward,
                self.m_side,
                self.m_up)
            self.ProcessRawInputs()
        else:
            logger.warning("10DOF Sensor Data checksum mismatch %d !> %d"%(checksum, ArduinoCmdMessenger.ComputeCheckSum(line, checksum)))


    def GetGPSData(self, args):
        line = args[0]
        NMEAParser.ParseNMEAStrings(line, self)
        self.ProcessRawInputs()

    def ProcessRawInputs(self):
        if self.m_side == None: # No sensor input yet
            return
        if self.m_side != None and self.m_up != None and self.m_forward != None:
            mag_vector = Spatial.Vector(self.m_side, self.m_up, self.m_forward)
            mag_polar = mag_vector.to_polar(limit_phi = True, robot_coordinates=False)
            self.PreviousHeading = self._Heading
            self._Heading = mag_polar.theta * util.DEG_RAD
        self.PreviousAltitude = self._Altitude
        self._Altitude = self.AltitudeFromPressure(self.pressure)
        if self.last_reading_time > 0 and self._Altitude != None:
            if self.last_reading_time > self.reading_time:
                # time rollover
                timediff = self.last_reading_time - self.reading_time - (2 << 32)
            else:
                timediff = self.reading_time - self.last_reading_time
            # Differentiate some inputs
            if self._altitude_valid:
                current_climb_rate = (self._Altitude - self.PreviousAltitude) / (timediff / 1000.0)
                self._ClimbRate = util.LowPassFilter (current_climb_rate, self.climb_rate_history)
                if len(self.climb_rate_history) >= len(util.LowPassFIR) and self.signal_quality > 0:
                    self.all_sensors_go = True
            current_heading_rate = (self._Heading - self.PreviousHeading) / (timediff / 1000.0)
            self._HeadingRate = util.LowPassFilter (current_heading_rate, self.heading_rate_history)
        else:
            self._ClimbRate = 0.0
            self._HeadingRate = 0.0
        if (self._flight_mode == Globals.FLIGHT_MODE_GROUND or
                (self._flight_mode != Globals.FLIGHT_MODE_AIRBORN and self._vertical)):
            if self.a_up == None or self.a_up == 0.0:
                logger.debug ("Do not Fly. Vertical Accelerometer reading 0")
                self._Pitch = None
                self._Roll = None
                self._Yaw = None
            else:
                self._Pitch = math.atan(self.a_forward / self.a_up) * util.DEG_RAD
                self._Roll = math.atan(self.a_side / self.a_up) * util.DEG_RAD
                self._Yaw = 0.0
            # TODO: Run a ground based correlation on all sensors. Raise an exception of a problem is found
        else:
            self._Pitch += self.r_pitch
            self._Roll += self.r_roll
            self._Yaw = math.atan (self.a_side / (self.a_forward + self.a_up))
            estimated_pitch = math.atan(self.a_forward / self.a_up) * util.DEG_RAD
            # TODO: regress attitude measurements toward estimates
            # TODO: Run a flight based correlation on all sensors. Adjust accuracy estimates if
            #   readings must revert to a secondary sensor.
        if self._flight_mode != Globals.FLIGHT_MODE_GROUND:
            self._known_altitude = None

        # mag heading - mag variation = true heading
        # true heading + mag variation = mag heading
        if self.ground_track != None and self._Heading != None:
            if self.magnetic_variation != 0.0:      # Let GPS's opinion of magnetic variation trump
                self._magnetic_variation = self.magnetic_variation
            crab_angle = self._Heading - (self.ground_track + self.magnetic_variation)
            # wind component / tas = sin(crab_angle)
            # wind component / ground speed = tan(crab_angle)
            # TODO: get wind component from ground based neumometer
            wind_component = math.tan(crab_angle) * self.ground_speed
            tas = wind_component / math.sin(crab_angle)
            self._AirSpeed = self.TasToCas(tas)
        else:
            self._AirSpeed = None

    def TasToCas(self, tas):
        return tas / math.sqrt(
                self.AirDensity (self._sea_level_pressure, self.StandardSeaLevelTemp()) /
                self.AirDensity(self.pressure, self.temp)
               )

    def AltitudeFromPressure(self, pressure):
        if pressure == None:
            if self.gps_altitude != None:
                if self._gps_altitude_variance == None:
                    ret = self.gps_altitude
                else:
                    ret = self.gps_altitude + self._gps_altitude_variance
            else:
                ret = None
        elif self._known_altitude != None:
            self._sea_level_pressure = pressure / pow(1.0 - (self._known_altitude * METERS_FOOT /44330.0), 5.255)
            ret = self._known_altitude
            if hasattr(self, "gps_altitude"):
                self._gps_altitude_variance = self._known_altitude - self.gps_altitude
        else:
            ret = 44330.0 * (1.0 - pow(pressure / self._sea_level_pressure, 0.1903))
            ret *= FEET_METER
            self._altitude_valid = True
        return ret
    
    # This function compute density for dry air. Humidity is not taken into account
    # pressure: in kilo Pascal (kPa)
    # temp: in degrees C
    def AirDensity(self, pressure, temp):
        temp += 273.15          # KELVIN C OFFSET
        pressure *= 1000.0       # 1 kPa = 1000 Pa
        return pressure / (temp * 287.058)

    def StandardSeaLevelTemp(self):
        return self.temp - 1.98 * self._Altitude / 1000.0

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


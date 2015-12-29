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

import NMEAParser, Globals, Spatial

logger=logging.getLogger(__name__)

METERS_FOOT = 0.3048
FEET_METER = 1.0 / METERS_FOOT

class ArduinoControl:
    def __init__(self, portname):
        global control
        control = self
        self.ServoRange = (-1.0, 1.0)
        self.arduino = serial.Serial(portname, 115200, timeout=0)
        if not self.arduino:
            raise RuntimeError("Cannot open serial port %s"%portname)

    def SetChannel(self, channel, val):
        for retry in range(5):
            if channel < 0 or channel >= len(self.controls):
                raise RuntimeError ("Invalid channel set (%d)"%channel)
            logger.log (3, "Setting channel %d to %g", channel, val)
            #print ("Setting channel %s = %g (%g)"%(self.controls[channel][1], val, scaled_value))
            try:
                #cmd = self.dref_struct_preamble + self.dref_struct_body.pack(channel, val)
                #self.arduino.write(cmd)
                break
            except:
                time.sleep(.01)

    def SetThrottles(self, throttles):
        cmd = self.data_struct_preamble + self.data_struct_body.pack(self.throttle_index,
                throttles[0],
                throttles[1],
                throttles[2],
                throttles[3],
                throttles[4],
                throttles[5],
                0.0, 0.0)
        logger.log (3, "Setting throttles to %g,%g,%g,%g  %g,%g", throttles[0],
                throttles[1],
                throttles[2],
                throttles[3],
                throttles[4],
                throttles[5],
                )
        control.sock.sendto(cmd, (self.xplane_host, self.xplane_port))

    def initialize(self, filelines):
        self.servo_range_size = self.ServoRange[1] - self.ServoRange[0]
        return

    def GetLimits(self, channel):
        return self.ServoRange

class ArduinoSensors:
    def __init__(self):
        self.SamplesPerSecond = 10
        self.residual_input = ""
        self.reading_time = 0.0
        self.climb_rate_history = list()
        self.heading_rate_history = list()
        self._sea_level_pressure = 0.0
        self.Altitude = None
        self.Heading = None
        self._flight_mode = Globals.FLIGHT_MODE_GROUND
        self._altitude_valid = False

    def initialize(self, filelines):
        global control
        self.all_sensors_go = False
        while not self.all_sensors_go:
            self.ProcessIncoming()
            time.sleep(.01)
        return

    def FlightMode(self, mode, vertical = True, known_altitude = None):
        self._flight_mode = mode
        self._vertical = vertical
        self._known_altitude = known_altitude

    def Altitude(self):
        self.ProcessIncoming()
        return self.Altitude

    def Heading(self):
        self.ProcessIncoming()
        return self.Heading

    def Roll(self):
        self.ProcessIncoming()
        return self.Roll

    def RollRate(self):
        self.ProcessIncoming()
        return self.r_roll

    def Pitch(self):
        self.ProcessIncoming()
        return self.Pitch

    def PitchRate(self):
        self.ProcessIncoming()
        return self.r_pitch

    def Yaw(self):
        self.ProcessIncoming()
        return self.Yaw

    def AirSpeed(self):
        self.ProcessIncoming()
        return self.AirSpeed

    def GroundSpeed(self):
        self.ProcessIncoming()
        return self.ground_speed

    def ClimbRate(self):
        self.ProcessIncoming()
        return self.ClimbRate

    def Position(self):
        self.ProcessIncoming()
        return (self.longitude, self.latitude)

    def HeadingRateChange(self):
        self.ProcessIncoming()
        return self.HeadingRate

    def TrueHeading(self):
        self.ProcessIncoming()
        return self.Heading - self.magnetic_variation

    def MagneticDeclination(self):
        self.ProcessIncoming()
        return self.magnetic_variation

    def Time(self):
        return time.time()

    # Actual flight path in true coordinates
    def GroundTrack(self):
        self.ProcessIncoming()
        return self.ground_track

    def WindSpeed(self):
        self.ProcessIncoming()
        return self.WindSpeed

    def WindDirection(self):
        self.ProcessIncoming()
        return self.WindDirection

    def AGL(self):
        self.ProcessIncoming()
        raise RuntimeError("AGL sensor Not implemented")

    def ProcessIncoming(self):
        global control
        newinput = False
        while True:
            rec = self.residual_input + control.arduino.read(1024)
            if (not rec) or rec.find('\n') < 0:
                self.residual_input = rec
                break
            nl = rec.find('\n')
            line = rec[:nl]
            self.residual_input = rec[nl+1:]
            self._parse_input(line)
            newinput = True
        if newinput:
            self.ProcessRawInputs()

    def Battery(self):
        return 100

    def _parse_input(self, line):
        line = line.strip()
        if line.startswith("$S10DOF"):
            fields = line.split(',')
            if fields < 13:
                logger.warning("Invalid sensor string: %s", line)
            else:
                try:
                    self.pressure = float(fields[1])
                except:
                    self.pressure = None
                    logger.debug("10DOF sensors are not reading. Do not fly")
                try:
                    self.temp = float(fields[2])
                except:
                    self.temp = None
                try:
                    self.r_pitch = float(fields[3])
                except:
                    self.r_pitch = None
                try:
                    self.r_roll = float(fields[4])
                except:
                    self.r_roll = None
                try:
                    self.r_yaw = float(fields[5])
                except:
                    self.r_yaw = None
                try:
                    self.a_forward = float(fields[6])
                except:
                    self.a_forward = None
                try:
                    self.a_side = float(fields[7])
                except:
                    self.a_side = None
                try:
                    self.a_up = float(fields[8])
                except:
                    self.a_up = None
                try:
                    self.m_forward = float(fields[9])
                except:
                    self.m_forward = None
                try:
                    self.m_side = float(fields[10])
                except:
                    self.m_side = None
                try:
                    self.m_up = float(fields[11])
                except:
                    self.m_up = None
                self.last_reading_time = self.reading_time
                try:
                    self.reading_time = int(fields[12])
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
        else:
            NMEAParser.ParseNMEAStrings(line, self)

    def ProcessRawInputs(self):
        if not hasattr(self,"m_side"):
            return
        if self.m_side != None and self.m_up != None and self.m_forward != None:
            mag_vector = Spatial.Vector(self.m_side, self.m_up, self.m_forward)
            mag_polar = mag_vector.to_polar(limit_phi = True, robot_coordinates=False)
            self.PreviousHeading = self.Heading
            self.Heading = mag_polar.theta * util.DEG_RAD
        self.PreviousAltitude = self.Altitude
        self.Altitude = self.AltitudeFromPressure(self.pressure)
        if self.last_reading_time > 0 and self.Altitude != None:
            if self.last_reading_time > self.reading_time:
                # time rollover
                timediff = self.last_reading_time - self.reading_time - (2 << 32)
            else:
                timediff = self.reading_time - self.last_reading_time
            # Differentiate some inputs
            if self._altitude_valid:
                current_climb_rate = (self.Altitude - self.PreviousAltitude) / (timediff / 1000.0)
                self.ClimbRate = util.LowPassFilter (current_climb_rate, self.climb_rate_history)
                if len(self.climb_rate_history) >= len(util.LowPassFIR) and self.signal_quality > 0:
                    self.all_sensors_go = True
            current_heading_rate = (self.Heading - self.PreviousHeading) / (timediff / 1000.0)
            self.HeadingRate = util.LowPassFilter (current_heading_rate, self.heading_rate_history)
        else:
            self.ClimbRate = 0.0
            self.HeadingRate = 0.0
        if (self._flight_mode == Globals.FLIGHT_MODE_GROUND or
                (self._flight_mode != Globals.FLIGHT_MODE_AIRBORN and self._vertical)):
            if self.a_up == None or self.a_up == 0.0:
                logger.debug ("Do not Fly. Vertical Accelerometer reading 0")
                self.Pitch = None
                self.Roll = None
                self.Yaw = None
            else:
                self.Pitch = math.atan(self.a_forward / self.a_up) * util.DEG_RAD
                self.Roll = math.atan(self.a_side / self.a_up) * util.DEG_RAD
                self.Yaw = 0.0
            # TODO: Run a ground based correlation on all sensors. Raise an exception of a problem is found
        else:
            self.Pitch += self.r_pitch
            self.Roll += self.r_roll
            self.Yaw = math.atan (self.a_side / (self.a_forward + self.a_up))
            estimated_pitch = math.atan(self.a_forward / self.a_up) * util.DEG_RAD
            # TODO: regress attitude measurements toward estimates
            # TODO: Run a flight based correlation on all sensors. Adjust accuracy estimates if
            #   readings must revert to a secondary sensor.

        # mag heading - mag variation = true heading
        # true heading + mag variation = mag heading
        if hasattr(self,"ground_track") and self.Heading != None:
            crab_angle = self.Heading - (self.ground_track + self.magnetic_variation)
            # wind component / tas = sin(crab_angle)
            # wind component / ground speed = tan(crab_angle)
            # TODO: get wind component from ground based neumometer
            wind_component = math.tan(crab_angle) * self.ground_speed
            tas = wind_component / math.sin(crab_angle)
            self.AirSpeed = self.TasToCas(tas)
        else:
            self.AirSpeed = None

    def TasToCas(self, tas):
        return tas / math.sqrt(
                self.AirDensity (self._sea_level_pressure, self.StandardSeaLevelTemp()) /
                self.AirDensity(self.pressure, self.temp)
               )

    def AltitudeFromPressure(self, pressure):
        if pressure == None:
            return None
        if self._known_altitude == None and self._sea_level_pressure == 0.0:
            if self.signal_quality > 0:
                self._known_altitude = self.gps_altitude
            else:
                return 0.0
        if self._known_altitude != None:
            self._sea_level_pressure = pressure / pow(1.0 - (self._known_altitude * METERS_FOOT /44330.0), 5.255)
            ret = self._known_altitude
            self._known_altitude = None
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
        return self.temp - 1.98 * self.Altitude / 1000.0

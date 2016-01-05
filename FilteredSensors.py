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

import Spatial, util

import RawSensors

logger=logging.getLogger(__name__)

METERS_FOOT = 0.3048
FEET_METER = 1.0 / METERS_FOOT

class FilteredSensors:
    def __init__(self):

        # Input from above
        self.Wind = None
        self._throttle_level = 0.0

        # Magnetic calibration tables: A list of tuple (throttle_level, list (tuple (mag_reading, calibrated_heading)))
        self._mag_cal_tables = list()

        # Readings from GPS
        self.Utc = None
        self.Latitude = list()
        self.Longitude = list()
        self.GpsAltitude = list()
        self.GroundSpeed = list()
        self.GroundTrack = list()
        self.ClimbRateEstimate = list()
        self.MagneticVariation = 0.0
        self.HaveNewGroundTrack = False
        self.HaveNewPosition = False

        # Readings from 10DOF sensors
        self.A_forward = list()
        self.A_side = list()
        self.A_up = list()
        self.R_pitch = list()
        self.R_roll = list()
        self.R_yaw = list()
        self.ReadingTime = list()

        # Computed from 10DOF sensors
        self.ClimbRate = list()
        self.HeadingRate = list()
        self.Altitude = list()
        self.Heading = list()
        self.Pitch = list()
        self.Roll = list()
        self.Yaw = list()
        self.TrackRate = list()      # Heading rate change from GPS
        self.PitchEstimate = list()
        self.RollEstimate = list()
        self.RollRateEstimate = list()
        self.PitchRateEstimate = list()
        self.GroundRollEstimate = list()
        self.GroundRollRateEstimate = list()
        self.AirSpeed = list()
        self.HaveNew10DOF = False

        # Internal state variables
        self._raw_heading_rate = list()
        self._raw_climb_rate = list()
        self._raw_track_rate = list()
        self._raw_roll_rate_estimate = list()
        self._raw_ground_roll_rate_estimate = list()
        self._raw_pitch_rate_estimate = list()
        self._sea_level_pressure = 101.325      # 1 Standard atm
        self._altitude_valid = False
        self._known_altitude = None
        self._gps_altitude_variance = None
        self._filter_size = len(util.LowPassFIR)
        self.GpsUpdatePeriod = 1.0      # in seconds
        self.GyroUpdatePeriod = .01     # in seconds

        # for every degree heading change per second, the degrees of roll for this airplane
        # degree_roll / (second * degree_heading)
        self.TurnCoordinatorFactor = 5.0    # 5 degree roll will make 1 degree / s heading change

    def initialize(self, messenger):
        self._raw_sensors = RawSensors.RawArduinoSensors()
        self._raw_sensors.initialize(messenger)
        self.RateDriftCorrectionFactor = self.GyroUpdatePeriod / 60.0       # 1 degree per minute max drift
        self._have_rmc = False
        self._have_gga = False
        self._have_10dof = False
        self._have_differentials = False
        while not (self._have_rmc and self._have_gga and self._have_10dof and self._have_differentials):
            self.Update()
            time.sleep(self.GyroUpdatePeriod)

    def Update(self):
        self._raw_sensors.ProcessIncoming()
        self.ProcessRawInputs()

    def KnownAltitude(self, alt):
        self._known_altitude = alt

    def KnownMagneticVariation(self, v):
        self.MagneticVariation = v

    def WindVector(self, v):
        self.Wind = v

    def ThrottleLevel (self, l):
        self._throttle_level = l

    def _grab_raw_sensor(self, name, history_size = -1, local_name=None):
        reading = getattr(self._raw_sensors, name)
        if not local_name:
            local_name = name
        val = getattr(self, local_name)
        assert(val != None)
        if reading != None:
            val.insert (0, reading)
            if history_size == -1:
                history_size = self._filter_size
            if len(val) > history_size:
                del val[-1]
        else:
            setattr(self, local_name, list())

    def _insert_computed_sensor (self, name, val, history_size = -1):
        c = getattr(self, name)
        c.insert(0, val)
        if history_size == -1:
            history_size = self._filter_size
        if len(c) > history_size:
            del c[-1]

    def ProcessRawInputs(self):
        if self._raw_sensors.M_side == None: # No sensor input yet
            return

        if self._raw_sensors.RMCUpdate:
            self._grab_raw_sensor ("GroundTrack")
            self._grab_raw_sensor ("GroundSpeed")
            self._raw_sensors.RMCUpdate = False
            self.HaveNewGroundTrack = True
            self._have_rmc = True
        if self._raw_sensors.GGAUpdate:
            self._grab_raw_sensor ("Latitude")
            self._grab_raw_sensor ("Longitude")
            if self._raw_sensors.Utc != None:
                self.Utc = self._raw_sensors.Utc
            self._grab_raw_sensor ("GpsAltitude")
            if self._raw_sensors.GpsAltitude != None:
                var = 0.0 if self._gps_altitude_variance == None else self._gps_altitude_variance
                self.GpsAltitude[0] = self._raw_sensors.GpsAltitude + var
            else:
                self.GpsAltitude = list()
                self.ClimbRateEstimate = list()
            if len(self.GpsAltitude) > 1:
                diff = self.GpsAltitude[0] - self.GpsAltitude[1]
                self._insert_computed_sensor ("ClimbRateEstimate", diff * self.GpsUpdatePeriod)

            self._raw_sensors.GGAUpdate = False
            self.HaveNewPosition = True
            self._have_gga = True
        if self._raw_sensors.HaveNew10DOF:
            self.HaveNew10DOF = True
            self._raw_sensors.HaveNew10DOF = False
            self._have_10dof = True

            self._grab_raw_sensor ("R_yaw", 20)
            self._grab_raw_sensor ("R_roll", 20)
            self._grab_raw_sensor ("R_pitch", 20)
            self._grab_raw_sensor ("A_up", 20)
            self._grab_raw_sensor ("A_forward", 20)
            self._grab_raw_sensor ("A_side", 20)
            self._grab_raw_sensor ("ReadingTime", 2)

            if len(self.ReadingTime) > 1:
                if self.ReadingTime[1] > self.ReadingTime[0]:
                    # time rollover
                    timediff = self.ReadingTime[1] + ((2 << 32) - self.ReadingTime[0])
                else:
                    timediff = self.ReadingTime[0] - self.ReadingTime[1]
                timediff = float(timediff)
                timediff /= 1000.0
            else:
                timediff = 0
            if len(self.GroundTrack) > 1:
                current_track_rate = (self.GroundTrack[0] - self.GroundTrack[1]) / self.GpsUpdatePeriod
                self._insert_computed_sensor ("TrackRate",
                    util.LowPassFilter (current_track_rate, self._raw_track_rate))
            else:
                self.TrackRate = list()

            mag_vector = Spatial.Vector(self._raw_sensors.M_side,
                    self._raw_sensors.M_up, self._raw_sensors.M_forward)
            mag_polar = mag_vector.to_polar(limit_phi = True, robot_coordinates=False)
            mag_heading = self._calibrated_heading (mag_polar.theta * util.DEG_RAD)
            self._insert_computed_sensor ("Heading", mag_heading, self._filter_size)
            if self._raw_sensors.Pressure != None:
                self._insert_computed_sensor ("Altitude",
                        self.AltitudeFromPressure(self._raw_sensors.Pressure),
                        100)
                if len(self.Altitude) > 1 and timediff > 0:
                    # Differentiate some inputs
                    if self._altitude_valid:
                        current_climb_rate = (self.Altitude[0] - self.Altitude[1]) / (timediff / 60.0)
                        self._insert_computed_sensor ("ClimbRate",
                                util.LowPassFilter (current_climb_rate, self._raw_climb_rate))
                    current_heading_rate = (self.Heading[0] - self.Heading[1]) / timediff
                    self._insert_computed_sensor ("HeadingRate",
                        util.LowPassFilter (current_heading_rate, self._raw_heading_rate))
                    self._have_differentials = True
            else:
                # Reset all history in case data is lost
                self.ClimbRate = list()
                self._raw_climb_rate = list()
                self.HeadingRate = list()
                self._raw_heading_rate = list()
                self.Altitude = list()
                self.Heading = list()
            if self._raw_sensors.A_up == None or self._raw_sensors.A_up == 0.0 or self._raw_sensors.R_pitch == None:
                logger.debug ("Do not Fly. Invalid readings from raw sensors")
                self.Pitch = list()
                self.Roll = list()
                self.Yaw = list()
                self.PitchEstimate = list()
                self.RollEstimate = list()
                self.RollRateEstimate = list()
                self.PitchRateEstimate = list()
                self.GroundRollEstimate = list()
                self.GroundRollRateEstimate = list()
            else:
                if timediff > 0:
                    self._insert_computed_sensor ("PitchEstimate",
                            math.atan(self.A_forward[0] / self.A_up[0]) * util.DEG_RAD)
                    self._insert_computed_sensor ("GroundRollEstimate",
                            math.atan(self.A_side[0] / self.A_up[0]) * util.DEG_RAD)
                    self._insert_computed_sensor ("Yaw",
                            math.atan (self.A_side[0] / (self.A_forward[0] + self.A_up[0])))
                    if len(self.Pitch):
                        self._insert_computed_sensor ("Pitch", self.Pitch[0])
                    else:
                        self.Pitch = [self.PitchEstimate[0]]
                    self.Pitch[0] += (self._raw_sensors.R_pitch * timediff -
                            (self.PitchEstimate[0] - self.Pitch[0]) * self.RateDriftCorrectionFactor)
                    if len(self.HeadingRate):
                        self._insert_computed_sensor ("RollEstimate",
                                self.HeadingRate[0] * self.TurnCoordinatorFactor)
                    if len(self.Roll):
                        self._insert_computed_sensor ("Roll", self.Roll[0])
                    else:
                        self.Roll = [self.GroundRollEstimate[0]]
                    if len(self.RollEstimate):
                        self.Roll[0] += (self._raw_sensors.R_roll * timediff -
                                (self.RollEstimate[0] - self.Roll[0]) * self.RateDriftCorrectionFactor)
                    if len(self.RollEstimate) > 1:
                        roll_rate_estimate = (self.RollEstimate[0] - self.RollEstimate[1]) / timediff
                        self._insert_computed_sensor ("RollRateEstimate",
                                util.LowPassFilter (roll_rate_estimate, self._raw_roll_rate_estimate))
                        ground_roll_rate_estimate = (self.GroundRollEstimate[0] - self.GroundRollEstimate[1]) / timediff
                        self._insert_computed_sensor ("GroundRollRateEstimate",
                                util.LowPassFilter (ground_roll_rate_estimate, self._raw_ground_roll_rate_estimate))
                        pitch_rate_estimate = (self.PitchEstimate[0] - self.PitchEstimate[1]) / timediff
                        self._insert_computed_sensor ("PitchRateEstimate",
                                util.LowPassFilter (pitch_rate_estimate, self._raw_pitch_rate_estimate))


            # mag heading - mag variation = true heading
            # true heading + mag variation = mag heading
            if len(self.GroundTrack) and len(self.Heading) and len(self.Altitude):
                if self._raw_sensors.MagneticVariation != 0.0:      # Let GPS's opinion of magnetic variation trump
                    self.MagneticVariation = self._raw_sensors.MagneticVariation
                if  self.Wind == None:
                    logger.error ("Cannot compute airspeed because wind is not known")
                else:
                    airvector = Spatial.Vector (
                            math.sin(self.GroundTrack[0] * util.RAD_DEG) * self.GroundSpeed[0],
                            math.cos(self.GroundTrack[0] * util.RAD_DEG) * self.GroundSpeed[0], 0)
                    airvector.sub (self.Wind)

                    tas = airvector.norm()
                    self._insert_computed_sensor ("AirSpeed", self.TasToCas(tas))
            else:
                self.AirSpeed = list()

    def TasToCas(self, tas):
        return tas / math.sqrt(
                self.AirDensity (self._sea_level_pressure, self.StandardSeaLevelTemp()) /
                self.AirDensity(self._raw_sensors.Pressure, self._raw_sensors.Temp)
               )

    def AltitudeFromPressure(self, pressure):
        if self._known_altitude != None:
            self._sea_level_pressure = pressure / pow(1.0 - (self._known_altitude * METERS_FOOT /44330.0), 5.255)
            ret = self._known_altitude
            if len(self.GpsAltitude):
                self._gps_altitude_variance = self._known_altitude - self.GpsAltitude[0]
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
        return self._raw_sensors.Temp - 1.98 * self.Altitude[0] / 1000.0

    def _calibrated_heading(self, heading):
        # Find the bracketing throttle tables
        # In each table, estimate the calibrated heading
        # estimate calibrated heading PWL from 2 throttle brackets
        # for test:
        return heading
        raise RuntimeError("Magnetic calibration unimplemented")

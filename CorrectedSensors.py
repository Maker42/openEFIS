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

import Globals, util

import FilteredSensors

logger=logging.getLogger(__name__)

NO_CONFIDENCE = -9999

class SensorReading:
    def __init__(self, name, source):
        self.Value = None
        self.Confidence = NO_CONFIDENCE
        self._logged = False
        self.Source = source
        self.Name = name

class CorrectedSensors:
    def __init__(self):
        self._flight_mode = Globals.FLIGHT_MODE_GROUND
        self._vertical = True
        self._turn_rate_factor = 0.2
        self.GpsUpdatePeriod = 1.0      # in seconds
        self.GyroUpdatePeriod = .01     # in seconds
        self.MagneticVariation = 0.0

        # Input from flight control for helping to decide what sensors to believe
        self._desired_climb_rate = 0.0
        self._desired_heading = 0.0
        self._desired_pitch = 0.0
        self._desired_roll = 0.0
        self.MaxAirSpeed = 200.0

        # Public output variables
        self.Readings = dict()
        self.DistrustedSensors = set()
        
        self._heading = [SensorReading("Heading", "Magnetic Sensor"),
                         SensorReading("Heading", "GPS")]
        self._heading_rate = [SensorReading("Heading Rate", "Gyro"),
                              SensorReading("Heading Rate", "Magnetic Sensor")]
        self._pitch = [SensorReading("Pitch", "Gyro"), SensorReading("Pitch", "Accelerometers")]
        self._pitch_rate = [SensorReading("Pitch Rate", "Gyro"),
                            SensorReading("Pitch Rate", "Accelerometers")]
        self._roll = [SensorReading("Roll", "Gyro"), SensorReading("Roll", "MagneticSensor")]
        self._roll_rate = [SensorReading("Roll Rate", "Gyro"),
                           SensorReading("Roll Rate", "MagneticSensor")]
        self._yaw = [SensorReading("Yaw", "Accelerometer")]
        self._air_speed = [SensorReading("Air Speed", "GPS")]
        self._altitude = [SensorReading("Altitude", "Pressure Sensor"),
                          SensorReading("Altitude", "GPS")]
        self._climb_rate = [SensorReading("Climb Rate", "Pressure Sensor"),
                            SensorReading("Climb Rate", "GPS")]
        self._position = [SensorReading("Position", "GPS")]
        self._ground_speed = [SensorReading("Ground Speed", "GPS")]
        self._ground_track = [SensorReading("Ground Track", "GPS")]

        self.HeadingRateConfidenceDivider = 10.0
        self.HeadingConfidenceDivider = 10.0
        self.PitchConfidenceDivider = 10.0
        self.PitchRateConfidenceDivider = 10.0
        self.RollConfidenceDivider = 10.0
        self.RollRateConfidenceDivider = 10.0
        self.AltitudeConfidenceDivider = 10.0
        self.ClimbRateConfidenceDivider = 10.0
        self.YawConfidenceDivider = 1.0

    def initialize(self, messenger):
        self._filtered_sensors = FilteredSensors.FilteredSensors()
        self._filtered_sensors.initialize(messenger)
        self.Update()

    def Update(self):
        self._filtered_sensors.Update()
        self.CorrectSensors()

    def FlightMode(self, mode, vertical = True):
        self._flight_mode = mode
        self._vertical = vertical
        if self._flight_mode != Globals.FLIGHT_MODE_GROUND:
            self._filtered_sensors.KnownAltitude(None)

    def KnownAltitude(self, alt):
        self._filtered_sensors.KnownAltitude(alt)

    def WindVector(self, v):
        self._filtered_sensors.WindVector(v)

    def FlightParameters(self, pitch, roll, heading, climb_rate):
        self._desired_heading = heading
        self._desired_climb_rate = climb_rate
        self._desired_pitch = pitch
        self._desired_roll = roll

    def ThrottleLevel (self, l):
        self._filtered_sensors.ThrottleLevel(l)

    def KnownMagneticVariation(self, v):
        self.MagneticVariation = v
        self._filtered_sensors.KnownMagneticVariation(v)

    def CorrectSensors(self):
        # Compare Pitch with TurnRate and HeadingRate
        if self._filtered_sensors.HaveNewGroundTrack:
            self.EvaluateGroundSpeed()
            self.EvaluateGroundTrack()
        if self._filtered_sensors.HaveNewPosition:
            self.EvaluatePosition()
        if self._filtered_sensors.HaveNew10DOF:
            self.EvaluateHeadingRate()
            self.EvaluateHeading()
            self.EvaluatePitch()
            self.EvaluatePitchRate()
            self.EvaluateRoll()
            self.EvaluateRollRate()
            self.EvaluateYaw()
            self.EvaluateAirSpeed()
            self.EvaluateAltitude()
            self.EvaluateClimbRate()
        self._filtered_sensors.HaveNew10DOF = False
        self._filtered_sensors.HaveNewPosition = False
        self._filtered_sensors.HaveNewGroundTrack = False

    def EvaluatePosition(self):
        PositionFlags = None
        if len(self._filtered_sensors.Latitude) > 1 and len(self._filtered_sensors.Longitude) > 1:
            self._position[0].Value = (self._filtered_sensors.Longitude[0],
                                       self._filtered_sensors.Latitude[0])
            heading,distance,_ = util.TrueHeadingAndDistance(
                    ((self._filtered_sensors.Longitude[0], self._filtered_sensors.Latitude[0]), 
                     (self._filtered_sensors.Longitude[-1], self._filtered_sensors.Latitude[-1])))
            avg_speed = distance / (len(self._filtered_sensors.Latitude) * self.GpsUpdatePeriod)
            if avg_speed / 2 > self.MaxAirSpeed:
                PositionFlags = "Nonsense"
            else:
                self._position[0].Confidence = 10
                PositionFlags = None
        else:
            self._position[0].Confidence = NO_CONFIDENCE
            PositionFlags = "Inop"
        self.Readings['Position'] = self._best_confidence (self._position, PositionFlags)

    def EvaluateGroundSpeed(self):
        GroundSpeedFlags = None
        if len(self._filtered_sensors.GroundSpeed):
            self._ground_speed[0].Value = self._filtered_sensors.GroundSpeed[0]
            if (len(self._filtered_sensors.Latitude) > 1 and len(self._filtered_sensors.Longitude) > 1
                    and self._position[0].Confidence != NO_CONFIDENCE):
                heading,distance,_ = util.TrueHeadingAndDistance(
                    ((self._filtered_sensors.Longitude[0], self._filtered_sensors.Latitude[0]), 
                     (self._filtered_sensors.Longitude[-1], self._filtered_sensors.Latitude[-1])))
                avg_speed = distance / (len(self._filtered_sensors.Latitude) * self.GpsUpdatePeriod)
                if avg_speed > 0:
                    if (abs(avg_speed - self._filtered_sensors.GroundSpeed[0]) / avg_speed > .5 or
                            self._filtered_sensors.GroundSpeed[0] / 2 > self.MaxAirSpeed):
                        self._ground_speed[0].Confidence = NO_CONFIDENCE
                        GroundSpeedFlags = "Nonsense"
                    else:
                        self._ground_speed[0].Confidence = 10
                        GroundSpeedFlags = None
            else:
                self._ground_speed[0].Confidence = 0
                GroundSpeedFlags = "Unverified"
        else:
            self._ground_speed[0].Confidence = NO_CONFIDENCE
            GroundSpeedFlags = "Inop"
        self.Readings['GroundSpeed'] = self._best_confidence (self._ground_speed, GroundSpeedFlags)

    def EvaluateGroundTrack(self):
        GroundTrackFlags = None
        if len(self._filtered_sensors.GroundTrack):
            self._ground_track[0].Value = self._filtered_sensors.GroundTrack[0]
            if (len(self._filtered_sensors.Latitude) > 1 and len(self._filtered_sensors.Longitude) > 1
                    and self._position[0].Confidence != NO_CONFIDENCE):
                heading,distance,_ = util.TrueHeadingAndDistance(
                    ((self._filtered_sensors.Longitude[0], self._filtered_sensors.Latitude[0]), 
                     (self._filtered_sensors.Longitude[1], self._filtered_sensors.Latitude[1])))
                diff = abs(self._filtered_sensors.GroundTrack[0] - heading)
                if diff > 180:
                    diff -= 360
                    diff = abs(diff)
                if (diff > 30):
                    self._ground_track[0].Confidence = NO_CONFIDENCE
                    GroundTrackFlags = "Nonsense"
                else:
                    self._ground_track[0].Confidence = 10
                    GroundTrackFlags = None
            else:
                self._ground_track[0].Confidence = 0
                GroundTrackFlags = "Unverified"
        else:
            self._ground_track[0].Confidence = NO_CONFIDENCE
            GroundTrackFlags = "Inop"
        self.Readings['GroundTrack'] = self._best_confidence (self._ground_track, GroundTrackFlags)

    def EvaluateHeadingRate(self):
        if len(self._filtered_sensors.R_yaw) > 1:
            desired_heading_rate = self._filtered_sensors.R_yaw[1] - self._desired_heading
            # Handle wrap-around
            if desired_heading_rate > 180:
                desired_heading_rate -= 360
            elif desired_heading_rate < -180:
                desired_heading_rate += 360
            # Clamp
            if desired_heading_rate > 30:
                desired_heading_rate = 30
            elif desired_heading_rate < -30:
                desired_heading_rate = -30
        else:
            desired_heading_rate = 0.0
        HeadingFlags = None
        if len(self._filtered_sensors.R_yaw):
            self._heading_rate[0].Value = self._filtered_sensors.R_yaw[0]
            self._heading_rate[0].Confidence = 10.0 - (
                abs(self._filtered_sensors.R_yaw[0] - desired_heading_rate) /
                self.HeadingRateConfidenceDivider)
            HeadingFlags = None
        else:
            self._heading_rate[0].Confidence = NO_CONFIDENCE
            HeadingFlags = "Inop"
        if len(self._filtered_sensors.HeadingRate):
            self._heading_rate[1].Value = self._filtered_sensors.HeadingRate[0]
            self._heading_rate[1].Confidence = 6.0 - (
                abs(self._filtered_sensors.HeadingRate[0] - desired_heading_rate) /
                self.HeadingRateConfidenceDivider)
        else:
            self._heading_rate[1].Confidence = NO_CONFIDENCE
        self.Readings['HeadingRate'] = self._best_confidence(self._heading_rate, HeadingFlags)

    def EvaluateHeading(self):
        HeadingFlag = None
        if len(self._filtered_sensors.Heading):
            self._heading[0].Value = self._filtered_sensors.Heading[0]
            self._heading[0].Confidence = 10.0
        else:
            self._heading[0].Confidence = NO_CONFIDENCE
            HeadingFlag = "Inop"
        if len(self._filtered_sensors.GroundTrack):
            self._heading[1].Value = self._filtered_sensors.GroundTrack[0] + self.MagneticVariation
            if len(self._filtered_sensors.GroundTrack) > 1:
                diff = abs(self._desired_heading - self._filtered_sensors.GroundTrack[0])
                if diff > 180:
                    diff = 360 - diff
                self._heading[1].Confidence = 5.0 - (diff / self.HeadingConfidenceDivider)
        else:
            self._heading[1].Confidence = NO_CONFIDENCE

        self.Readings['Heading'] = self._best_confidence (self._heading, HeadingFlag)

    def EvaluatePitch(self):
        PitchFlag = None
        if len(self._filtered_sensors.Pitch):
            self._pitch[0].Value = self._filtered_sensors.Pitch[0]
            diff = abs(self._desired_pitch - self._filtered_sensors.Pitch[0])
            self._pitch[0].Confidence = 10.0 - (diff / self.PitchConfidenceDivider)
        else:
            self._pitch[0].Confidence = NO_CONFIDENCE
            PitchFlag = "Inop"
        if len(self._filtered_sensors.PitchEstimate):
            self._pitch[1].Value = self._filtered_sensors.PitchEstimate[0]
            diff = abs(self._desired_pitch - self._filtered_sensors.PitchEstimate[0])
            self._pitch[1].Confidence = 4.0 - (diff / self.PitchConfidenceDivider)
        else:
            self._pitch[1].Confidence = NO_CONFIDENCE

        self.Readings['Pitch'] = self._best_confidence (self._pitch, PitchFlag)

    def EvaluatePitchRate(self):
        if len(self._filtered_sensors.Pitch) > 1:
            desired_pitch_rate = (self._filtered_sensors.Pitch[1] - self._desired_pitch) / 5.0
        else:
            desired_pitch_rate = 0.0
        PitchRateFlag = None
        if len(self._filtered_sensors.R_pitch):
            self._pitch_rate[0].Value = self._filtered_sensors.R_pitch[0]
            diff = abs(desired_pitch_rate - self._filtered_sensors.R_pitch[0])
            self._pitch_rate[0].Confidence = 10.0 - (diff / self.PitchRateConfidenceDivider)
        else:
            self._pitch_rate[0].Confidence = NO_CONFIDENCE
            PitchRateFlag = "Inop"
        if len(self._filtered_sensors.PitchRateEstimate):
            self._pitch_rate[1].Value = self._filtered_sensors.PitchRateEstimate[0]
            diff = abs(desired_pitch_rate - self._filtered_sensors.PitchRateEstimate[0])
            self._pitch_rate[1].Confidence = 4.0 - (diff / self.PitchRateConfidenceDivider)
        else:
            self._pitch_rate[1].Confidence = NO_CONFIDENCE

        self.Readings['PitchRate'] = self._best_confidence (self._pitch_rate, PitchRateFlag)

    def EvaluateRoll(self):
        RollFlag = None
        if len(self._filtered_sensors.Roll):
            self._roll[0].Value = self._filtered_sensors.Roll[0]
            diff = abs(self._desired_roll - self._filtered_sensors.Roll[0])
            self._roll[0].Confidence = 10.0 - (diff / self.RollConfidenceDivider)
        else:
            self._roll[0].Confidence = NO_CONFIDENCE
            RollFlag = "Inop"
        if (self._flight_mode == Globals.FLIGHT_MODE_GROUND or
                (self._flight_mode != Globals.FLIGHT_MODE_AIRBORN and self._vertical)):
            if len(self._filtered_sensors.GroundRollEstimate):
                self._roll[1].Value = self._filtered_sensors.GroundRollEstimate[0]
                diff = abs(self._desired_roll - self._filtered_sensors.GroundRollEstimate[0])
                self._roll[1].Confidence = 4.0 - (diff / self.RollConfidenceDivider)
            else:
                self._roll[1].Confidence = NO_CONFIDENCE
        else:
            if len(self._filtered_sensors.RollEstimate):
                self._roll[1].Value = self._filtered_sensors.RollEstimate[0]
                diff = abs(self._desired_roll - self._filtered_sensors.RollEstimate[0])
                self._roll[1].Confidence = 4.0 - (diff / self.RollConfidenceDivider)
            else:
                self._roll[1].Confidence = NO_CONFIDENCE

        self.Readings['Roll'] = self._best_confidence (self._roll, RollFlag)

    def EvaluateRollRate(self):
        if len(self._filtered_sensors.Roll):
            desired_roll_rate = (self._filtered_sensors.Roll[0] - self._desired_roll) / 5.0
        else:
            desired_roll_rate = 0.0
        RollRateFlag = None
        if len(self._filtered_sensors.R_roll):
            self._roll_rate[0].Value = self._filtered_sensors.R_roll[0]
            diff = abs(desired_roll_rate - self._filtered_sensors.R_roll[0])
            self._roll_rate[0].Confidence = 10.0 - (diff / self.RollRateConfidenceDivider)
        else:
            self._roll_rate[0].Confidence = NO_CONFIDENCE
            RollRateFlag = "Inop"
        if (self._flight_mode == Globals.FLIGHT_MODE_GROUND or
                (self._flight_mode != Globals.FLIGHT_MODE_AIRBORN and self._vertical)):
            if len(self._filtered_sensors.GroundRollRateEstimate):
                self._roll_rate[1].Value = self._filtered_sensors.GroundRollRateEstimate[0]
                diff = abs(desired_roll_rate - self._filtered_sensors.GroundRollRateEstimate[0])
                self._roll_rate[1].Confidence = 4.0 - (diff / self.RollRateConfidenceDivider)
            else:
                self._roll_rate[1].Confidence = NO_CONFIDENCE
        else:
            if len(self._filtered_sensors.RollRateEstimate):
                self._roll_rate[1].Value = self._filtered_sensors.RollRateEstimate[0]
                diff = abs(desired_roll_rate - self._filtered_sensors.RollRateEstimate[0])
                self._roll_rate[1].Confidence = 4.0 - (diff / self.RollRateConfidenceDivider)
            else:
                self._roll_rate[1].Confidence = NO_CONFIDENCE

        self.Readings['RollRate'] = self._best_confidence (self._roll_rate, RollRateFlag)

    def EvaluateYaw(self):
        YawFlag = None
        if len(self._filtered_sensors.Yaw):
            self._yaw[0].Value = self._filtered_sensors.Yaw[0]
            if self._filtered_sensors.Yaw[0] > 60 or self._filtered_sensors.Yaw[0] < -60:
                self._yaw[0].Confidence = NO_CONFIDENCE
                YawFlag = "Nonsense"
            else:
                self._yaw[0].Confidence = 10.0 - abs(self._filtered_sensors.Yaw[0]) / self.YawConfidenceDivider
        else:
            self._yaw[0].Confidence = NO_CONFIDENCE
            YawFlag = "Inop"

        self.Readings['Yaw'] = self._best_confidence (self._yaw, YawFlag)

    def EvaluateAirSpeed(self):
        AirSpeedFlags = None
        if len(self._filtered_sensors.AirSpeed):
            if abs(self._filtered_sensors.AirSpeed[0] / 2) > self.MaxAirSpeed:
                self._air_speed[0].Confidence = NO_CONFIDENCE
                AirSpeedFlags = "Nonsense"
            else:
                self._air_speed[0].Confidence = 10
                AirSpeedFlags = None
        else:
            self._air_speed[0].Confidence = NO_CONFIDENCE
            AirSpeedFlags = "Inop"
        self.Readings['AirSpeed']= self._best_confidence (self._air_speed, AirSpeedFlags)

    def EvaluateAltitude(self):
        AltitudeFlag = None
        if len(self._filtered_sensors.Altitude):
            timediff = len(self._filtered_sensors.Altitude) * self.GyroUpdatePeriod
            estimated_altitude = self._filtered_sensors.Altitude[-1] + (
                    self._desired_climb_rate * timediff / 60.0)
            self._altitude[0].Value = self._filtered_sensors.Altitude[0]
            diff = abs(estimated_altitude - self._filtered_sensors.Altitude[0])
            self._altitude[0].Confidence = 10.0 - (diff / self.AltitudeConfidenceDivider)
        else:
            self._altitude[0].Confidence = NO_CONFIDENCE
            AltitudeFlag = "Inop"
        if len(self._filtered_sensors.GpsAltitude):
            timediff = len(self._filtered_sensors.GpsAltitude) * self.GpsUpdatePeriod
            estimated_altitude = self._filtered_sensors.GpsAltitude[-1] + (
                    self._desired_climb_rate * timediff / 60.0)
            self._altitude[1].Value = self._filtered_sensors.GpsAltitude[0]
            diff = abs(estimated_altitude - self._filtered_sensors.GpsAltitude[0])
            self._altitude[1].Confidence = 6.0 - (diff / self.AltitudeConfidenceDivider)
        else:
            self._altitude[1].Confidence = NO_CONFIDENCE

        self.Readings['Altitude'] = self._best_confidence (self._altitude, AltitudeFlag)


    def EvaluateClimbRate(self):
        ClimbRateFlag = None
        if len(self._filtered_sensors.ClimbRate):
            self._climb_rate[0].Value = self._filtered_sensors.ClimbRate[0]
            diff = abs(self._desired_climb_rate - self._filtered_sensors.ClimbRate[0])
            self._climb_rate[0].Confidence = 10.0 - (diff / self.ClimbRateConfidenceDivider)
        else:
            self._climb_rate[0].Confidence = NO_CONFIDENCE
            ClimbRateFlag = "Inop"
        if len(self._filtered_sensors.ClimbRateEstimate):
            self._climb_rate[1].Value = self._filtered_sensors.ClimbRateEstimate[0]
            diff = abs(self._desired_climb_rate - self._filtered_sensors.ClimbRateEstimate[0])
            self._climb_rate[1].Confidence = 4.0 - (diff / self.ClimbRateConfidenceDivider)
        else:
            self._climb_rate[1].Confidence = NO_CONFIDENCE

        self.Readings['ClimbRate'] = self._best_confidence (self._climb_rate, ClimbRateFlag)

    def _best_confidence(self, l, preflag):
        best_index = 0
        warning = preflag
        for i in range(1,len(l)):
            if l[i].Confidence > l[best_index].Confidence or l[best_index].Source in self.DistrustedSensors:
                self._downgrade_sensors (l[best_index].Source)
                best_index = i
        if best_index > 0:
            warning = "Confidence lost in primary source (%s). Reverting to %s."%(
                    l[0].Source, l[best_index].Source)
            if not l[0]._logged:
                logger.warning(warning)
                logger.debug ("Confidence %g/%g, distrusted = %s", l[0].Confidence, l[best_index].Confidence, l[0].Source in self.DistrustedSensors)
                l[0]._logged = True
        if l[best_index].Source in self.DistrustedSensors or l[best_index].Confidence == NO_CONFIDENCE:
            warning = "Source (%s) is distrusted. No fall-back."%(l[0].Source)
            if not l[best_index]._logged:
                logger.warning(warning)
                l[best_index]._logged = True
            l[best_index].Value = None
            self._downgrade_sensors(l[best_index].Source)
        if l[best_index].Confidence == NO_CONFIDENCE:
            return None,warning
        else:
            return l[best_index].Value,warning

    def _downgrade_sensors(self, source):
        self.DistrustedSensors.add (source)

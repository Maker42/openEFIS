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

import time, logging, math, copy

import Spatial
import PID
import util

logger=logging.getLogger(__name__)

NAUTICAL_MILES_PER_DEGREE = 60.0
FEET_PER_NAUTICAL_MILE = 6076.12

def UpdatePosition(pos, _time, direction, speed):
    dist = speed * _time
    logger.debug("Position Update: %g degrees, %g feet", dist,
            dist * NAUTICAL_MILES_PER_DEGREE * FEET_PER_NAUTICAL_MILE)

    pos.x += dist * math.sin (direction * util.RAD_DEG)
    pos.y += dist * math.cos (direction * util.RAD_DEG)

class AttitudeVTOLEstimation:
    def __init__(self):
        self.last_position = None
        self.last_time = 0
        self.position_offset = Spatial.Vector()
        self.next_change_time = 0
        self.desired_pitch = 0.0
        self.desired_roll = 0.0
        self._last_velocity = Spatial.Vector()
        self.xpid = None
        self.ypid = None
        self.AccelerationFIR = [0.45, 0.2, 0.1, 0.05, 0.05, .05, .05, .05]
        self._last_acceleration_x = list()
        self._last_acceleration_y = list()

        self.JournalFileName = ''
        self._journal_file = None
        self._journal_flush_count = 0

    def initialize(self, tuning_params, sample_period, limits, ms):
        kp,ki,kd = tuning_params
        self.xpid = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)
        self.xpid.SetSampleTime (sample_period)
        self.ypid = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)
        self.ypid.SetSampleTime (sample_period)
        mn,mx = limits
        self.xpid.SetOutputLimits (mn, mx)
        self.ypid.SetOutputLimits (mn, mx)

    def Start(self, journal_file=None):
        if journal_file:
            self.JournalFileName = journal_file
        if self.JournalFileName and (not self._journal_file):
            self._journal_file = open(self.JournalFileName, 'w+')
            if self._journal_file:
                self._journal_file.write("Time,ErrorDistance,Speed.x,Speed.y,Desired.x,Desired.y,CA.x,CA.y,SP.x*10,SP.y*10,DesiredPitch,DesiredRoll,ActualPitch,ActualRoll\n")
        self.xpid.SetMode (PID.AUTOMATIC, 0.0, 0.0)
        self.ypid.SetMode (PID.AUTOMATIC, 0.0, 0.0)

    def Stop(self):
        if self._journal_file:
            self._journal_file.close()
        self.xpid.SetMode (PID.MANUAL, 0.0, 0.0)
        self.ypid.SetMode (PID.MANUAL, 0.0, 0.0)

    def Reset(self):
        self.__init__()

    # Returns recommended pitch and roll
    def EstimateNextAttitude(self,
            desired_position,       # Where to stay or move to (2-tuple of lng,lat)
            CorrectionCurve,
            sensors):               # Reference to the sensors object

        _time = sensors.Time()

        current_speed = sensors.GroundSpeed()
        ground_track = sensors.GroundTrack()
        current_velocity = Spatial.Vector()
        # y is North component (positive latitude)
        current_velocity.y = current_speed * math.cos(ground_track*util.RAD_DEG)
        # x is East Component (positive longitude)
        current_velocity.x = current_speed * math.sin(ground_track*util.RAD_DEG)
        # velocity units: knots
        current_position = sensors.Position()
        if current_position != self.last_position:
            self.last_position = current_position
            self.position_offset.x = 0
            self.position_offset.y = 0
        else:
            time_delta = _time - self.last_time
            if time_delta > 0.0:
                # Convert knots (nautical miles per hour) to globe degrees per second:
                # 60 nautical miles per degree, 3600 seconds per hour
                UpdatePosition(self.position_offset, time_delta, ground_track, current_speed / (NAUTICAL_MILES_PER_DEGREE * 3600))
            current_position = (current_position[0] + self.position_offset.x, current_position[1] + self.position_offset.y)
        deltat = _time - self.last_time

        logger.debug("current_velocity %g(%g) ==> %g,%g", current_speed, ground_track,
                current_velocity.x, current_velocity.y)

        desired_heading,distance,_ = util.TrueHeadingAndDistance ([current_position, desired_position])
        # Units nautical miles (nm) per hour (knots)
        # Distance units given in globe degrees (which contain 60 nm)
        desired_groundspeed = util.rate_curve(distance * FEET_PER_NAUTICAL_MILE, CorrectionCurve)
        # desired_groundspeed in knots
        desired_velocity = Spatial.Vector()
        desired_velocity.y = desired_groundspeed * math.cos(desired_heading * util.RAD_DEG)
        desired_velocity.x = desired_groundspeed * math.sin(desired_heading * util.RAD_DEG)
        logger.debug("current_position = %g,%g, desired_position = %g,%g, Desired Velocity: %g(%g) ==> %g,%g",
                current_position[0], current_position[1],
                desired_position[0], desired_position[1],
                desired_groundspeed,
                desired_heading,
                desired_velocity.x, desired_velocity.y)

        delta_vel = copy.copy(desired_velocity)
        delta_vel.sub (current_velocity)
        current_acceleration = copy.copy(current_velocity)
        if self.last_time == 0 or deltat == 0.0:
            current_acceleration.x = 0.0
            current_acceleration.y = 0.0
        else:
            current_acceleration.sub(self._last_velocity)
            current_acceleration.div(deltat)
        self._last_velocity = current_velocity
        current_acceleration.x = util.FIRFilter (current_acceleration.x,
                self._last_acceleration_x, self.AccelerationFIR)
        current_acceleration.y = util.FIRFilter (current_acceleration.y,
                self._last_acceleration_y, self.AccelerationFIR)

        desired_accel = Spatial.Vector()
        ms = util.millis(_time)
        self.xpid.SetSetPoint(delta_vel.x)
        desired_accel.x = self.xpid.Compute(current_acceleration.x, ms)
        self.ypid.SetSetPoint(delta_vel.y)
        desired_accel.y = self.ypid.Compute(current_acceleration.y, ms)

        dvnorm = desired_accel.norm()
        if dvnorm > .001:
            desired_thrust_direction = util.atan_globe(desired_accel.x, desired_accel.y)
            relative_vector = dvnorm
            current_true_heading = sensors.TrueHeading()
            desired_relative_thrust = desired_thrust_direction - current_true_heading * util.RAD_DEG
            forward_vector = math.cos(desired_relative_thrust) * relative_vector
            side_vector = math.sin(desired_relative_thrust) * relative_vector
            # tan(-pitch) = forward_vector
            self.desired_pitch = -math.atan(forward_vector) * util.DEG_RAD
            self.desired_roll = math.atan(side_vector) * util.DEG_RAD
        else:
            self.desired_pitch = 0.0
            self.desired_roll = 0.0

        if self._journal_file:
            self._journal_file.write("%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n"%(_time,
                    distance * FEET_PER_NAUTICAL_MILE,
                    current_velocity.x, current_velocity.y,
                    desired_velocity.x, desired_velocity.y,
                    current_acceleration.x, current_acceleration.y,
                    desired_accel.x * 10, desired_accel.y * 10,
                    self.desired_pitch, self.desired_roll,
                    sensors.Pitch(),sensors.Roll()))

        self.last_time = _time
        return self.desired_pitch,self.desired_roll

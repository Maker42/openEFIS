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
import util

logger=logging.getLogger(__name__)

NAUTICAL_MILES_PER_DEGREE = 60.0
FEET_PER_NAUTICAL_MILE = 6076.12

def UpdatePosition(pos, _time, direction, speed):
    dist = speed * _time
    logger.debug("Position Update: %g degrees, %g feet", dist, dist * NAUTICAL_MILES_PER_DEGREE * FEET_PER_NAUTICAL_MILE)

    pos.x += dist * math.sin (direction * util.RAD_DEG)
    pos.y += dist * math.cos (direction * util.RAD_DEG)

def UpdateSpeed(speed, _time, direction, acceleration):
    speed.x += acceleration * _time * math.sin (direction * util.RAD_DEG)
    speed.y += acceleration * _time * math.cos (direction * util.RAD_DEG)


PITCH_INCREMENTS=[(.25, 5.0), (0.5, 5.0), (1.0, 5.0), (2.0, 5.0), (3.0, 5.0), (5.0, 5.0)]
# Returns 3 tuple of declination angle, time to stay at that angle, and the direction of declination
def EstimateNextAngle(delta_vel):
    delta_speed = delta_vel.norm()
    if delta_speed != 0:
        delta_vel.div(delta_speed)
        logger.log(9, "Next Angle: delta_vel = %g,%g", delta_vel.x, delta_vel.y)
        for angle,_time in PITCH_INCREMENTS:
            delta_accel = delta_speed / _time
            accel = util.G_IN_KNOTS_PER_SEC * math.sin(angle * util.RAD_DEG)
            if delta_accel <= accel:
                return angle,delta_accel*_time/accel,delta_vel
        else:
            angle,_time = PITCH_INCREMENTS[-1]
            delta_accel = delta_speed / _time
            accel = util.G_IN_KNOTS_PER_SEC * math.sin(angle * util.RAD_DEG)
            return angle,_time,delta_vel
    else:
        return 0.0, 0.0, None


class AttitudeVTOLEstimation:
    def __init__(self):
        self.last_position = None
        self.last_time = 0
        self.position_offset = Spatial.Vector()
        self.next_change_time = 0
        self.desired_pitch = 0.0
        self.desired_roll = 0.0
        self.JournalFileName = ''
        self.AcclerationFIRTap = 0.45
        self._journal_file = None
        self._journal_flush_count = 0
        self._thrust_angle = 0.0
        self._thrust_direction = None
        self._last_deltav = None

    def Start(self, journal_file=None):
        if journal_file:
            self.JournalFileName = journal_file
        if self.JournalFileName and (not self._journal_file):
            self._journal_file = open(self.JournalFileName, 'w+')
            if self._journal_file:
                self._journal_file.write("Time,ErrorDistance,Speed.x,Speed.y,Desired.x,Desired.y,Angle\n")

    def Stop(self):
        if self._journal_file:
            self._journal_file.close()

    def Reset(self):
        self.__init__()

    # Returns recommended pitch and roll
    def EstimateNextAttitude(self,
            desired_position,       # Where to stay or move to (2-tuple of lng,lat)
            CorrectionCurve,
            sensors,                # Reference to the sensors object
            airplane):              # Reference to airplane object

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
        self.last_time = _time

        current_true_heading = sensors.Heading() + sensors.MagneticDeclination()
        vector_time = 1.0
        if self.next_change_time == 0 or _time >= self.next_change_time:
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
            if self._last_deltav != None:
                self._last_deltav.div(self.AcclerationFIRTap)
                adder = self._last_deltav
                self._last_deltav = copy.copy(delta_vel)
                delta_vel.add(adder)

            self._thrust_angle,vector_time,self._thrust_direction = EstimateNextAngle(delta_vel)
            if vector_time >= 1.0:
                self.next_change_time = _time + vector_time
            else:
                logger.debug ("delta_v too small to take action yet. Min vector time = %g", vector_time)
                self.next_change_time = _time + 1.0
                self.desired_pitch = 0.0
                self.desired_roll = 0.0


            if self._journal_file:
                self._journal_file.write("%g,%g,%g,%g,%g,%g,%g\n"%(_time,
                        distance * FEET_PER_NAUTICAL_MILE,
                        current_velocity.x, current_velocity.y,
                        desired_velocity.x, desired_velocity.y,
                        self._thrust_angle))

        if self._thrust_direction != None:
            desired_thrust_direction = util.atan_globe(self._thrust_direction.x, self._thrust_direction.y)
            relative_vector = math.atan(self._thrust_angle * util.RAD_DEG)
            desired_relative_thrust = desired_thrust_direction - current_true_heading * util.RAD_DEG
            forward_vector = math.cos(desired_relative_thrust) * relative_vector
            side_vector = math.sin(desired_relative_thrust) * relative_vector
            # tan(-pitch) = forward_vector
            self.desired_pitch = -math.atan(forward_vector) * util.DEG_RAD
            self.desired_roll = math.atan(side_vector) * util.DEG_RAD
            logger.log (3, "thrust_angle = %g, relative_thrust_theta = %g, pitch = %g, roll = %g, time=%g",
                    self._thrust_angle, desired_relative_thrust * util.DEG_RAD, 
                    self.desired_pitch, self.desired_roll, vector_time)
        else:
            self.desired_pitch = 0.0
            self.desired_roll = 0.0

        return self.desired_pitch,self.desired_roll

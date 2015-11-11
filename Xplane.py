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

import socket, struct, logging, time

control = None

logger=logging.getLogger(__name__)

class XplaneControl:
    def __init__(self, localportno, xplane_host, xplane_port):
        global control
        self.localport = localportno
        self.xplane_host = xplane_host
        self.xplane_port = xplane_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind (("", localportno))
        self.sock.setblocking (0)
        control = self
        self.ServoRange = (-1.0, 1.0)
        self.controls = [
                ("sim/joystick/yoke_pitch_ratio", "YokePitch", (-1.0, 1.0)), #	The deflection of the joystick axis controlling pitch. Use override_joystick or override_joystick_pitch
                ("sim/joystick/yoke_roll_ratio", "YokeRoll", (-1.0, 1.0)),  #	The deflection of the joystick axis controlling roll. Use override_joystick or override_joystick_roll
                ("sim/joystick/yoke_heading_ratio", "Yaw", (-1.0, 1.0)),   #	The deflection of the joystick axis controlling yaw. Use override_joystick or override_joystick_heading
                ("sim/flightmodel/engine/ENGN_thro_override", "Throttle", (0.0, 1.0)),  #	y	ratio	Throttle (per engine) as set by user, 0 = idle, 1 = max
                ]
        self.dref_struct_preamble = struct.pack("5s", "DREF")
        self.dref_struct_body = struct.Struct("f500s")

        self.data_struct_preamble = struct.pack("5s", "DATA")
        self.data_struct_body = struct.Struct("iffffffff")
        self.throttle_index = 26

        override_joystick = self.dref_struct_preamble + self.dref_struct_body.pack(1.0, "sim/operation/override/override_joystick")
        self.sock.sendto (override_joystick, (self.xplane_host, self.xplane_port))

        override_throttle = self.dref_struct_preamble + self.dref_struct_body.pack(1.0, "sim/operation/override/override_throttles")
        self.sock.sendto (override_throttle, (self.xplane_host, self.xplane_port))

        set_zero = [
                        "sim/joystick/joystick_pitch_nullzone",
                        "sim/joystick/joystick_roll_nullzone",
                        "sim/joystick/joystick_heading_nullzone",
                   ]
        for sz in set_zero:
            szc = self.dref_struct_preamble + self.dref_struct_body.pack(0.0, sz)
            self.sock.sendto (szc, (self.xplane_host, self.xplane_port))

                # Other controls that may be of interest:
#sim/joystick/servo_pitch_ratio	float	n	[-1..1]	Servo input for pitch
#sim/joystick/servo_roll_ratio	float	n	[-1..1]	Servo input for roll
#sim/joystick/servo_heading_ratio	float	n	[-1..1]	Servo input for yaw
#sim/joystick/joystick_pitch_nullzone	float	y	ratio	The nullzone size for the pitch axis (as of 940, one null zone serves all 3 axes)
#sim/joystick/joystick_roll_nullzone	float	y	ratio	The nullzone size for the roll axis
#sim/joystick/joystick_heading_nullzone	float	y	ratio	The nullzone size for the heading axis
#sim/joystick/joystick_pitch_center	float	y	ratio	Joystick center for pitch axis
#sim/joystick/joystick_roll_center	float	y	ratio	Joystick center for roll axis
#sim/joystick/joystick_heading_center	float	y	ratio	Joystick center for heading axis
#sim/operation/override/override_joystick	int	y	boolean	Override control of the joystick deflections (overrides stick, yoke, pedals, keys, mouse, and auto-coordination)

    def SetChannel(self, channel, val):
        for retry in range(5):
            if channel < 0 or channel >= len(self.controls):
                raise RuntimeError ("Invalid channel set (%d)"%channel)
            channel_range = self.controls[channel][2]
            channel_range_size = abs(channel_range[1] - channel_range[0])
            scaled_value = (val - self.ServoRange[0]) * channel_range_size / self.servo_range_size + channel_range[0]
            logger.log (3, "Setting channel %s to %g (%g)", self.controls[channel][1], val, scaled_value)
            #print ("Setting channel %s = %g (%g)"%(self.controls[channel][1], val, scaled_value))
            try:
                cmd = self.dref_struct_preamble + self.dref_struct_body.pack(scaled_value, self.controls[channel][0])
                control.sock.sendto(cmd, (self.xplane_host, self.xplane_port))
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

class XplaneSensors:
    def __init__(self):
        self.sensor_suite = [(0, "Altitude", "sim/flightmodel/misc/h_ind"),
                (0, "Heading", "sim/flightmodel/position/mag_psi"),
                (0, "Roll", "sim/flightmodel/position/true_phi"),
                (0, "RollRate", "sim/flightmodel/position/P"),
                (0, "Pitch", "sim/flightmodel/position/true_theta"),
                (0, "PitchRate", "sim/flightmodel/position/Q"),
                (0, "Yaw", "sim/flightmodel/position/beta"),
                (0, "AirSpeed", "sim/flightmodel/position/indicated_airspeed"),
                (0, "GroundSpeed", "sim/flightmodel/position/groundspeed"),
                (0, "ClimbRate", "sim/flightmodel/position/vh_ind_fpm"),
                (0, "Longitude", "sim/flightmodel/position/longitude"),
                (0, "Latitude", "sim/flightmodel/position/latitude"),
                (0, "MagneticDeclination", "sim/flightmodel/position/magnetic_variation"),
                (0, "TrueHeading", "sim/flightmodel/position/true_psi"),
                (0, "SimTime", "sim/time/total_running_time_sec"),
                (0, "hpath", "sim/flightmodel/position/hpath"),
                (0, "beta", "sim/flightmodel/position/beta"),
                ]
        self.previous_readings = [s[0] for s in self.sensor_suite]
        self.SamplesPerSecond = 10
        self.dref_rcv_struct = struct.Struct("If")
        self.preamble_struct = struct.Struct("5s")
        self.dref_request_struct = struct.Struct("II400s")
        self.data_struct_body = struct.Struct("iffffffff")


    def initialize(self, filelines):
        global control
        dref_num = 0
        pre = self.preamble_struct.pack ("RREF")
        for d in self.sensor_suite:
            req = self.dref_request_struct.pack(self.SamplesPerSecond, dref_num, d[2])
            control.sock.sendto(pre + req, (control.xplane_host, control.xplane_port))
            dref_num += 1


# Other data refs that might be of interest:
#sim/cockpit/gyros/psi_ind_vac_pilot_degm	float	y	degrees_magnetic	The indicated magnetic heading on the panel for the pilot's side, vacuum driven
#sim/flightmodel/position/elevation	double	n	meters	The elevation above MSL of the aircraft
#sim/flightmodel/position/theta	float	y	degrees	The pitch relative to the plane normal to the Y axis in degrees - OpenGL coordinates
#sim/flightmodel/position/phi	float	y	degrees	The roll of the aircraft in degrees - OpenGL coordinates
#sim/flightmodel/position/psi	float	y	degrees	The true heading of the aircraft in degrees from the Z axis - OpenGL coordinates
#sim/flightmodel/position/magpsi	float	n	degrees	DO NOT USE THIS
#sim/flightmodel/position/true_theta	float	n	degrees	The pitch of the aircraft relative to the earth precisely below the aircraft

#sim/flightmodel/position/alpha	float	n	degrees	The pitch relative to the flown path (angle of attack)
#sim/flightmodel/position/vpath	float	n	degrees	The pitch the aircraft actually flies.  (vpath+alpha=theta)
#sim/flightmodel/position/hpath	float	n	degrees	The heading the aircraft actually flies.  (hpath+beta=psi)
#sim/flightmodel/position/indicated_airspeed2	float	y	kias	Air speed indicated - this takes into account air density and wind direction
#sim/flightmodel/position/true_airspeed	float	n	meters/sec	Air speed true - this does not take into account air density at altitude!
#sim/flightmodel/position/M	float	n	NM	The angular momentum of the aircraft (relative to flight axis).
#sim/flightmodel/position/N	float	n	NM	The angular momentum of the aircraft (relative to flight axis)
#sim/flightmodel/position/L	float	n	NM	The angular momentum of the aircraft (relative to flight axis)
#sim/flightmodel/position/R	float	y	deg/sec	The yaw rotation rates (relative to the flight)
#sim/flightmodel/position/P_dot	float	n	deg/sec2	The roll angular acceleration (relative to the flight)
#sim/flightmodel/position/Q_dot	float	n	deg/sec2	The pitch angular acceleration (relative to the flight)
#sim/flightmodel/position/R_dot	float	n	deg/sec2	The yaw angular acceleration rates (relative to the flight)
#sim/flightmodel/position/Prad	float	y	rad/sec	The roll rotation rates (relative to the flight)
#sim/flightmodel/position/Qrad	float	y	rad/sec	The pitch rotation rates (relative to the flight)
#sim/flightmodel/position/Rrad	float	y	rad/sec	The yaw rotation rates (relative to the flight)
#sim/flightmodel/position/q	float[4]	y	quaternion	A quaternion representing the rotation from local OpenGL coordinates to the aircrafts coordinates.
#sim/flightmodel/position/vh_ind	float	n	meters/second	VVI (vertical velocity in meters per second)
#sim/flightmodel/position/vh_ind_fpm2	float	y	fpm	VVI (vertical velocity in feet per second)
#sim/flightmodel/position/y_agl	float	n	meters	AGL
        time.sleep(.5)
        self.ProcessIncoming()
        return

    def Altitude(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[0][1] == "Altitude")
        return self.sensor_suite[0][0]

    def Heading(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[1][1] == "Heading")
        return self.sensor_suite[1][0]

    def Roll(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[2][1] == "Roll")
        return self.sensor_suite[2][0]

    def RollRate(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[3][1] == "RollRate")
        return self.sensor_suite[3][0]

    def Pitch(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[4][1] == "Pitch")
        return self.sensor_suite[4][0]

    def PitchRate(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[5][1] == "PitchRate")
        return self.sensor_suite[5][0]

    def Yaw(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[6][1] == "Yaw")
        return self.sensor_suite[6][0]

    def AirSpeed(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[7][1] == "AirSpeed")
        return self.sensor_suite[7][0]

    def GroundSpeed(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[8][1] == "GroundSpeed")
        return self.sensor_suite[8][0]

    def ClimbRate(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[9][1] == "ClimbRate")
        return self.sensor_suite[9][0]

    def Position(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[10][1] == "Longitude")
        assert(self.sensor_suite[11][1] == "Latitude")
        return (self.sensor_suite[10][0], self.sensor_suite[11][0])

    def HeadingRateChange(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[1][1] == "Heading")
        return ((self.sensor_suite[1][0] - self.previous_readings[1]) * self.SamplesPerSecond)

    def TrueHeading(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[13][1] == "TrueHeading")
        return self.sensor_suite[13][0]

    def MagneticDeclination(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[12][1] == "MagneticDeclination")
        return self.sensor_suite[12][0]

    def Time(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[14][1] == "SimTime")
        return self.sensor_suite[14][0]

    def GroundTrack(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[15][1] == "hpath")
        return (self.sensor_suite[15][0])

    def beta(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[16][1] == "beta")
        return (self.sensor_suite[16][0])

    def ProcessIncoming(self):
        global control
        while True:
            try:
                rec = control.sock.recv(1024)
                self._parse_input(rec)
            except:
                break

    def Battery(self):
        return 100

    def _parse_input(self, rec):
        DREF_SIZE=8
        assert(self.dref_rcv_struct.size == DREF_SIZE)
        if rec.startswith ("RREF"):
            rec = rec[5:]
            for index in range(0,len(rec),DREF_SIZE):
                parse = rec[index:index+DREF_SIZE]
                index,val = self.dref_rcv_struct.unpack(parse)
                if index < len(self.sensor_suite):
                    self.previous_readings[index] = self.sensor_suite[index][0]
                    ss = self.sensor_suite[index]
                    self.sensor_suite[index] = (val, ss[1], ss[2])
                    logger.log (2, "Xplane reading[%s] = %g", self.sensor_suite[index][1], val)
                    #print ("Xplane reading[%s] = %g"%(self.sensor_suite[index][1], val))
                else:
                    logger.warning("Got input from X-Plane in index %d", index)
        elif rec.startswith("DATA"):
            rec = rec[5:]
            index,v1,v2,v3,v4,v5,v6,v7,v8 = self.data_struct_body.unpack(rec)
            print ("DATA[%d]: %g, %g, %g, %g,    %g, %g, %g, %g"%(index, v1, v2,v3,v4,v5,v6,v7))

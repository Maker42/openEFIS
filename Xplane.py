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

SECONDS_HOUR = 3600.0
NM_METER = .00053996

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
        self._lookup_tables = list()
        self._throttle_table = None
        self.controls = [
                (b"sim/joystick/yoke_pitch_ratio", "YokePitch", (-1.0, 1.0)), #	The deflection of the joystick axis controlling pitch. Use override_joystick or override_joystick_pitch
                (b"sim/joystick/yoke_roll_ratio", "YokeRoll", (-1.0, 1.0)),  #	The deflection of the joystick axis controlling roll. Use override_joystick or override_joystick_roll
                (b"sim/joystick/yoke_heading_ratio", "Yaw", (-1.0, 1.0)),   #	The deflection of the joystick axis controlling yaw. Use override_joystick or override_joystick_heading
                (b"sim/flightmodel/engine/ENGN_thro_override", "Throttle", (0.0, 1.0)),  #	y	ratio	Throttle (per engine) as set by user, 0 = idle, 1 = max
                ]
        self.dref_struct_preamble = struct.pack("5s", b"DREF")
        self.dref_struct_body = struct.Struct("f500s")

        self.data_struct_preamble = struct.pack("5s", b"DATA")
        self.data_struct_body = struct.Struct("iffffffff")
        self.throttle_index = 26

        override_joystick = self.dref_struct_preamble + self.dref_struct_body.pack(1.0, b"sim/operation/override/override_joystick")
        self.sock.sendto (override_joystick, (self.xplane_host, self.xplane_port))

        override_throttle = self.dref_struct_preamble + self.dref_struct_body.pack(1.0, b"sim/operation/override/override_throttles")
        self.sock.sendto (override_throttle, (self.xplane_host, self.xplane_port))

        set_zero = [
                        b"sim/joystick/joystick_pitch_nullzone",
                        b"sim/joystick/joystick_roll_nullzone",
                        b"sim/joystick/joystick_heading_nullzone",
                   ]
        for sz in set_zero:
            szc = self.dref_struct_preamble + self.dref_struct_body.pack(0.0, sz)
            self.sock.sendto (szc, (self.xplane_host, self.xplane_port))

    def initialize(self, filelines):
        pass

    def SetThrottleTable(self, table):
        self._throttle_table = table

    def SetLookupTable(self, channel, table):
        while len(self._lookup_tables) <= channel:
            self._lookup_tables.append(None)
        self._lookup_tables[channel] = table

    def GetLimits(self, channel):
        if channel < 0 or channel >= len(self._lookup_tables) or self._lookup_tables[channel] == None:
            raise RuntimeError ("Invalid channel set (%d)"%channel)
        mn = 9999999
        mx = -9999999
        for k,v in self._lookup_tables[channel]:
            mn = mn if mn < k else k
            mx = mx if mx > k else k
        return (mn,mx)

    def SetAnalogChannel(self, channel, val):
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
        self.sensor_suite = [(0, "Altitude", b"sim/flightmodel/misc/h_ind"),
                (0, "Heading", b"sim/flightmodel/position/mag_psi"),
                (0, "Roll", b"sim/flightmodel/position/true_phi"),
                (0, "RollRate", b"sim/flightmodel/position/P"),
                (0, "Pitch", b"sim/flightmodel/position/true_theta"),
                (0, "PitchRate", b"sim/flightmodel/position/Q"),
                (0, "Yaw", b"sim/flightmodel/position/beta"),
                (0, "AirSpeed", b"sim/flightmodel/position/indicated_airspeed"),
                (0, "GroundSpeed", b"sim/flightmodel/position/groundspeed"),
                (0, "ClimbRate", b"sim/flightmodel/position/vh_ind_fpm"),
                (0, "Longitude", b"sim/flightmodel/position/longitude"),
                (0, "Latitude", b"sim/flightmodel/position/latitude"),
                (0, "MagneticDeclination", b"sim/flightmodel/position/magnetic_variation"),
                (0, "TrueHeading", b"sim/flightmodel/position/true_psi"),
                (0, "SimTime", b"sim/time/total_running_time_sec"),
                (0, "GroundTrack", b"sim/flightmodel/position/hpath"),
                (0, "WindSpeed", b"sim/weather/wind_speed_kt[0]"),
                (0, "WindDirection", b"sim/weather/wind_direction_degt[0]"),
                (0, "AGL", b"sim/flightmodel/position/y_agl"),
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
        pre = self.preamble_struct.pack (b"RREF")
        for d in self.sensor_suite:
            req = self.dref_request_struct.pack(self.SamplesPerSecond, dref_num, d[2])
            control.sock.sendto(pre + req, (control.xplane_host, control.xplane_port))
            dref_num += 1

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
        return self.sensor_suite[8][0] * SECONDS_HOUR * NM_METER

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

    # Actual flight path in true coordinates
    def GroundTrack(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[15][1] == "GroundTrack")
        return (self.sensor_suite[15][0])

    def WindSpeed(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[16][1] == "WindSpeed")
        return (self.sensor_suite[16][0])

    def WindDirection(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[17][1] == "WindDirection")
        return (self.sensor_suite[17][0])

    def AGL(self):
        self.ProcessIncoming()
        assert(self.sensor_suite[18][1] == "AGL")
        return (self.sensor_suite[18][0])

    def OuterEnginePosition(self):
        return "vertical"

    def Snapshot(self):
        ret = dict()
        for s in self.sensor_suite:
            ret[s[1]] = s[0]
        return str(ret)

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
        header = rec[:4]
        if header  == "RREF" or header == b"RREF":
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
        elif header == "DATA" or header == b"DATA":
            rec = rec[5:]
            index,v1,v2,v3,v4,v5,v6,v7,v8 = self.data_struct_body.unpack(rec)
            print ("DATA[%d]: %g, %g, %g, %g,    %g, %g, %g, %g"%(index, v1, v2,v3,v4,v5,v6,v7))

    def KnownAltitude(self, alt):
        pass

    def KnownMagneticVariation(self, v):
        pass

    def FlightMode (self, mode, vertical=True):
        pass

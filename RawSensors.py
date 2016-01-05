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

import time, logging

import serial

import NMEAParser, ArduinoCmdMessenger, Globals

logger=logging.getLogger(__name__)

METERS_FOOT = 0.3048
FEET_METER = 1.0 / METERS_FOOT

class RawArduinoSensors:
    def __init__(self):
        self.residual_input = ""

        # Readings from GPS
        self.Utc = None
        self.Latitude = None
        self.Longitude = None
        self.GpsAltitude = None
        self.GroundSpeed = None
        self.GroundTrack = None
        self.SignalQuality = 0
        self.MagneticVariation = 0.0

        # Readings from 10DOF sensors
        self.ReadingTime = 0.0
        self.Pressure = None
        self.Temp = None
        self.A_forward = None
        self.A_side = None
        self.A_up = None
        self.R_pitch = None
        self.R_roll = None
        self.R_yaw = None
        self.M_forward = None
        self.M_side = None
        self.M_up = None
        self.HaveNew10DOF = False


    def initialize(self, cmdMessenger):
        self._cmdMessenger = cmdMessenger
        cmdMessenger.attach(Globals.CMD_10DOF_SENSOR_DATA, self.Get10DOFSensorData)
        cmdMessenger.attach(Globals.CMD_GPS_DATA, self.GetGPSData)
        cmdMessenger.attach(Globals.CMD_LOG, self._logMainCmd)
        self.HaveNew10DOF = False
        self._have_gps = False
        self.HaveNewGroundTrack = False
        self.HaveNewPosition = False
        while not (self.HaveNew10DOF and self.HaveNewGroundTrack and self.HaveNewPosition):
            self.ProcessIncoming()
            logger.debug("10dof = %s, gt=%s, pos=%s", self.HaveNew10DOF, self.HaveNewGroundTrack, self.HaveNewPosition)
            time.sleep(.01)
        return

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

    def ProcessIncoming(self):
        self._cmdMessenger.feedinSerialData()

    def Get10DOFSensorData(self, args):
        line,checksum = args
        checksum = int(checksum)
        if 0 == ArduinoCmdMessenger.ComputeCheckSum(line, checksum):
            line = line.strip()
            fields = line.split(',')
            if len(fields) < 12:
                logger.warning("Invalid sensor string: %s", line)
            else:
                self.HaveNew10DOF = True
                try:
                    self.Pressure = float(fields[0])
                except:
                    self.Pressure = None
                    self.HaveNew10DOF = False
                    logger.debug("10DOF sensors are not reading. Do not fly")
                try:
                    self.Temp = float(fields[1])
                except:
                    self.Temp = None
                    self.HaveNew10DOF = False
                try:
                    self.R_pitch = float(fields[2])
                except:
                    self.R_pitch = None
                    self.HaveNew10DOF = False
                try:
                    self.R_roll = float(fields[3])
                except:
                    self.R_roll = None
                    self.HaveNew10DOF = False
                try:
                    self.R_yaw = float(fields[4])
                except:
                    self.R_yaw = None
                    self.HaveNew10DOF = False
                try:
                    self.A_side = float(fields[5])
                except:
                    self.A_side = None
                    self.HaveNew10DOF = False
                try:
                    self.A_forward = float(fields[6])
                except:
                    self.A_forward = None
                    self.HaveNew10DOF = False
                try:
                    self.A_up = float(fields[7])
                except:
                    self.A_up = None
                    self.HaveNew10DOF = False
                try:
                    self.M_forward = float(fields[8])
                except:
                    self.M_forward = None
                    self.HaveNew10DOF = False
                try:
                    self.M_side = float(fields[9])
                except:
                    self.M_side = None
                    self.HaveNew10DOF = False
                try:
                    self.M_up = float(fields[10])
                except:
                    self.M_up = None
                    self.HaveNew10DOF = False
                try:
                    self.ReadingTime = int(fields[11])
                except:
                    self.HaveNew10DOF = False
                    return
                logger.log(1, "Got sensor readings 10DOF: Pressure = %s, Temp=%s, a=%s,%s,%s, r=%s,%s,%s, m=%s,%s,%s",
                self.Pressure,
                self.Temp,
                self.A_forward,
                self.A_side,
                self.A_up,
                self.R_pitch,
                self.R_roll,
                self.R_yaw,
                self.M_forward,
                self.M_side,
                self.M_up)
        else:
            logger.warning("10DOF Sensor Data checksum mismatch %d !> %d"%(checksum, ArduinoCmdMessenger.ComputeCheckSum(line, checksum)))


    def GetGPSData(self, args):
        line = args[0]
        NMEAParser.ParseNMEAStrings(line, self)
        if self.Utc != None and self.Latitude != None and self.GroundSpeed != None:
            if not self._have_gps:
                logger.info ("Obtained GPS readings")
            self._have_gps = True
        else:
            if self._have_gps:
                logger.warning ("Lost GPS readings")
            self._have_gps = False

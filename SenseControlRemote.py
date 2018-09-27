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

import os
import sys,  math, time

import logging
import argparse

import yaml

import serial

from ArduinoCmdMessenger import ArduinoCmdMessenger

import Globals
import Common.Spatial as Spatial
import Common.util as util

from NMEAParser import ParseNMEAStrings

from MicroServerComs import MicroServerComs

logger=logging.getLogger(__name__)

arduino_messages = [
# Messages from sense/control board
 "ack"                          # arguments description:
,"nack"                         # reason for nack
,"sensor_reading"               # channel, value, timestamp
,"log"                          # loglevel, string

# Messages from this program to the board
,"setup_digital_sensor"         # channel, pin, polling period (ms), use internal pullup (0 or 1)
,"setup_analog_sensor"          # channel, pin, polling period (ms)
,"setup_i2c_sensor"             # channel, function, polling period (ms)
,"setup_spi_sensor"             # channel, function, polling period (ms)
,"setup_serial_sensor"          # channel, function, port number, polling period (ms)
,"setup_analog_output"          # pin, initial_value
,"setup_digital_output"         # pin, initial_value
,"set_analog_output"            # pin, value
,"set_digital_output"           # pin, value
]

class SenseControlSlave(MicroServerComs):
    def __init__(self, command_channel, config):
        self._mainCmd = command_channel
        self._config = config
        self._sensors = dict()
        sensor_chnum = 0
        self._accelerometers = Accelerometers()
        self._rotation = Rotation()
        self._magnetic = Magnetic()
        self._pressure = Pressure()
        self._temperature = Temperature()
        self._gps = GPS()
        for chname,cfg in self._config.items():
            if cfg[0].startswith ('sensor'):
                if cfg[0] == 'sensor_digital':
                    _,function,pin,period = cfg
                    self._mainCmd.sendCmd ("setup_digital_sensor", [sensor_chnum, pin, period])
                elif cfg[0] == 'sensor_analog':
                    _,function,pin,period = cfg
                    self._mainCmd.sendCmd ("setup_analog_sensor", [sensor_chnum, pin, period])
                elif cfg[0] == 'sensor_i2c':
                    _,function,period = cfg
                    self._mainCmd.sendCmd ("setup_i2c_sensor", [sensor_chnum, function, period])
                elif cfg[0] == 'sensor_spi':
                    _,function,period = cfg
                    self._mainCmd.sendCmd ("setup_spi_sensor", [sensor_chnum, function, period])
                elif cfg[0] == 'sensor_serial':
                    _,function,port,period = cfg
                    self._mainCmd.sendCmd ("setup_serial_sensor", [sensor_chnum, function, port, period])
                else:
                    raise RuntimeError ("Unsupported sensor type: %s"%cfg[0])
                self._sensors[sensor_chnum] = function
                sensor_chnum += 1
        MicroServerComs.__init__(self, "ControlSlave")

    def update(self, channel):
        if channel == "Control":
            if not self.channel in self._config:
                raise RuntimeError ("Invalid channel name received: %s"%self.channel)
            chtype,pin = self._config[self.channel]
            if chtype == 'analog_output':
                self._mainCmd.sendCmd ("set_analog_output", pin, self.value)
            elif chtype == 'digital_output':
                self._mainCmd.sendCmd ("set_digital_output", pin, self.value)
            else:
                raise RuntimeError ("Trying to set invalid channel (%s) type %s"%(self.channel, chtype))

    def ReadSensors(self):
        global rootlogger
        while True:
            cmd = self._mainCmd.read_command (0)
            if cmd is not None:
                name, args, ts = cmd
                if name == 'sensor_reading':
                    ch = args[0]
                    if not ch in self._sensors:
                        rootlogger.error ("Invalid sensor channel received from board: %d"%ch)
                    else:
                        function = self._sensors[ch]
                        self.sensor_updated (function, args[1:])
                if name == 'log':
                    loglevel,log_string = args
                    rootlogger.log (loglevel, "scboard: " + log_string)
            else:
                break

    def sensor_updated(self, function, args):
        """ function labels used by sensor board:
        ts = timestamp. unsigned 4 byte integer in milliseconds since boot
        a: accelerometers. output format x,y,z,ts floating point. units: don't care
        r: rotation sensors. output format x,y,z,ts floating point. units: degrees / sec
        m: magnetic sensors. output format x,y,z,ts floating point. units: don't care
        p: pressure sensors. output format static,pitot,ts floating point. units: Kilo-Pascals
        t: temperature sensors. output format temperature floating point. units: Celcius
        g: gps. output format NMEA string
        """
        if function == 'a':
            self._accelerometers.send (args)
        elif function == 'r':
            self._rotation.send (args)
        elif function == 'm':
            self._magnetic.send (args)
        elif function == 'p':
            self._pressure.send (args)
        elif function == 't':
            self._temperature.send (args)
        elif function == 'g':
            self._gps.send (args)
        elif function == 'd':
            # generic digital input, configured elsewhere
            # Not implemented
            pass
        elif function == 'n':
            # generic analog input, configured elsewhere
            # Not implemented
            pass


class Accelerometers(MicroServerComs):
    def __init__(self):
        self.a_x = None
        self.a_y = None
        self.a_z = None
        self.timestamp = None
        MicroServerComs.__init__(self, "RawAccelerometers")

    def send(self, args):
        self.a_x, self.a_y, self.a_z, ts = args
        self.timestamp = make_timestamp (ts)
        print ("accel %g,%g,%g"%(self.a_x, self.a_y, self.a_z))
        self.publish()

class Rotation(MicroServerComs):
    def __init__(self):
        self.r_x = None
        self.r_y = None
        self.r_z = None
        self.timestamp = None
        MicroServerComs.__init__(self, "RawRotationSensors")

    def send(self, args):
        self.r_x, self.r_y, self.r_z, ts = args
        self.timestamp = make_timestamp (ts)
        print ("rotation %g,%g,%g"%(self.r_x, self.r_y, self.r_z))
        self.publish()

class Magnetic(MicroServerComs):
    def __init__(self):
        self.m_x = None
        self.m_y = None
        self.m_z = None
        self.timestamp = None
        MicroServerComs.__init__(self, "RawMagneticSensors")

    def send(self, args):
        self.m_x, self.m_y, self.m_z, ts = args
        self.timestamp = make_timestamp (ts)
        print ("magnetic %g,%g,%g"%(self.m_x, self.m_y, self.m_z))
        self.publish()

class Pressure(MicroServerComs):
    def __init__(self):
        self.static_pressure = None
        self.pitot_pressure = None
        self.timestamp = None
        MicroServerComs.__init__(self, "RawPressureSensors")

    def send(self, args):
        self.static_pressure, self.pitot_pressure, ts = args
        self.static_pressure /= 1000.0
        self.pitot_pressure /= 1000.0
        self.timestamp = make_timestamp (ts)
        print ("pressure %g,%g"%(self.static_pressure, self.pitot_pressure))
        self.publish()

class Temperature(MicroServerComs):
    def __init__(self):
        self.temperature = None
        MicroServerComs.__init__(self, "RawTemperatureSensors")

    def send(self, args):
        self.temperature = args[0]
        print ("temp %g"%(self.temperature))
        self.publish()

class GPS(MicroServerComs):
    def __init__(self):
        self.gps_utc = None
        self.gps_lat = None
        self.gps_lng = None
        self.gps_altitude = None
        self.gps_ground_speed = None
        self.gps_ground_track = None
        self.gps_signal_quality = None
        self.gps_magnetic_variation = None
        MicroServerComs.__init__(self, "RawTemperatureSensors")

    def send(self, args):
        nmea_string,ts = args
        ParseNMEAStrings (nmea_string, self)
        if self.HaveNewPosition and self.gps_ground_speed is not None:
            update_time_offset (self.gps_utc, ts)
            self.HaveNewPosition = False
            print ("gps: %g,%g,%g,%d,%d,%d,%d,%g"%(
        self.gps_utc,
        self.gps_lat,
        self.gps_lng,
        self.gps_altitude,
        self.gps_ground_speed,
        self.gps_ground_track,
        self.gps_signal_quality,
        self.gps_magnetic_variation))
            self.publish()

time_offset = 0

def update_time_offset(curtime, board_ts):
    global time_offset
    if time_offset == 0:
        time_offset = curtime - float(board_ts) / 1000.0

def make_timestamp (board_ts):
    if time_offset == 0:
        return time.time()
    else:
        return board_ts + time_offset

if '__main__' == __name__:
    opt = argparse.ArgumentParser(description='Control/sensor slave process')
    opt.add_argument('serial_port', help = 'Serial port path of physical controller')
    opt.add_argument('--config-file', default='sensors.yml', help='YAML config file for sensor / control board')
    opt.add_argument('-p', '--log-prefix', default=None, help = 'Over-ride logging prefix')
    opt.add_argument('-l', '--log-level', type=int, default=logging.WARNING, help = '1 = Maximum Logging. 100 = Absolute Silence. 40 = Errors only. 10 = Basic Debug')
    opt.add_argument('-v', '--magnetic-variation', default=None, help='The magnetic variation(declination) of the current position')
    opt.add_argument('-a', '--altitude', default=None, help='The currently known altitude')
    opt.add_argument('-w', '--wind', help="Wind speed and direction 'speed_knots,dir_deg'")
    args = opt.parse_args()

    rootlogger = logging.getLogger()
    rootlogger.setLevel(args.log_level)

    if args.log_prefix:
        log_prefix = args.log_prefix
    else:
        datestamp = Globals.datestamp()
        log_prefix = os.path.join('Logs', 'Sensors', datestamp)
    log_start = "Sensors beginning with logging prefix %s"%log_prefix
    try:
        os.makedirs(log_prefix)
    except:
        pass
    rootlogger.addHandler(logging.FileHandler(os.path.join(log_prefix, 'info.log')))
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    rootlogger.addHandler(console_handler)
    rootlogger.log(99, log_start)

    try:
        _main_port = serial.Serial(args.serial_port, 115200, timeout=0)
    except:
        raise RuntimeError("Cannot open control/sensor port %s"%args.serial_port)
    command_channel = ArduinoCmdMessenger(arduino_messages)
    command_channel.StartComs (_main_port)


    if args.wind:
        try:
            speed,dr = args.wind.split(',')
            speed = float(speed)
            dr = float(dr)
            wv = Spatial.Vector(math.sin(dr * util.RAD_DEG) * speed, math.cos(dr * util.RAD_DEG) * speed, 0)
            sensors.WindVector (wv)
            rootlogger.info("Wind Vector = %s", str(wv))
        except Exception as e:
            rootlogger.error ("Invalid wind input: %s (%s)", args.wind, str(e))
    else:
        rootlogger.info("Wind = None")

    with open(args.config_file, 'r') as yml:
        config = yaml.load (yml)
        yml.close()
    slave = SenseControlSlave(command_channel, config)
    while True:
        slave.ReadSensors()
        slave.listen (timeout=0, loop=False)
        time.sleep(.01)

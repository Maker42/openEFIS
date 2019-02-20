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
from PubSub import assign_all_ports

logger=logging.getLogger(__name__)

arduino_messages = [
# Messages from sense/control board
 "ack"                          # arguments description:
,"nack"                         # reason for nack
,"sensor_reading"               # channel, value, timestamp, nsubsamples
,"log"                          # loglevel, string

# Messages from this program to the board
,"setup_digital_sensor"         # channel, pin, polling period (ms), use internal pullup (0 or 1)
,"setup_analog_sensor"          # channel, pin, polling period (ms), filter_coefficients, secondary_band, rejection_band, secondary_duration
,"setup_i2c_sensor"             # channel, function, polling period (ms), filter_coefficients, secondary_band, rejection_band, secondary_duration
,"setup_spi_sensor"             # channel, function, polling period (ms), filter_coefficients, secondary_band, rejection_band, secondary_duration
,"setup_serial_sensor"          # channel, function, type, port number
,"setup_analog_output"          # pin, initial_value
,"setup_digital_output"         # pin, initial_value
,"set_analog_output"            # pin, value
,"set_digital_output"           # pin, value
,"save_configuration"           # nchans
]

class SenseControlSlave(MicroServerComs):
    def __init__(self, command_channel, config, pubsub_cfg):
        self._mainCmd = command_channel
        self._config = config
        self._sensors = dict()
        sensor_chnum = 0
        output_chnum = 0
        self._accelerometers = Accelerometers(pubsub_cfg)
        self._rotation = Rotation(pubsub_cfg)
        self._magnetic = Magnetic(pubsub_cfg)
        self._pressure = Pressure(pubsub_cfg)
        self._temperature = Temperature(pubsub_cfg)
        self._gps = GPS(pubsub_cfg)
        self._calibrations = dict()
        self.channel = 0
        self.value = 0
        self.engaged = 0
        self.previous_engaged = 0
        self.engage_channel = None
        for config_term,cfg in self._config.items():
            if config_term.startswith ('cal'):
                self._calibrations.update (cfg)
            elif cfg[0].startswith ('sensor'):
                function = cfg[1]
                if cfg[0] == 'sensor_digital':
                    resp = self._mainCmd.sendCmd ("setup_digital_sensor", [sensor_chnum] + cfg[1:])
                    function = 'd'
                elif cfg[0] == 'sensor_analog':
                    resp = self._mainCmd.sendCmd ("setup_analog_sensor", [sensor_chnum] + cfg[1:])
                    function = 'n'
                elif cfg[0] == 'sensor_i2c':
                    resp = self._mainCmd.sendCmd ("setup_i2c_sensor", [sensor_chnum] + cfg[1:])
                elif cfg[0] == 'sensor_spi':
                    resp = self._mainCmd.sendCmd ("setup_spi_sensor", [sensor_chnum] + cfg[1:])
                elif cfg[0] == 'sensor_serial':
                    resp = self._mainCmd.sendCmd ("setup_serial_sensor", [sensor_chnum] + cfg[1:])
                else:
                    raise RuntimeError ("Unsupported sensor type: %s"%cfg[0])
                if resp is not None:
                    for r in resp:
                        self.ProcessResponse(r)
                self._sensors[sensor_chnum] = function
                sensor_chnum += 1
            elif cfg[0].startswith('output'):
                function = cfg[1]
                if cfg[0] == 'output_digital':
                    resp = self._mainCmd.sendCmd ("setup_digital_output", [output_chnum] + cfg[1:])
                    function = 'd'
                elif cfg[0] == 'output_analog':
                    resp = self._mainCmd.sendCmd ("setup_analog_output", [output_chnum] + cfg[1:])
                    function = 'n'
                if config_term == "engage":
                    self.engage_channel = output_chnum
                output_chnum += 1
        self._mainCmd.sendCmd ("save_configuration", [sensor_chnum])
        MicroServerComs.__init__(self, "ControlSlave", config=pubsub_cfg)

    def update(self, channel):
        if channel == "Control":
            if self.engaged != self.previous_engaged and self.engage_channel is not None:
                chtype,pin = self._config[self.engage_channel]
                self._mainCmd.sendCmd ("set_digital_output", pin, self.engaged)
                self.previous_engaged = self.engaged
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
            cmd = self._mainCmd.read_command (0.01)
            if not self.ProcessResponse (cmd):
                break

    def ProcessResponse(self, cmd):
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
            return True
        else:
            return False

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
            self._accelerometers.send (args, self._calibrations.get('accelerometers'))
        elif function == 'r':
            self._rotation.send (args,self._calibrations.get('rotation'))
        elif function == 'm':
            cal = self._calibrations.get('magnetic')
            if cal is not None:
                cal.append(self._rotation)
            self._magnetic.send (args, cal)
        elif function == 'p':
            self._pressure.send (args, self._calibrations.get('pressure'))
        elif function == 't':
            self._temperature.send (args, self._calibrations.get('temperature'))
        elif function == 'g':
            self._gps.send (args)
        elif function == 'd':
            # generic digital input, configured elsewhere
            # Not implemented
            raise RuntimeError ("digital input not implemented")
        elif function == 'n':
            # generic analog input, configured elsewhere
            # Not implemented
            raise RuntimeError ("analog input not implemented")


class SampleCounter():
    def __init__(self, pp=50):
        self.sc_min = 9999999
        self.sc_max = 0
        self.sc_sum = 0
        self.sc_count = 0
        self.reject_count = 0
        self.secondary_count = 0
        self.print_period = pp

    def update_counts(self, secondary, rj):
        if self.sample_count < self.sc_min:
            self.sc_min = self.sample_count
        if self.sample_count > self.sc_max:
            self.sc_max = self.sample_count
        self.sc_sum += self.sample_count
        self.sc_count += 1
        self.reject_count += rj
        self.secondary_count += secondary
        if self.sc_count >= self.print_period:
            print ("%s: min %d, max %d, mean %g; sec %d, rej %d"%(str(type(self)),
                self.sc_min, self.sc_max, float(self.sc_sum) / float(self.sc_count),
                self.secondary_count, self.reject_count))
            self.sc_min = 9999999
            self.sc_max = 0
            self.sc_sum = 0
            self.sc_count = 0
            self.secondary_count = 0
            self.reject_count = 0

class Accelerometers(MicroServerComs,SampleCounter):
    def __init__(self, pubsub_cfg):
        self.a_x = None
        self.a_y = None
        self.a_z = None
        self.timestamp = None
        MicroServerComs.__init__(self, "RawAccelerometers", channel='accelerometers', config=pubsub_cfg)
        SampleCounter.__init__(self)

    def send(self, args, calibration):
        self.a_x, self.a_y, self.a_z, ts, self.sample_count, secondary, rj = args
        self.update_counts(secondary, rj)
        self.timestamp = make_timestamp (ts)
        #print ("accel %g,%g,%g"%(self.a_x, self.a_y, self.a_z))
        self.publish()

class Rotation(MicroServerComs,SampleCounter):
    def __init__(self, pubsub_cfg):
        self.r_x = None
        self.r_y = None
        self.r_z = None
        self.cal_collection_count = 0
        self.current_bias = [0.0, 0.0, 0.0]
        self.samples = [list(), list(), list()]
        self.timestamp = None
        self.print_count = 0
        MicroServerComs.__init__(self, "RawRotationSensors", channel='rotationsensors', config=pubsub_cfg)
        SampleCounter.__init__(self)

    def send(self, args, calibration):
        self.r_x, self.r_y, self.r_z, ts, self.sample_count, secondary, rj = args
        self.update_counts(secondary, rj)
        if calibration:
            if isinstance (calibration[0],str) and calibration[0] == 'collect':
                if self.cal_collection_count < calibration [1]:
                    self.current_bias [0] += self.r_x
                    self.current_bias [1] += self.r_y
                    self.current_bias [2] += self.r_z
                    self.cal_collection_count += 1
                    if self.cal_collection_count == calibration [1]:
                        self.current_bias [0] /= self.cal_collection_count
                        self.current_bias [1] /= self.cal_collection_count
                        self.current_bias [2] /= self.cal_collection_count
                    c_x = self.current_bias [0] / self.cal_collection_count
                    c_y = self.current_bias [1] / self.cal_collection_count
                    c_z = self.current_bias [2] / self.cal_collection_count
                else:
                    c_x, c_y, c_z = self.current_bias
            else:
                c_x, c_y, c_z = calibration
            if len(calibration) >= 3 and calibration[2] == 'respond_heading':
                self.samples[0].append (self.r_x)
                self.samples[1].append (self.r_y)
                self.samples[2].append (self.r_z)
            self.r_x -= c_x
            self.r_y -= c_y
            self.r_z -= c_z

        self.timestamp = make_timestamp (ts)
        self.print_count += 1
        if self.print_count % 10 == 0:
            print ("rotation %g,%g,%g"%(self.r_x, self.r_y, self.r_z))
        self.publish()

    def reset_bias(self):
        mean = [sum(x) for x in self.samples]
        self.current_bias = [x/len(self.samples[0]) for x in mean]
        print ("New rotation bias %s"%(str(self.current_bias)))

    def reset_samples(self):
        self.samples = [list(), list(), list()]

class Magnetic(MicroServerComs,SampleCounter):
    def __init__(self, pubsub_cfg):
        self.m_x = None
        self.m_y = None
        self.m_z = None
        self.samples = [list(), list(), list()]
        self.timestamp = None
        MicroServerComs.__init__(self, "RawMagneticSensors", channel='magneticsensors', config=pubsub_cfg)
        SampleCounter.__init__(self)

    def send(self, args, calibration):
        self.m_x, self.m_y, self.m_z, ts, self.sample_count, secondary, rj = args
        self.update_counts(secondary, rj)
        if calibration is not None and isinstance (calibration[0],str) and calibration[0] == 'rotation':
            if len(calibration) >= 5:
                sample_count,max_sample_dev,max_trend_dev,rot_object = calibration[1:5]
                self.samples[0].append (self.m_x)
                self.samples[1].append (self.m_y)
                self.samples[2].append (self.m_z)
                if len(self.samples[0]) >= sample_count:
                    mean = [sum(x)/len(self.samples[0]) for x in self.samples]
                    deviation = [[abs(x-m) for x in samp] for samp,m in zip(self.samples,mean)]
                    deviation = [max(x) for x in deviation]
                    deviation = max(deviation)
                    end_mean = [sum(x[-3:])/3 for x in self.samples]
                    beg_mean = [sum(x[:3])/3 for x in self.samples]
                    trend = [abs(e-b) for e,b in zip(end_mean,beg_mean)]
                    max_trend = max(trend)
                    if deviation < max_sample_dev and max_trend < max_trend_dev:
                        rot_object.reset_bias()
                        print ("reset rotation bias because %.4g<%.4g and %.4g<%.4g"%(
                                    deviation, max_sample_dev,
                                    max_trend, max_trend_dev))
                    else:
                        print ("DO NOT reset rotation bias because %.4g>=%.4g or %.4g>=%.4g"%(
                                    deviation, max_sample_dev,
                                    max_trend, max_trend_dev))
                    self.samples = [list(), list(), list()]
                    rot_object.reset_samples()
        self.timestamp = make_timestamp (ts)
        #print ("magnetic %g,%g,%g"%(self.m_x, self.m_y, self.m_z))
        self.publish()

class Pressure(MicroServerComs,SampleCounter):
    def __init__(self, pubsub_cfg):
        self.static_pressure = None
        self.pitot_pressure = None
        self.timestamp = None
        self.sc_min = 9999999
        self.sc_max = 0
        MicroServerComs.__init__(self, "RawPressureSensors", channel='pressuresensors', config=pubsub_cfg)
        SampleCounter.__init__(self, 10)

    def send(self, args, calibration):
        self.static_pressure, self.pitot_pressure, ts, self.sample_count, secondary, rj = args
        self.update_counts(secondary, rj)
        self.static_pressure /= 1000.0
        self.pitot_pressure /= 1000.0
        self.timestamp = make_timestamp (ts)
        #print ("pressure %g,%g"%(self.static_pressure, self.pitot_pressure))
        self.publish()

class Temperature(MicroServerComs,SampleCounter):
    def __init__(self, pubsub_cfg):
        self.temperature = None
        MicroServerComs.__init__(self, "RawTemperatureSensors", channel='temperaturesensors', config=pubsub_cfg)
        SampleCounter.__init__(self, 10)

    def send(self, args, calibration):
        self.temperature, self.sample_count, secondary, rj = args
        self.update_counts(secondary, rj)
        #print ("temp %g"%(self.temperature))
        self.publish()

class GPS(MicroServerComs):
    def __init__(self, pubsub_cfg):
        self.gps_utc = None
        self.gps_lat = None
        self.gps_lng = None
        self.gps_altitude = None
        self.gps_ground_speed = None
        self.gps_ground_track = None
        self.gps_signal_quality = None
        self.HaveNewPosition = False
        MicroServerComs.__init__(self, "GPSFeed", channel='gpsfeed', config=pubsub_cfg)

    def send(self, args):
        nmea_string,ts = args
        ParseNMEAStrings (nmea_string, self)
        if self.HaveNewPosition and self.gps_ground_speed is not None and \
                self.gps_lat is not None:
            update_time_offset (self.gps_utc, ts)
            self.HaveNewPosition = False
            print ("gps: %g,%g,%g,%g,%g,%g,%d"%(
        self.gps_utc,
        self.gps_lat,
        self.gps_lng,
        self.gps_altitude,
        self.gps_ground_speed,
        self.gps_ground_track,
        self.gps_signal_quality))
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
        return float(board_ts)/1000.0 + time_offset

if '__main__' == __name__:
    opt = argparse.ArgumentParser(description='Control/sensor slave process')
    opt.add_argument('serial_port', help = 'Serial port path of physical controller')
    opt.add_argument('starting_port', type=int,
            help='The port number to start with for serialized network port assignments')
    opt.add_argument('-c', '--config-file', default='sensors.yml', help='YAML config file for sensor / control board')
    opt.add_argument('-p', '--pubsub-config', default='sensors_pubsub.yml', help='YAML config file coms configuration')
    opt.add_argument('--log-prefix', default=None, help = 'Over-ride logging prefix')
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
    with open (args.pubsub_config, 'r') as yml:
        pubsub_config = yaml.load(yml)
        assign_all_ports (pubsub_config, args.starting_port)
        yml.close()
    slave = SenseControlSlave(command_channel, config, pubsub_config)
    while True:
        try:
            slave.ReadSensors()
        except Exception as e:
            time.sleep(1)
            try:
                _main_port = serial.Serial(args.serial_port, 115200, timeout=0)
                command_channel.StartComs (_main_port)
            except:
                raise RuntimeError("Cannot re-open control/sensor port %s"%args.serial_port)

        slave.listen (timeout=0, loop=False)

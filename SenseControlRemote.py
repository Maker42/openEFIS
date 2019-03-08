# Copyright (C) 2015-2019  Garrett Herschleb
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
from cmd import Cmd
import threading

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
,"trace"                        # line number

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
,"update_sensor"                # channel, polling period (ms), filter_coefficients, secondary_band, rejection_band, secondary_duration
]

class SenseControlSlave(MicroServerComs):
    def __init__(self, command_channel, config, pubsub_cfg):
        self._mainCmd = command_channel
        self.cmd_lock = threading.Lock()
        self._config = config
        self._sensors = dict()
        self._scfg = dict()
        sensor_chnum = 0
        output_chnum = 0
        self._accelerometers = Accelerometers(pubsub_cfg)
        self._rotation = Rotation(pubsub_cfg)
        self._magnetic = Magnetic(pubsub_cfg)
        self._pressure = Pressure(pubsub_cfg)
        self._pitot = Pitot(pubsub_cfg)
        self._temperature = Temperature(pubsub_cfg)
        self._gps = GPS(pubsub_cfg)
        self._calibrations = dict()
        self.channel = 0
        self.value = 0
        self.engaged = 0
        self.previous_engaged = 0
        self.engage_channel = None
        self.last_trace_report = time.time()
        self.trace_lines = list()
        for config_term,cfg in self._config.items():
            if config_term.startswith ('cal'):
                self._calibrations.update (cfg)
            elif cfg[0].startswith ('sensor'):
                function = cfg[1]
                if cfg[0] == 'sensor_digital':
                    resp = self._mainCmd.sendCmd ("setup_digital_sensor", [sensor_chnum] + cfg[1:])
                    function = 'd' + config_term
                elif cfg[0] == 'sensor_analog':
                    resp = self._mainCmd.sendCmd ("setup_analog_sensor", [sensor_chnum] + cfg[1:])
                    function = 'n' + config_term
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
                self._scfg[sensor_chnum] = cfg
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
        #self._mainCmd.sendCmd ("save_configuration", [sensor_chnum])
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
        begin_time = time.time()
        while True:
            self.cmd_lock.acquire()
            cmd = self._mainCmd.read_command (0.01)
            self.cmd_lock.release()
            if not self.ProcessResponse (cmd):
                break
        #if time.time() - self.last_trace_report > 1:
        #    self.last_trace_report = time.time()
        #    print ("last trace lines: %s"%str(self.trace_lines[-10:]))

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
            elif name == 'log':
                loglevel,log_string = args
                rootlogger.log (loglevel, "scboard: " + log_string)
            elif name == 'trace':
                self.trace_lines.append (int(args[0]))
                if len(self.trace_lines) > 100:
                    del self.trace_lines[0]
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
        elif function.startswith('n'):
            if 'pitot' in function:
                self._pitot.send (args)
            else:
                raise RuntimeError ("Unknown analog input type %s"%function[1:])

    def sensor_name2obj(self, name):
        if name.startswith('t'):
            return self._temperature
        elif name.startswith('pi'):
            return self._pitot
        elif name.startswith('pr'):
            return self._pressure
        elif name.startswith('a'):
            return self._accelerometers
        elif name.startswith('g'):
            return self._rotation
        elif name.startswith('m'):
            return self._magnetic
        else:
            return None

    def print_sensor(self, name):
        obj = self.sensor_name2obj (name)
        if obj is None:
            print ("Unknown sensor '%s'"%name)
            return
        else:
            obj.print_data()

    def update_sensor(self, sensor_chnum, cfg):
        self.cmd_lock.acquire()
        resp = self._mainCmd.sendCmd ("update_sensor", [sensor_chnum] + cfg[2:])
        self.cmd_lock.release()
        if resp is not None:
            for r in resp:
                self.ProcessResponse(r)

    def sensor_name2chnum(self, sname):
        for num,func in enumerate(self._sensors.values()):
            if func == sname or func.startswith(sname):
                return num
        else:
            return -1

    def sensor_parm2index (self, pname):
        for i,n in enumerate(['polling period (ms)', 'f0', 'f1', 'sband', 'rejection_band', 'sduration']):
            if pname == n or n.startswith(pname):
                return i+2
        else:
            return -1

    def modify_sensor_parm(self, sname, pname, val):
        chnum = self.sensor_name2chnum(sname)
        if chnum < 0:
            print ("Can't find sensor %s"%sname)
            return
        pnum = self.sensor_parm2index(pname)
        if pnum < 0:
            print ("Can't find parameter %s"%pname)
            return
        try:
            val = float(val)
        except Exception as e:
            print ("Invalid value: %s (%s)"%(val, str(e)))
            return
        self._scfg[chnum][pnum] = val
        self.update_sensor (chnum, self._scfg[chnum])
        print ("Sensor update sent")

class SampleCounter():
    def __init__(self, pp=50):
        self.sc_min = 9999999
        self.sc_max = 0
        self.sc_sum = 0
        self.sc_count = 0
        self.reject_count = 0
        self.secondary_count = 0
        self.print_period = pp
        self.enable_printing = False

    def update_counts(self, secondary, rj):
        ret = False
        if self.sample_count < self.sc_min:
            self.sc_min = self.sample_count
        if self.sample_count > self.sc_max:
            self.sc_max = self.sample_count
        self.sc_sum += self.sample_count
        self.sc_count += 1
        self.reject_count += rj
        self.secondary_count += secondary
        if self.sc_count >= self.print_period:
            if self.enable_printing:
                self.print_data()
                ret = True
            self.sc_min = 9999999
            self.sc_max = 0
            self.sc_sum = 0
            self.sc_count = 0
            self.secondary_count = 0
            self.reject_count = 0
        return ret

    def print_stats(self):
        if self.sc_count == 0:
            print ("%s: No current data. Try again."%str(type(self)))
        else:
            print ("%s: min %d, max %d, mean %g; sec %d, rej %d"%(
                str(type(self)),
                self.sc_min, self.sc_max,
                float(self.sc_sum) / float(self.sc_count),
                self.secondary_count, self.reject_count))

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
        self._send (secondary, rj, ts)

    def send2(self, values, ts, stats, calibration):
        self.a_x, self.a_y, self.a_z = values
        self.sample_count, secondary, rj = stats
        self._send (secondary, rj, ts)

    def _send(self, secondary, rj, ts):
        if self.update_counts(secondary, rj):
            print ("accel %.2f,%.2f,%.2f"%(self.a_x, self.a_y, self.a_z))
        self.timestamp = make_timestamp (ts)
        self.publish()

    def print_data(self):
        print ("accel %.2f,%.2f,%.2f"%(self.a_x, self.a_y, self.a_z))
        self.print_stats()

class Rotation(MicroServerComs,SampleCounter):
    def __init__(self, pubsub_cfg):
        self.r_x = None
        self.r_y = None
        self.r_z = None
        self.cal_collection_count = 0
        self.current_bias = [0.0, 0.0, 0.0]
        self.samples = [list(), list(), list()]
        self.timestamp = None
        MicroServerComs.__init__(self, "RawRotationSensors", channel='rotationsensors', config=pubsub_cfg)
        SampleCounter.__init__(self)

    def send(self, args, calibration):
        self.r_x, self.r_y, self.r_z, ts, self.sample_count, secondary, rj = args
        self.calibrate(calibration)
        self._send(secondary, rj, ts)

    def send2(self, values, ts, stats, calibration):
        self.r_x, self.r_y, self.r_z = values
        self.sample_count, secondary, rj = stats
        self.calibrate(calibration)
        self._send(secondary, rj, ts)

    def _send(self, secondary, rj, ts):
        if self.update_counts(secondary, rj):
            print ("gyro %.4f,%.4f,%.4f"%(self.r_x, self.r_y, self.r_z))
        self.timestamp = make_timestamp (ts)
        self.publish()


    def calibrate(self, calibration):
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

    def reset_bias(self):
        mean = [sum(x) for x in self.samples]
        self.current_bias = [x/len(self.samples[0]) for x in mean]
        #print ("New rotation bias %s"%(str(self.current_bias)))

    def reset_samples(self):
        self.samples = [list(), list(), list()]

    def print_data(self):
        print ("gyro %.4f,%.4f,%.4f"%(self.r_x, self.r_y, self.r_z))
        self.print_stats()

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
        self.calibrate(calibration)
        self._send (secondary, rj, ts)

    def send2(self, values, ts, stats, calibration):
        self.m_x, self.m_y, self.m_z = values
        self.sample_count, secondary, rj = stats
        self.calibrate(calibration)
        self._send (secondary, rj, ts)

    def calibrate(self, calibration):
        if calibration is not None and isinstance (calibration[0],str) and calibration[0] == 'rotation':
            if len(calibration) >= 5:
                sample_count,max_sample_dev,max_trend_dev,rot_object = calibration[1:5]
                self.samples[0].append (self.m_x)
                self.samples[1].append (self.m_y)
                self.samples[2].append (self.m_z)
                if len(self.samples[0]) >= sample_count:
                    #print ("magnet samples = %s"%str(self.samples))
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
                        #print ("reset rotation bias because %.4g<%.4g and %.4g<%.4g"%(
                        #            deviation, max_sample_dev,
                        #            max_trend, max_trend_dev))
                    #else:
                    #    print ("DO NOT reset rotation bias because %.4g>=%.4g or %.4g>=%.4g"%(
                    #                deviation, max_sample_dev,
                    #                max_trend, max_trend_dev))
                    self.samples = [list(), list(), list()]
                    rot_object.reset_samples()

    def _send(self, secondary, rj, ts):
        if self.update_counts(secondary, rj):
            print ("magnet %.2f,%.2f,%.2f"%(self.m_x, self.m_y, self.m_z))
        self.timestamp = make_timestamp (ts)
        self.publish()

    def print_data(self):
        print ("magnet %.2f,%.2f,%.2f"%(self.m_x, self.m_y, self.m_z))
        self.print_stats()

class Pressure(MicroServerComs,SampleCounter):
    def __init__(self, pubsub_cfg):
        self.static_pressure = None
        self.timestamp = None
        MicroServerComs.__init__(self, "RawPressureSensors", channel='pressuresensors', config=pubsub_cfg)
        SampleCounter.__init__(self, 10)

    def send(self, args, calibration):
        self.static_pressure, ts, self.sample_count, secondary, rj = args
        self.static_pressure /= 1000.0
        self._send (secondary, rj, ts)

    def send2(self, values, ts, stats, calibration):
        self.static_pressure = values
        self.sample_count, secondary, rj = stats
        self.static_pressure /= 1000.0
        self._send (secondary, rj, ts)

    def _send(self, secondary, rj, ts):
        if self.update_counts(secondary, rj):
            print ("pressure %.2f"%(self.static_pressure))
        self.timestamp = make_timestamp (ts)
        self.publish()

    def print_data(self):
        print ("pressure %.2f"%(self.static_pressure))
        self.print_stats()

class Temperature(MicroServerComs,SampleCounter):
    def __init__(self, pubsub_cfg):
        self.temperature = None
        MicroServerComs.__init__(self, "RawTemperatureSensors", channel='temperaturesensors', config=pubsub_cfg)
        SampleCounter.__init__(self, 10)

    def send(self, args, calibration):
        self.temperature, self.sample_count, secondary, rj = args
        self._send (secondary, rj)

    def send2(self, values, stats, calibration):
        self.temperature = values
        self.sample_count, secondary, rj = stats
        self._send (secondary, rj)

    def _send(self, secondary, rj):
        if self.update_counts(secondary, rj):
            print ("temp %.1f"%(self.temperature))
        self.publish()

    def print_data(self):
        print ("temp %.1f"%(self.temperature))
        self.print_stats()

class Pitot(MicroServerComs,SampleCounter):
    def __init__(self, pubsub_cfg):
        self.pitot = None
        self.timestamp = None
        MicroServerComs.__init__(self, "RawPitotSensor", channel='pitotsensor', config=pubsub_cfg)
        SampleCounter.__init__(self, 10)

    def send(self, args):
        self.pitot, ts, self.sample_count, secondary, rj = args
        self._send (secondary, rj, ts)

    def send2(self, values, ts, stats):
        self.pitot = values
        self.sample_count, secondary, rj = stats
        self._send (secondary, rj, ts)

    def _send(self, secondary, rj, ts):
        if self.update_counts(secondary, rj):
            print ("pitot %.2f"%(self.pitot,))
        self.timestamp = make_timestamp (ts)
        self.publish()

    def print_data(self):
        print ("pitot %.2f"%(self.pitot,))
        self.print_stats()

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
        self.enable_printing = False
        MicroServerComs.__init__(self, "GPSFeed", channel='gpsfeed', config=pubsub_cfg)

    def send(self, args):
        nmea_string,ts = args
        ParseNMEAStrings (nmea_string, self)
        if self.HaveNewPosition and self.gps_ground_speed is not None and \
                self.gps_lat is not None:
            update_time_offset (self.gps_utc, ts)
            self.HaveNewPosition = False
            if self.enable_printing:
                self.print_vals()
            self.publish()

    def print_vals(self):
        print ("gps: %.1f,%.5f,%.5f,%.1f,%.1f,%.1f,%d"%(
                self.gps_utc,
                self.gps_lat,
                self.gps_lng,
                self.gps_altitude,
                self.gps_ground_speed,
                self.gps_ground_track,
                self.gps_signal_quality))

    def print_data(self):
        self.print_vals()
        self.print_stats()

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

slave = None

class hmi(Cmd):
    prompt = 'sensors: '
    def __init__(self, slave):
        Cmd.__init__(self)
        self.stop = False
        self.slave = slave

    def do_print(self, args):
        '''Print a summary of a sensor.
        Sensors can be: temp, pressure, accelerometers, gyro,
                        magnetometer, pitot.
        '''
        self.slave.print_sensor (args)

    def do_quit(self, args):
        'Exit the program'
        self.stop = True
        return True

    def do_mod(self, args):
        '''Modify a sensor parameter. Syntax: mod sensor_func parm_name val
        sensor_func matches one of the function args from the configuration,
            which is the second sensor config argument
        parm_name is one of: polling period (ms), f0, f1, sband,
                             rejection_band, sduration
        val is a floating point number representing the new value to use
        '''

        args = args.split()
        if len(args) != 3:
            print ("mod: Invalid number of arguments (should be 3)")
            return False
        self.slave.modify_sensor_parm (*tuple(args))


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
    command = hmi(slave)
    cmd_thread = threading.Thread (target=command.cmdloop)
    cmd_thread.start()

    while not command.stop:
        try:
            slave.ReadSensors()
        except Exception as e:
            try:
                _main_port.close()
                time.sleep(1)
                _main_port = serial.Serial(args.serial_port, 115200, timeout=0)
                command_channel.StartComs (_main_port)
            except:
                raise RuntimeError("Cannot re-open control/sensor port %s"%args.serial_port)

        slave.listen (timeout=0, loop=False)

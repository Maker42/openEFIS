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

import os
import sys,  math, datetime, time

import socket, logging
import argparse

import serial

if '__main__' == __name__:
    for p in sys.path:
        if 'Common' in p:
            break
    else:
        sys.path.append (os.path.join ('Common'))
        sys.path.append (os.path.join ('Test'))

import Globals, ArduinoCmdMessenger, CorrectedSensors

import Spatial, util

logger=logging.getLogger(__name__)

class SenseControlMaster:
    def __init__(self, localportno, command_host, command_port):
        self.localport = localportno
        self.command_host = command_host
        self.command_port = command_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind (("", localportno))
        self.sock.setblocking (0)
        self.Sensors = dict()

    def SetAnalogChannel(self, channel, val):
        cmd = "SetAnalogChannel (%d,%d)\n"%(channel, val)
        self.sock.sendto(cmd, (self.command_host, self.command_port))

    def SetDigitalChannel(self, channel, val):
        cmd = "SetDigitalChannel (%d,%d)\n"%(channel, val)
        self.sock.sendto(cmd, (self.command_host, self.command_port))

    def SetThrottles(self, *args):
        throttles = tuple(args)
        cmd = "SetThrottles %s\n"%str(throttles)
        logger.log (3, "Setting throttles to %g,%g,%g,%g  %g,%g", throttles[0],
                throttles[1],
                throttles[2],
                throttles[3],
                throttles[4],
                throttles[5],
                )
        self.sock.sendto(cmd, (self.command_host, self.command_port))

    def FlightParameters(self, *args):
        self.sock.sendto("FlightParameters %s\n"%str(*args), (self.command_host, self.command_port))

    def KnownAltitude(self, a):
        self.sock.sendto("KnownAltitude (%g)\n"%a, (self.command_host, self.command_port))

    def MagneticVariation(self, v):
        self.sock.sendto("MagneticVariation (%g)\n"%v, (self.command_host, self.command_port))

    def FlightMode(self, m, v):
        self.sock.sendto("FlightMode ('%s',%s)\n"%(m,v), (self.command_host, self.command_port))

    def WindVector(self, v):
        self.sock.sendto("WindVector (%s)\n"%str(v), (self.command_host, self.command_port))

    def Update(self):
        while True:
            try:
                rec = self.sock.recv(1024)
            except:
                rec = None
            if rec:
                lines = rec.split('\n')
                for index in range(-1,-len(lines)-1,-1):
                    try:
                        line = lines[index].strip()
                        if line:
                            self.Sensors = eval(line)
                            break
                    except Exception, e:
                        logger.error ("(%s): Bad receive: %s", str(e), lines[index])
            if len(self.Sensors):
                break
            time.sleep(.01)

class SenseControlSlave:
    def __init__(self, command_channel, master_sock, sensors, master_address, master_port):
        self._mainCmd = command_channel
        self._master_sock = master_sock
        self._master_address = master_address
        self._master_port = master_port
        self._sensors = sensors
        self._valid_sensor_commands = ["FlightParameters", "KnownAltitude", "MagneticVariation",
                "FlightMode", "WindVector"]
        self._valid_control_commands = ["SetAnalogChannel", "SetDigitalChannel", "SetThrottles"]

    def SetAnalogChannel(self, channel, val):
        self._mainCmd.sendValidatedCmd(Globals.CMD_ANALOG_SET_CHANNEL, channel, val)

    def SetDigitalChannel(self, channel, val):
        self._mainCmd.sendValidatedCmd(Globals.CMD_DIGITAL_SET_CHANNEL, channel, val)

    def SetThrottles(self, *throttles):
        global rootlogger
        self._mainCmd.sendValidatedCmd(Globals.CMD_SET_THROTTLES, *throttles)
        tst = tuple(throttles)
        rootlogger.log (3, "Setting throttles to %s", str(tst))

    def Update(self):
        global rootlogger
        while True:
            try:
                rec = self._master_sock.recv(1024)
            except:
                rec = None
            if rec:
                lines = rec.split('\n')
                for line in lines:
                    line = line.strip()
                    if len(line):
                        command = line.split(' ', 1)[0]
                        if command in self._valid_sensor_commands:
                            try:
                                eval ("self._sensors.%s"%line)
                            except Exception, e:
                                rootlogger.error ("%s: Bad sensor command: %s"%(str(e), line))
                        elif command in self._valid_control_commands:
                            try:
                                eval ("self.%s"%line)
                            except Exception, e:
                                rootlogger.error ("%s: Bad control command: %s"%(str(e), line))
                        else:
                            rootlogger.error ("Invalid command: %s"%line)
            else:
                break
        self._master_sock.sendto(str(self._sensors.Readings) + '\n',
                                (self._master_address, self._master_port))


if '__main__' == __name__:
    opt = argparse.ArgumentParser(description='Control/sensor slave process')
    opt.add_argument('serial-port', help = 'Serial port path of physical controller')
    opt.add_argument('--master-address', default='localhost', help='IP address of master')
    opt.add_argument('--master-port', default=49000, help = 'Port number of master')
    opt.add_argument('--local-port', default=49001, help = 'Local port number to use')
    opt.add_argument('-r', '--replay', default=None, help = 'Specify log to replay in simulation')
    opt.add_argument('-p', '--log-prefix', default=None, help = 'Over-ride logging prefix')
    opt.add_argument('-m', '--home', default=None, help = 'Over-ride home directory for objects and procedures')
    opt.add_argument('-c', '--record', action='store_true', help = 'Create a recording in simulation')
    opt.add_argument('-l', '--log-level', type=int, default=logging.WARNING, help = '1 = Maximum Logging. 100 = Absolute Silence. 40 = Errors only. 10 = Basic Debug')
    opt.add_argument('-v', '--magnetic-variation', default=None, help='The magnetic variation(declination) of the current position')
    opt.add_argument('-a', '--altitude', default=None, help='The currently known altitude')
    opt.add_argument('-w', '--wind', help="Wind speed and direction 'speed_knots,dir_deg'")
    args = opt.parse_args()

    if args.home:
        if os.path.isdir(args.home):
            os.environ['HOME'] = args.home
        else:
            print("Invalid home directory: " + args.home)
            sys.exit(1)
    rootlogger = logging.getLogger()
    rootlogger.setLevel(args.log_level)

    if not os.environ.has_key('HOME'):
        if os.environ.has_key('HOMEPATH'):
            os.environ['HOME'] = os.environ['HOMEDRIVE'] + os.environ['HOMEPATH']
        elif len(sys.argv) > 0:
            os.environ['HOME'],_ = os.path.split(sys.argv[0])
        elif os.path.isdir('C:\\'):
            os.environ['HOME'] = 'C:\\'
        else:
            os.environ['HOME'] = '/'

    dist_path = '/usr/local/lib/python2.7/dist-packages'
    if os.path.isdir(dist_path):
        sys.path.append('/usr/local/lib/python2.7/dist-packages')
    Globals.SimulationMode = Globals.LIVE_LOGGING
    if args.replay:
        Globals.SimulationMode = Globals.SIM_REPLAY
        Globals.LoggingPrefix = args.replay
        log_start = "REPLAY: Sensors with logging prefix %s"%Globals.LoggingPrefix
    else:
        if args.record:
            Globals.SimulationMode = Globals.SIM_RECORD
        datestamp = Globals.datestamp()
        Globals.LoggingPrefix = os.path.join('Logs', 'Sensors', datestamp)
        log_start = "Sensors beginning with logging prefix %s"%Globals.LoggingPrefix
    try:
        os.makedirs(Globals.LoggingPrefix)
    except:
        pass
    rootlogger.addHandler(logging.FileHandler(os.path.join(Globals.LoggingPrefix, 'info.log')))
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    rootlogger.addHandler(console_handler)
    rootlogger.log(99, log_start)

    master_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    master_sock.bind (("", args.local_port))
    master_sock.setblocking (0)

    sensors = CorrectedSensors.CorrectedSensors()
    try:
        _main_port = serial.Serial(args.serial_port, 115200, timeout=0)
        command_channel = ArduinoCmdMessenger.CmdMessenger(_main_port, arg_delimiter='^')
    except:
        # For test
        command_channel = ArduinoCmdMessenger.CmdMessenger(None, arg_delimiter='^')
        #raise RuntimeError("Cannot open control/sensor port %s"%args.serial_port)

    sensors.initialize (command_channel)

    if args.wind:
        try:
            speed,dr = args.wind.split(',')
            speed = float(speed)
            dr = float(dr)
            wv = Spatial.Vector(math.sin(dr * util.RAD_DEG) * speed, math.cos(dr * util.RAD_DEG) * speed, 0)
            sensors.WindVector (wv)
            rootlogger.info("Wind Vector = %s", str(wv))
        except Exception, e:
            rootlogger.error ("Invalid wind input: %s (%s)", args.wind, str(e))
    else:
        rootlogger.info("Wind = None")

    slave = SenseControlSlave(command_channel, master_sock, sensors, args.master_address, args.master_port)
    while True:
        sensors.Update()
        slave.Update()
        time.sleep(.01)

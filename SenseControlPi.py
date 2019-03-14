#!/usr/bin/env python3
# Copyright (C) 2019  Garrett Herschleb
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
import sys, time
import threading

import logging
import argparse

import yaml

import RPiSensors.sensors as sensors

import Globals

from SenseControlRemote import Accelerometers, Rotation, Magnetic
from SenseControlRemote import Pressure, Temperature, GPS, hmi

from MicroServerComs import MicroServerComs
from PubSub import assign_all_ports

logger=logging.getLogger(__name__)

class SenseControlSlave(MicroServerComs):
    def __init__(self, config, pubsub_cfg):
        self._config = config
        self._sensors = dict()
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
        self.last_trace_report = time.time()
        self.trace_lines = list()
        for config_term,cfg in self._config.items():
            if config_term.startswith ('cal'):
                self._calibrations.update (cfg)
            elif config_term.startswith ('sensors'):
                if not sensors.start(self._config):
                    raise RuntimeError ("Failed to start sensors")
        MicroServerComs.__init__(self, "ControlSlave", config=pubsub_cfg)

    def ReadSensors(self):
        sensors.sample_sensors()
        temp,stats,_ = sensors.read_temperature()
        if temp is not None:
            self._temperature.send2(temp[0], stats,
                        self._calibrations.get('temperature'))
        pres,stats,ts = sensors.read_pressure()
        if pres is not None:
            self._pressure.send2(pres[0], ts, stats,
                        self._calibrations.get('pressure'))
        magnetic,stats,ts = sensors.read_magnetic()
        if magnetic is not None:
            cal = self._calibrations.get('magnetic')
            self._magnetic.send2(magnetic, ts, stats, cal, self._rotation)
        accel,stats,ts = sensors.read_accelerometer()
        if accel is not None:
            self._accelerometers.send2(accel, ts, stats,
                        self._calibrations.get('accelerometers'))
        gyro,stats,ts = sensors.read_gyroscope()
        if gyro is not None:
            self._rotation.send2(gyro, ts, stats,
                        self._calibrations.get('rotation'))

    def modify_sensor_parm(self, sname, pname, val):
        sensors.modify_sensor_parm (sname, pname, val)

    def print_sensor(self, name):
        sensors.print_sensor(name)

    def collect_raw (self, sname, seconds, filename):
        sensors.collect_raw (sname, seconds, filename)

if '__main__' == __name__:
    opt = argparse.ArgumentParser(description='Control/sensor slave process for Raspberry Pi')
    opt.add_argument('starting_port', type=int,
            help='The port number to start with for serialized network port assignments')
    opt.add_argument('-c', '--config-file', default='sensors.yml', help='YAML config file for sensor / control board')
    opt.add_argument('-p', '--pubsub-config', default='sensors_pubsub.yml', help='YAML config file coms configuration')
    opt.add_argument('--log-prefix', default=None, help = 'Over-ride logging prefix')
    opt.add_argument('-l', '--log-level', type=int, default=logging.WARNING, help = '1 = Maximum Logging. 100 = Absolute Silence. 40 = Errors only. 10 = Basic Debug')
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

    with open(args.config_file, 'r') as yml:
        config = yaml.load (yml)
        yml.close()
    with open (args.pubsub_config, 'r') as yml:
        pubsub_config = yaml.load(yml)
        assign_all_ports (pubsub_config, args.starting_port)
        yml.close()
    slave = SenseControlSlave(config, pubsub_config)
    command = hmi(slave)
    cmd_thread = threading.Thread (target=command.cmdloop)
    cmd_thread.start()

    while not command.stop:
        slave.ReadSensors()
        slave.listen (timeout=0.01, loop=False)

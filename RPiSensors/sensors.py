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


import time

from smbus import SMBus

from RPiSensors.BMP280 import BMP280
from RPiSensors.MPU9250 import MPU9250

class SensorChannel:
    def __init__(self, sample, period, coefficients, bands, secondary_duration):
        self.sample_func = sample
        self.period = period
        self.coefficients = coefficients
        self.bands = bands
        self.secondary_duration = secondary_duration
        self.state = None
        self.next_time = 0
        self.last_sample_time = 0
        self.secondary_filter_count = 0
        self.reset_counts()

    def reset_counts(self):
        self.sample_count = 0
        self.reject_count = 0
        self.secondary_use_count = 0

    def filter_values (self, values):
        if self.state is None:
            self.state = [v for v in values]
        else:
            trigger_secondary = False
            filt = 0
            diff = [(i-s) for i,s in zip(values,self.state)]
            adiff = [abs(d) for d in diff]
            max_adiff = max(adiff)
            if max_adiff > self.bands[1]:
                self.reject_count += 1
                if ((self.reject_count > 9) and
                    (self.secondary_filter_count + self.sample_count <
                        self.reject_count)):
                    # Reset filter state
                    self.state = [v for v in values]
                    self.reset_counts()
                    return
            elif max_adiff > self.bands[0]: trigger_secondary = True
            if trigger_secondary: self.secondary_filter_count = 1
            if self.secondary_filter_count > 0:
                filt = 1
                self.secondary_use_count += 1
            self.state = [(s+self.coefficients[filt]*d)
                            for s,d in zip(self.state,diff)]
            if self.secondary_filter_count > 0:
                self.secondary_filter_count += 1
                if self.secondary_filter_count > self.secondary_duration:
                    self.secondary_filter_count = 0
            self.sample_count += 1

    def sample(self):
        values = self.sample_func()
        if values is not None:
            self.filter_values (values)
            self.last_sample_time = time.time()

    def read(self):
        tm = time.time()
        if tm > self.next_time:
            stats = [self.sample_count, self.secondary_use_count, self.reject_count]
            self.reset_counts()
            self.next_time = tm + self.period
            return self.state,stats,self.last_sample_time
        else:
            return None,None,None


base_sensor_suite = dict()
sensor_objects = dict()

def start(config):
    global base_sensor_suite
    ado = 0 if 'ado' not in config else config['ado']
    base_sensor_suite[9250] = MPU9250(ado=ado)
    base_sensor_suite[280] = BMP280(ado=ado)
    devnum = 1 if 'devnum' not in config else config['devnum']

    # Open i2c bus
    bus = SMBus(devnum)
    if not base_sensor_suite[9250].begin(bus):
        return False
    if not base_sensor_suite[280].begin(bus):
        return False

    for measurement,parms in config['sensors'].items():
        if 'pressure' == measurement:
            sample_func = base_sensor_suite[280].readPressure
        elif 'temperature' == measurement:
            sample_func = base_sensor_suite[280].readTemperature
        elif 'accel' == measurement:
            sample_func = base_sensor_suite[9250].readAccel
        elif 'gyro' == measurement:
            sample_func = base_sensor_suite[9250].readGyro
        elif 'magnet' == measurement:
            sample_func = base_sensor_suite[9250].readMagnetometer
        else:
            raise RuntimeError ("sensor type %s from config not found"%measurement)


        sensor_objects[measurement] = SensorChannel (sample_func,
                        parms['period'],
                        [parms['filter_coefficient1'],
                        parms['filter_coefficient2']],
                        [parms['secondary_band'],
                        parms['rejection_band']],
                        parms['secondary_filter_duration'])
    return True

def sample_sensors():
    for o in sensor_objects.values():
        o.sample()

def read_magnetic():
    if 'magnet' in sensor_objects:
        return sensor_objects['magnet'].read()
    else:
        return None,None,None

def read_gyroscope():
    return sensor_objects['gyro'].read()

def read_accelerometer():
    return sensor_objects['accel'].read()

def read_temperature():
    return sensor_objects['temperature'].read()

def read_pressure():
    return sensor_objects['pressure'].read()

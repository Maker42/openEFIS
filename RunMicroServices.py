#!/usr/bin/env python3
# Copyright (C) 2018-2019  Garrett Herschleb
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

import sys, os
import argparse

import yaml

from PitchEstimate import PitchEstimate
from GroundRoll import GroundRoll
from RollEstimate import RollEstimate
from RollRateEstimate import RollRateEstimate
from TurnRateComputed import TurnRateComputed
from HeadingComputed import HeadingComputed
from Pitch import Pitch
from Roll import Roll
from Yaw import Yaw
from Heading import Heading
from RollRate import RollRate
from HeadingTasEstimate import HeadingTasEstimate
from WindEstimate import WindEstimate
from PressureFactors import PressureFactors
from AirspeedComputed import AirspeedComputed
from AirspeedEstimate import AirspeedEstimate
from AltitudeComputed import AltitudeComputed
from TrackRate import TrackRate
from TurnRate import TurnRate
from Airspeed import Airspeed
from Altitude import Altitude
from ClimbRate import ClimbRate
from GroundVector import GroundVector
from ClimbRateEstimate import ClimbRateEstimate
from PitchRate import PitchRate
import InternalPublisher
import MicroServerComs
from PubSub import CONFIG_FILE, assign_all_ports

def run_service(so):
    so.listen()

if __name__ == "__main__":
    opt = argparse.ArgumentParser(description=
            'Run the microservices necessary for a complete, self checking AHRS computation pipeline')
    opt.add_argument('starting_port', type=int,
            help='The port number to start with for serialized network port assignments')
    opt.add_argument('-p', '--pubsub-config', default=CONFIG_FILE,
            help='YAML config file coms configuration')
    opt.add_argument('-a', '--airspeed-config', default='airspeed_curve.yml',
            help='YAML config file pressure differential->airspeed curve')
    opt.add_argument('-m', '--heading-calibration', default='heading_calibration.yml',
            help='YAML config file magnetic heading calibration curve')
    opt.add_argument('-r', '--pressure-calibration', default='pressure_calibration.yml',
            help='YAML config file altitude calibration curve')
    opt.add_argument('-c', '--accelerometer-calibration', default='accelerometer_calibration.yml',
            help='YAML config file accelerometer calibration curve')
    opt.add_argument('-y', '--yaw-multiplier', type=float, default=1.0,
            help='a_x * yaw_multiplier = yaw reading')
    opt.add_argument('-g', '--gyro-correction', type=float, default=1.0,
         help='Degrees per minute roll and pitch drift toward computed flight estimates')
    opt.add_argument('--roll-conf-mult', type=float, default=1.0,
         help='roll confidence = degrees variation * roll_conf_mult')
    opt.add_argument('--pitch-conf-mult', type=float, default=1.0,
         help='pitch confidence = degrees variation * pitch_conf_mult')
    args = opt.parse_args()

    with open (args.pubsub_config, 'r') as yml:
        MicroServerComs._pubsub_config = yaml.load(yml)
        assign_all_ports (MicroServerComs._pubsub_config, args.starting_port)
        yml.close()
    InternalPublisher.TheInternalPublisher = InternalPublisher.InternalPublisher(
            MicroServerComs._pubsub_config)
    if os.path.exists(args.airspeed_config):
        with open (args.airspeed_config, 'r') as yml:
            airspeed_config = yaml.load(yml)
            yml.close()
    else:
        airspeed_config = dict()
    airspeed_config['rais_id'] = args.starting_port

    heading_calibration = None
    if os.path.exists(args.heading_calibration):
        with open (args.heading_calibration, 'r') as yml:
            heading_calibration = yaml.load(yml)
            yml.close()
        heading_calibration['rais_id'] = args.starting_port
    pressure_calibration = None
    if os.path.exists(args.pressure_calibration):
        with open (args.pressure_calibration, 'r') as yml:
            pressure_calibration = yaml.load(yml)
            yml.close()
        pressure_calibration['rais_id'] = args.starting_port
    accelerometer_calibration = None
    if os.path.exists(args.accelerometer_calibration):
        with open (args.accelerometer_calibration, 'r') as yml:
            accelerometer_calibration = yaml.load(yml)
            yml.close()
        accelerometer_calibration['rais_id'] = args.starting_port

    service_objects = [
                 PitchEstimate(accelerometer_calibration)
                ,GroundRoll(accelerometer_calibration)
                ,RollEstimate()
                ,RollRateEstimate()
                ,TurnRateComputed()
                ,HeadingComputed(heading_calibration, args.starting_port)
                ,Pitch(args.gyro_correction, args.pitch_conf_mult)
                ,Roll(args.gyro_correction, args.roll_conf_mult)
                ,Yaw(accelerometer_calibration, args.yaw_multiplier)
                ,Heading()
                ,RollRate()
                ,HeadingTasEstimate()
                ,WindEstimate()
                ,PressureFactors(pressure_calibration)
                ,AirspeedComputed(airspeed_config)
                ,AirspeedEstimate()
                ,AltitudeComputed(pressure_calibration)
                ,TrackRate()
                ,TurnRate()
                ,Airspeed()
                ,Altitude()
                ,ClimbRate()
                ,GroundVector()
                ,ClimbRateEstimate()
                ,PitchRate()
                ]
    InternalPublisher.TheInternalPublisher.listen()

# Copyright (C) 2018  Garrett Herschleb
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
import argparse

import yaml

from SenseControl import Sensors

if __name__ == "__main__":
    opt = argparse.ArgumentParser(description='Send weather info to sensors')
    opt.add_argument('-v', '--magnetic-variation', default=None, help='The magnetic variation(declination) of the current position')
    opt.add_argument('-a', '--altitude', default=None, type=int, help='The currently known altitude')
    opt.add_argument('-b', '--barometer', default=None, type=float, help='The given barometric pressure in inches of mercury')
    opt.add_argument('-s', '--wind-speed', default=None, type=int, help='The current wind speed in knots')
    opt.add_argument('--wind-heading', default=None, type=int, help='The current wind heading in degrees')
    opt.add_argument('-p', '--pubsub-config', default='sensors_pubsub.yml', help='YAML config file coms configuration')
    args = opt.parse_args()

    with open (args.pubsub_config, 'r') as yml:
        pubsub_config = yaml.load(yml)
        yml.close()
    s = Sensors(pubsub_config)
    s.initialize (args.altitude, args.barometer, (args.wind_heading, args.wind_speed))
    s.WaitSensorsGreen()


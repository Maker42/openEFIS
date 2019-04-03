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

import sys, time
import subprocess
import yaml

def get_calibrations(i, accel_calibrations, magnet_calibrations,
        pressure_calibrations, airspeed_calibrations, alat_multipliers):
    a = None
    if len(accel_calibrations) > i:
        a = accel_calibrations[i]
    m = None
    if len(magnet_calibrations) > i:
        m = magnet_calibrations[i]
    p = None
    if len(pressure_calibrations) > i:
        p = pressure_calibrations[i]
    s = None
    if len(airspeed_calibrations) > i:
        s = airspeed_calibrations[i]
    y = None
    if len(alat_multipliers) > i:
        y = alat_multipliers[i]
    return [a, m, p, s, y]

if __name__ == "__main__":
    with open (sys.argv[1], 'r') as yml:
        config = yaml.load(yml)
        yml.close()
    pubsub_file = config['pubsub_config']
    port_bases = config['port_bases']
    rais_pubsub = config['rais_pubsub']
    rais_port = config['rais_port']
    accel_calibrations = list()
    alat_multipliers = list()
    if 'accel_calibrations' in config:
        accel_calibrations = config['accel_calibrations']
    magnet_calibrations = list()
    if 'magnet_calibrations' in config:
        magnet_calibrations = config['magnet_calibrations']
    pressure_calibrations = list()
    if 'pressure_calibrations' in config:
        pressure_calibrations = config['pressure_calibrations']
    airspeed_calibrations = list()
    if 'airspeed_calibrations' in config:
        airspeed_calibrations = config['airspeed_calibrations']
    if 'alat_multipliers' in config:
        alat_multipliers = config['alat_multipliers']
    python = '/usr/bin/python3' if 'python' not in config else config['python']
    pubsub = 'PubSub.py' if 'pubsub' not in config else config['pubsub']
    runms = 'RunMicroServices.py' if 'runms' not in config else config['runms']
    rais = 'RAISDiscriminator.py' if 'rais' not in config else config['rais']
    pubsub_processes = list()
    ms_processes = list()
    for i,p in enumerate(port_bases):
        args = [python, pubsub, pubsub_file, str(p)]
        print ("Spawning: %s"%str(args))
        proc = subprocess.Popen (args,
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        pubsub_processes.append(proc)

        args = [python, runms, str(p), '-p', pubsub_file]
        cals = get_calibrations(i, accel_calibrations, magnet_calibrations,
                pressure_calibrations, airspeed_calibrations, alat_multipliers)
        for arg_prefix,arg in zip(['-c', '-m', '-r', '-a', '-y'], cals):
            if arg is not None:
                args.append(arg_prefix)
                args.append(arg)
        print ("Spawning: %s"%str(args))
        proc = subprocess.Popen (args,
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        ms_processes.append(proc)
    args = [python, pubsub, rais_pubsub, str(rais_port)]
    print ("Spawning: %s"%str(args))
    proc = subprocess.Popen (args,
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    pubsub_processes.append(proc)
    rais_args = [python, rais]
    for p in port_bases:
        rais_args.append(str(p))
        rais_args.append(pubsub_file)
    rais_args.append (str(rais_port))
    rais_args.append (rais_pubsub)
    print ("Spawning: %s"%str(rais_args))
    rais_proc = subprocess.Popen (rais_args)

    while True:
        time.sleep(1)

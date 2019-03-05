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

if __name__ == "__main__":
    with open (sys.argv[1], 'r') as yml:
        config = yaml.load(yml)
        yml.close()
    pubsub_file = config['pubsub_config']
    port_bases = config['port_bases']
    rais_pubsub = config['rais_pubsub']
    rais_port = config['rais_port']
    python = '/usr/bin/python3' if 'python' not in config else config['python']
    pubsub = 'PubSub.py' if 'pubsub' not in config else config['pubsub']
    runms = 'RunMicroServices.py' if 'runms' not in config else config['runms']
    rais = 'RAISDiscriminator.py' if 'rais' not in config else config['rais']
    pubsub_processes = list()
    ms_processes = list()
    for p in port_bases:
        args = [python, pubsub, pubsub_file, str(p)]
        print ("Spawning: %s"%str(args))
        proc = subprocess.Popen (args,
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        pubsub_processes.append(proc)

        args = [python, runms, str(p), '-p', pubsub_file]
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

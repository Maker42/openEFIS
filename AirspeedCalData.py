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

import sys,os

import yaml

from MicroServerComs import MicroServerComs

from PubSub import assign_all_ports

class AirspeedCalData(MicroServerComs):
    def __init__(self, cfg):
        MicroServerComs.__init__(self, "AdminCommand", channel='admincommand', config=cfg)

    def send(self, a):
        self.command = b'airspeed'
        self.args = bytes(a, 'utf-8')
        print ("Airspeed %s published %s"%(a, str(self.pubchannel.getpeername())))
        self.publish()

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print ("Usage: AirspeedCalData.py <starting_port> <pubsub.yaml> <current_ias>")
        sys.exit(-1)
    with open (sys.argv[2], 'r') as yml:
        config = yaml.load (yml)
        yml.close ()
        starting_port = int(sys.argv[1])
        ias = float(sys.argv[3])
        assign_all_ports (config, starting_port)

        aspd = AirspeedCalData (config)
        aspd.send(sys.argv[3])

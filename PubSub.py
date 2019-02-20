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

import socket, select, sys
import threading

import yaml

MAX_DATA_SIZE=4096
CONFIG_FILE='sensors_pubsub.yml'

def run_channel (*args, **kwargs):
    chconfig = args[0]
    chname = args[1]
    pubs_cfg = chconfig['pubs']
    subs_cfg = chconfig['subs']
    # Listeners can be TCP listening ports for either pubs or subs,
    # or UDP pub ports
    listeners = dict()
    select_list = list()
    exception_list = list()
    subs = dict()
    if pubs_cfg is not None:
        for pipe in pubs_cfg:
            protocol = pipe['protocol']
            if protocol == 'udp':
                port = pipe['port']
                usock = socket.socket (type=socket.SOCK_DGRAM)
                try:
                    usock.bind(('',port))
                    print ("%s/%s udp bound to %d"%(chname,pipe['function'], port))
                except Exception as e:
                    raise RuntimeError ("port %d bind error %s"%(port, str(e)))
                listeners[usock.fileno()] = (usock, protocol, 'p')
                select_list.append (usock.fileno())
                exception_list.append (usock.fileno())
                #print ("Created udp pub socket for %s"%pipe['function'])
            elif protocol == 'tcp':
                port = pipe['port']
                tsock = socket.socket (type=socket.SOCK_STREAM)
                tsock.bind(('',port))
                print ("%s/%s listening to %d"%(chname,pipe['function'], port))
                tsock.listen(10)
                listeners[tsock.fileno()] = (tsock, 'tcplisten', 'p')
                select_list.append (tsock.fileno())
                exception_list.append (tsock.fileno())
                #print ("Created TCP pub listener socket for %s"%pipe['function'])

    if subs_cfg is not None:
        for pipe in subs_cfg:
            protocol = pipe['protocol']
            if protocol == 'udp':
                port = pipe['port']
                usock = socket.socket (type=socket.SOCK_DGRAM)
                addr = pipe['addr']
                try:
                    usock.bind ((addr,port-1000))
                    usock.connect((addr,port))
                except Exception as e:
                    print ("Could not connect to %s:%d for channel %s, function %s"%(
                        addr, port, chname, pipe['function']))
                    continue
                subs[usock.fileno()] = (usock, chname, pipe['function'])
                exception_list.append (usock.fileno())
                print ("Created UDP sub socket connected to %s:%d for %s"%(addr, port, pipe['function']))
            elif protocol == 'tcp':
                port = pipe['port']
                tsock = socket.socket (type=socket.SOCK_STREAM)
                tsock.bind(('',port))
                tsock.listen(10)
                listeners[tsock.fileno()] = (tsock, 'tcplisten', 's')
                select_list.append (tsock.fileno())
                exception_list.append (tsock.fileno())
                print ("Created TCP sub listener socket for %s"%pipe['function'])

    if len(select_list) == 0:
        print ("No external connections for channel %s. This thread quiting"%chname)
        return

    while True:
        r,w,x = select.select (select_list, [], exception_list)
        for p in x:
            if p in listeners:
                s,protocol,role = listeners[p]
                if protocol == 'tcplisten':
                    raise RuntimeError ("Unexpected exception on a pub listen socket")
                elif protocol == 'udp':
                    raise RuntimeError ("Unexpected exception on a pub UDP socket")
                else:
                    del listeners[p]
            elif p in subs:
                del subs[p]

        for p in r:
            s,protocol,role = listeners[p]
            if protocol == 'tcplisten':
                newsock = s.accept()
                if role == 's':
                    subs[newsock.fileno()] = (newsock, 'tcp', role)
                else:
                    listeners[newsock.fileno()] = (newsock, 'tcp', role)
                    select_list.append (newsock.fileno())
                exception_list.append (newsock.fileno())
            else:
                message,frm = s.recvfrom(MAX_DATA_SIZE)
                for fileno,(sock,chname,function) in subs.items():
                    try:
                        #print ("PubSub sending to %s"%chname)
                        sock.sendall (message)
                    except:
                        pass


def assign_ports(chcfg, portno):
    pubs_cfg = chcfg['pubs']
    subs_cfg = chcfg['subs']
    if pubs_cfg is not None:
        for pipe in pubs_cfg:
            protocol = pipe['protocol']
            if protocol == 'udp' or protocol == 'tcp':
                if not 'port' in pipe:
                    pipe['port'] = portno
                    portno += 1
    if subs_cfg is not None:
        for pipe in subs_cfg:
            protocol = pipe['protocol']
            if protocol == 'udp' or protocol == 'tcp':
                if not 'port' in pipe:
                    pipe['port'] = portno
                    portno += 1

    return portno

if __name__ == "__main__":
    starting_port = 49010
    if len(sys.argv) > 1:
        cfg_file = sys.argv[1]
        if len(sys.argv) > 2:
            starting_port = int(sys.argv[2])
    else:
        cfg_file = CONFIG_FILE
    with open (cfg_file, 'r') as yml:
        config = yaml.load (yml)
        yml.close ()
        channels = list()
        print ("Starting port: %d"%starting_port)
        ckeys = [k for k in config.keys()]
        ckeys.sort()
        for chname in ckeys:
            chcfg = config[chname]
            starting_port = assign_ports(chcfg, starting_port)
            ch = threading.Thread (target=run_channel, args=(chcfg,chname))
            ch.start()
            print ("Created thread for %s"%chname)
            channels.append(ch)
        print ("Final port: %d"%starting_port)
        for ch in channels:
            ch.join()

def assign_all_ports (cfg, starting_port):
    ckeys = [k for k in cfg.keys()]
    ckeys.sort()
    for chname in ckeys:
        chcfg = cfg[chname]
        starting_port = assign_ports(chcfg, starting_port)

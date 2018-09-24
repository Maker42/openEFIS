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

import os, sys
import socket, select
import threading

import yaml

MAX_DATA_SIZE=4096
CONFIG_FILE='sensors_pubsub_internal.yml'

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
    for pipe in pubs_cfg:
        protocol = pipe['protocol']
        port = pipe['port']
        if protocol == 'udp':
            usock = socket.socket (type=socket.SOCK_DGRAM)
            usock.bind(('',port))
            listeners[usock.fileno()] = (usock, protocol, 'p')
            select_list.append (usock.fileno())
            exception_list.append (usock.fileno())
            print ("Created udp pub socket for %s"%pipe['function'])
        elif protocol == 'tcp':
            tsock = socket.socket (type=socket.SOCK_STREAM)
            tsock.bind(('',port))
            tsock.listen(10)
            listeners[tsock.fileno()] = (tsock, 'tcplisten', 'p')
            select_list.append (tsock.fileno())
            exception_list.append (tsock.fileno())
            print ("Created TCP pub listener socket for %s"%pipe['function'])

    for pipe in subs_cfg:
        protocol = pipe['protocol']
        port = pipe['port']
        if protocol == 'udp':
            usock = socket.socket (type=socket.SOCK_DGRAM)
            addr = pipe['addr']
            #usock.bind(('',port-1000))
            try:
                usock.connect((addr,port))
            except Exception as e:
                print ("Could not connect to %s:%d for channel %s, function %s"%(
                    addr, port, chname, pipe['function']))
                continue
            subs[usock.fileno()] = (usock, chname, pipe['function'])
            exception_list.append (usock.fileno())
            print ("Created UDP sub socket for %s"%pipe['function'])
        elif protocol == 'tcp':
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
                message = s.recv(MAX_DATA_SIZE)
                todelete = list()
                #print ("PubSub received from %s"%role)
                for fileno,(sock,chname,function) in subs.items():
                    try:
                        #print ("PubSub sending to %s"%chname)
                        sock.sendall (message)
                    except Exception as e:
                        #print ("sock channel %s, function %s, unable to send (%s)"%(
                        #   str(e), chname, function))
                        todelete.append (fileno)
                for f in todelete:
                    del subs[f]
                    del exception_list[exception_list.index(f)]



if __name__ == "__main__":
    with open (CONFIG_FILE, "r") as yml:
        config = yaml.load (yml)
        yml.close ()
        channels = list()
        for chname,chcfg in config.items():
            ch = threading.Thread (target=run_channel, args=(chcfg,chname))
            ch.start()
            print ("Created thread for %s"%chname)
            channels.append(ch)
        for ch in channels:
            ch.join()

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

import socket, select, struct, time
import yaml

from PubSub import MAX_DATA_SIZE, CONFIG_FILE
import InternalPublisher
_pubsub_config = None

class MicroServerComs:
    def __init__(self, function, input_mode='injection', channel=None, timeout=None):
        global _pubsub_config
        self.pubchannel = None
        self.output_values = None
        self.output_format = None
        self.subchannels = dict()
        self.has_internal_listeners = False
        self.has_external_listeners = False
        self.function = function
        if channel is None:
            self.channel = self.function
        else:
            self.channel = channel
        self.input_mode = input_mode
        if _pubsub_config is None:
            with open (CONFIG_FILE, 'r') as yml:
                _pubsub_config = yaml.load(yml)
                yml.close()
        if _pubsub_config is None:
            raise RuntimeError ("MicroserverComs: pubsub config %s invalid"%CONFIG_FILE)
        for chname,chcfg in _pubsub_config.items():
            pubs_cfg = chcfg['pubs']
            #print ("pubs_cfg = %s"%str(pubs_cfg))
            subs_cfg = chcfg['subs']
            #print ("subs_cfg = %s"%str(subs_cfg))
            if chname == self.channel:
                # Describing my listeners
                if subs_cfg is not None:
                    for pipe in subs_cfg:
                        if pipe['protocol'] == 'internal':
                            self.has_internal_listeners = True
                        else:
                            self.has_external_listeners = True
                if pubs_cfg is not None:
                    for pipe in pubs_cfg:
                        if self.function == pipe['function']:
                            if self.pubchannel:
                                raise RuntimeError ("Duplicate pub channel found for \
                                        function %s"%pipe['function'])
                            protocol = pipe['protocol']
                            if protocol != 'internal':
                                port = pipe['port']
                                addr = pipe['addr']
                                self.output_values = chcfg['output_values']
                                self.output_format = chcfg['format']
                                if protocol == 'udp':
                                    self.pubchannel = socket.socket(type=socket.SOCK_DGRAM)
                                else:
                                    self.pubchannel = socket.socket(type=socket.SOCK_STREAM)
                                self.pubchannel.connect ((addr,port))
            else:
                if subs_cfg is not None:
                    for pipe in subs_cfg:
                        if self.function == pipe['function']:
                            protocol = pipe['protocol']
                            if protocol != 'internal':
                                port = pipe['port']
                                addr = pipe['addr']
                                if protocol == 'udp':
                                    subchannel = socket.socket(type=socket.SOCK_DGRAM)
                                else:
                                    subchannel = socket.socket(type=socket.SOCK_STREAM)
                                subchannel.bind ((addr,port))
                                subchannel.settimeout (timeout)
                                self.subchannels[subchannel.fileno()] = ((subchannel,self.channel,chname
                                                                          ,chcfg['output_values']
                                                                          ,chcfg['format']
                                                                          ))
        if InternalPublisher.TheInternalPublisher is not None:
            InternalPublisher.TheInternalPublisher.register_channel (self.channel, self, self.subchannels)

    def __str__(self):
        return "%s: pub=%s, subs=%s"%(self.function, str(self.pubchannel), str(self.subchannels))

    def listen(self, timeout=None, loop=True):
        rsocks = list(self.subchannels.keys())
        if self.pubchannel is not None:
            xsocks = rsocks + [self.pubchannel.fileno()]
        else:
            xsocks = rsocks
        while True:
            got_data = False
            if timeout is None:
                r,w,x = select.select (rsocks, [], xsocks)
            else:
                r,w,x = select.select (rsocks, [], xsocks, timeout)
            for errfd in x:
                if errfd in self.subchannels:
                    raise RuntimeError ()
                elif errfd == self.pubchannel.fileno():
                    raise RuntimeError ()
                else:
                    raise RuntimeError ()
            for rfd in r:
                self.data_ready(rfd)
                got_data = True
            if not (loop or got_data):
                break

    def data_ready(self, rfd):
        if rfd in self.subchannels:
            s,mychname,from_chname,input_values,input_format = self.subchannels[rfd]
            data = s.recv(MAX_DATA_SIZE)
            values_list = struct.unpack (input_format, data)
            if self.input_mode == 'injection':
                timestamped = False
                ts_name = from_chname + '_updated'
                for vname,value in zip(input_values,values_list):
                    if vname == 'timestamp':
                        # Special timestamp handling
                        setattr (self, ts_name, value)
                        timestamped = True
                        self.last_update_time = value
                    else:
                        setattr (self, vname, value)
                if not timestamped:
                    setattr (self, ts_name, time.time())
                self.updated (from_chname)
            else:   # Input list mode
                self.input (from_chname, input_values, values_list)

    def publish(self):
        if self.has_external_listeners:
            if self.pubchannel is None:
                raise RuntimeError ("Function %s has no pub channel"%self.function)
            pack_args = (self.output_format, )
            for vname in self.output_values:
                pack_args = pack_args + (getattr (self, vname),)
            try:
                message = struct.pack (*pack_args)
            except Exception as e:
                print ("Struct packing error %s: args %s"%(str(e), str(pack_args)))
                raise
            #print ("%s connected sends %s"%(self.function, str(message)))
            self.pubchannel.sendall (message)
            #print ("External publish %s to function %s, file %d"%(self.output_values, self.function,
            #    self.pubchannel.fileno()))
        if self.has_internal_listeners:
            InternalPublisher.TheInternalPublisher.publish (self.channel)

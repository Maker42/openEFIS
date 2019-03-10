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

import socket, select, struct, time
import yaml

from PubSub import MAX_DATA_SIZE, CONFIG_FILE
import InternalPublisher
_pubsub_config = None

class MicroServerComs:
    def __init__(self, function, input_mode='injection', channel=None, timeout=None, config=None):
        global _pubsub_config
        self.pubchannel = None
        self.output_values = None
        self.output_format = None
        self.subchannels = dict()
        self.has_internal_listeners = False
        self.has_external_listeners = False
        self.function = function
        self.multi_receiver = False
        self.timeout = timeout
        if channel is None:
            self.channel = self.function
        else:
            self.channel = channel
        self.input_mode = input_mode
        if config is None:
            if _pubsub_config is None:
                with open (CONFIG_FILE, 'r') as yml:
                    _pubsub_config = yaml.load(yml)
                    yml.close()
            if _pubsub_config is None:
                raise RuntimeError ("MicroserverComs: pubsub config %s invalid"%CONFIG_FILE)
            config = _pubsub_config
        if isinstance(config,list):
            self.multi_receiver = True
            for i,c in enumerate(config):
                self.init_config(c,i, timeout)
        else:
            self.init_config(config,0, timeout)

    def init_config(self, config, cfg_index, timeout):
        for chname,chcfg in config.items():
            pubs_cfg = chcfg['pubs']
            #print ("%s pubs_cfg = %s"%(chname, str(pubs_cfg)))
            subs_cfg = chcfg['subs']
            #print ("%s subs_cfg = %s"%(chname, str(subs_cfg)))
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
                                #self.pubchannel.bind (('',port-1000))
                                self.pubchannel.connect ((addr,port))
                                print ("Channel %s connecting to %s:%d"%(self.channel, addr, port))
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
                                try:
                                    subchannel.bind ((addr,port))
                                    print ("%s/%s bound to %s:%d"%(self.channel,self.function, addr,port))
                                except Exception as e:
                                    print ("Error: cannot bind %s/%s socket to %s,%d: %s"%(self.channel,self.function, addr,port,str(e)))
                                subchannel.settimeout (timeout)
                                self.subchannels[subchannel.fileno()] = ((subchannel,self.channel,chname
                                                                          ,chcfg['output_values']
                                                                          ,chcfg['format']
                                                                          ,cfg_index
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
                try:
                    r,w,x = select.select (rsocks, [], xsocks, timeout)
                except:
                    print ("select error. rsocks=%s, xsocks=%s, timeout=%g"%(rsocks, xsocks, timeout))
                    raise
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
            if not loop:
                break
            if not got_data:
                break

    def inject(self, attr, val, cfg_index):
        if self.multi_receiver:
            d = getattr(self, attr)
            d[cfg_index] = val
        else:
            setattr (self, attr, val)

    def data_ready(self, rfd):
        if rfd in self.subchannels:
            s,mychname,from_chname,input_values,input_format,cfg_index = self.subchannels[rfd]
            while True:
                try:
                    data,addr = s.recvfrom(MAX_DATA_SIZE)
                except BlockingIOError:
                    break
                try:
                    values_list = struct.unpack (input_format, data)
                except Exception as e:
                    print (
                    ("MSCom: receive unpack error for %s from %s (%s). Got %d bytes"%(
                        self.function, from_chname, str(e), len(data))))
                    return
                if self.input_mode == 'injection':
                    timestamped = False
                    ts_name = from_chname + '_updated'
                    for vname,value in zip(input_values,values_list):
                        if vname == 'timestamp':
                            # Special timestamp handling
                            self.inject (ts_name, value, cfg_index)
                            timestamped = True
                            self.last_update_time = value
                        else:
                            self.inject (vname, value, cfg_index)
                    if not timestamped:
                        self.inject (ts_name, time.time(), cfg_index)
                    if self.multi_receiver:
                        self.updated (from_chname, cfg_index)
                    else:
                        self.updated (from_chname)
                else:   # Input list mode
                    self.input (from_chname, input_values, values_list)
                if self.timeout is None:
                    break

    def publish(self,debug=False):
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
            #if debug: print ("%s connected sends %s"%(self.function, str(message)))
            try:
                self.pubchannel.sendall (message)
            except:
                pass
            if debug:
                print ("External publish %s to function %s, file %d"%(self.output_values, self.function,
                        self.pubchannel.fileno()))
        if self.has_internal_listeners:
            InternalPublisher.TheInternalPublisher.publish (self.channel)

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

import select, time

TheInternalPublisher = None

class InternalPublisher:
    def __init__(self, config):
        self.channels = dict()
        self.pending_channels = list()
        self.config = config
        self.external_subscriptions = dict()
        self.debug_printing = False

    def register_channel (self, name, obj, subchannels):
        self.channels[name] = obj
        self.external_subscriptions.update (subchannels)

    def publish(self, source_name):
        if not source_name in self.channels:
            raise RuntimeError ("Internal publish: publisher not found")
        if not source_name in self.config:
            raise RuntimeError ("Internal publish: source_name config not found")
        source = self.channels[source_name]
        for pipe in self.config[source_name]['subs']:
            if pipe['protocol'] != 'internal':
                continue
            target_name = pipe['function']
            if not target_name in self.channels:
                raise RuntimeError ("Internal publish: listener not found")
            if not target_name in self.config:
                raise RuntimeError ("Internal publish: listener config not found")
            target = self.channels[target_name]
            if self.debug_printing:
                print ("Internal publish from %s to %s values (%s)"%(source_name, target_name, 
                        self.config[source_name]['output_values']))
            for v in self.config[source_name]['output_values']:
                val = getattr (source, v)
                timestamped = False
                ts_name = source_name + '_updated'
                if v == 'timestamp':
                    target_property = ts_name
                    timestamped = True
                else:
                    target_property = v
                setattr (target, target_property, val)
                if not timestamped:
                    setattr (target, ts_name, time.time())
            self.pending_channels.append ((target_name,source_name))
        self.propagate()

    def propagate(self, iteration_limit=100):
        while len(self.pending_channels):
            update_list = self.pending_channels
            self.pending_channels = list()
            for target,source in update_list:
                tobj = self.channels [target]
                tobj.updated (source)
            iteration_limit -= 1
            if iteration_limit == 0:
                raise RuntimeError ("Internal publisher: Propagation recursion limit reached. Infinite loop?")
        self.debug_printing = False

    def listen(self):
        rsocks = list(self.external_subscriptions.keys())
        xsocks = rsocks
        while True:
            r,w,x = select.select (rsocks, [], xsocks)
            for errfd in x:
                if errfd in self.external_subscriptions:
                    raise RuntimeError ("exception from subscription %s"%self.external_subscriptions[errfd])
                else:
                    raise RuntimeError ("Unknown exception from unknown file descriptor %d"%errfd)
            for rfd in r:
                s,to_chname,from_chname,input_values,input_format = self.external_subscriptions[rfd]
                tobj = self.channels[to_chname]
                tobj.data_ready(rfd)

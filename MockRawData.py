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


import time, sys
import threading

import yaml

from MicroServerComs import MicroServerComs

class MockDataSet:
    def __init__(self, cfg):
        self.sequences = cfg


class MockRawData(MicroServerComs):
    def __init__(self, data_name, dcfg, tm = time.time()):
        self.mock_period = 0
        self.cur_seq = 0
        self.cur_period = 0

        self.channel = data_name
        self.mock_period = dcfg['period']
        self.next_time = tm + self.mock_period
        self.function = dcfg['function']
        self.sequences = dcfg['sequences']

        MicroServerComs.__init__(self, self.function, channel=self.channel)
        print ("MockRawData init complete. pub = %s, output values = %s"%(self.pubchannel, self.output_values))

    def get_next(self):
        seq = self.sequences [self.cur_seq]
        ret = seq['value']
        # Future: add the ability to plot functions and smooth transitions between sequences
        self.cur_period += 1
        if self.cur_period >= seq['duration']:
            self.cur_period = 0
            self.cur_seq += 1
            if self.cur_seq >= len(self.sequences):
                self.cur_seq = 0
        return ret

    def peek_next_time(self):
        return self.next_time

    def send_data(self, tm):
        if tm >= self.next_time:
            values = self.get_next()
            for attrname,val in zip(self.output_values, values):
                if val == 'time':
                    setattr (self, attrname, self.next_time)
                else:
                    setattr (self, attrname, val)
            print ("mock %s sending data"%self.function)
            self.publish ()
            self.increment_time(tm)

    def increment_time(self, tm):
        nxt = self.next_time + self.mock_period
        # Skip forward if we have missed multiple periods
        while nxt < tm:
            nxt += self.mock_period
            self.get_next()
        self.next_time = nxt

def run_mock_data(mock):
    while True:
        next_time = mock.peek_next_time()
        sleep_time = (next_time - time.time())
        if sleep_time > 0:
            time.sleep(sleep_time)
        mock.send_data(time.time())


if __name__ == "__main__":
    with open(sys.argv[1], "r") as yml:
        cfg = yaml.load(yml)
        yml.close()
        print ("cfg=" + str(cfg))
        mocks = list()
        for dn,dcfg in cfg.items():
            mock = threading.Thread (target=run_mock_data, args=(MockRawData(dn, dcfg),))
            mock.start()
            mocks.append(mock)
        for m in mocks:
            m.join()

#!/usr/bin/env python3
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

import sys, time

import yaml

from MicroServerComs import MicroServerComs

class RAISDiscriminator(MicroServerComs):
    LOST_SIGNAL_INTERVALS=10
    HIST_LOST_SIGNAL='LOST'
    HIST_ACQUIRED_SIGNAL='ACQ'
    HIST_CONFIDENCE_FAIL='FAIL'
    HIST_CONFIDENCE_GOOD='GOOD'
    LOW_CONFIDENCE_THRESHOLD = 4.0
    LOST_SIGNAL_SCORE_WEIGHT = 10.0
    CONFIDENCE_SCORE_WEIGHT = 1.0
    LOST_SIGNAL_SCORE_FINAL_WEIGHT = 100.0
    CONFIDENCE_SCORE_FINAL_WEIGHT = 10.0
    VARIANCE_SCORE_WEIGHT=1.0

    def __init__(self, inputs_config, outputs_config):
        MicroServerComs.__init__(self, "RAISDiscriminator", config=inputs_config)
        self.gps_magnetic_variation = dict()
        self.altitude = dict()
        self.altitude_confidence = dict()
        self.Altitude_updated = dict()
        self.Altitude_updated_local_time = dict()
        self.airspeed = dict()
        self.airspeed_confidence = dict()
        self.airspeed_is_estimated = dict()
        self.Airspeed_updated = dict()
        self.Airspeed_updated_local_time = dict()
        self.heading = dict()
        self.heading_confidence = dict()
        self.Heading_updated = dict()
        self.Heading_updated_local_time = dict()
        self.roll_rate = dict()
        self.roll_rate_confidence = dict()
        self.RollRate_updated = dict()
        self.RollRate_updated_local_time = dict()
        self.roll = dict()
        self.roll_confidence = dict()
        self.Roll_updated = dict()
        self.Roll_updated_local_time = dict()
        self.pitch = dict()
        self.pitch_confidence = dict()
        self.Pitch_updated = dict()
        self.Pitch_updated_local_time = dict()
        self.pitch_rate = dict()
        self.pitch_rate_confidence = dict()
        self.PitchRate_updated = dict()
        self.PitchRate_updated_local_time = dict()
        self.yaw = dict()
        self.yaw_confidence = dict()
        self.Yaw_updated = dict()
        self.Yaw_updated_local_time = dict()
        self.climb_rate = dict()
        self.climb_rate_confidence = dict()
        self.ClimbRate_updated = dict()
        self.ClimbRate_updated_local_time = dict()
        self.turn_rate = dict()
        self.turn_rate_confidence = dict()
        self.TurnRate_updated = dict()
        self.TurnRate_updated_local_time = dict()
        self.gps_utc = dict()
        self.gps_lat = dict()
        self.gps_lng = dict()
        self.gps_ground_speed = dict()
        self.gps_ground_track = dict()
        self.ground_vector_confidence = dict()
        self.GroundVector_updated = dict()
        self.GroundVector_updated_local_time = dict()

        self.history = dict()
        self.input_map = dict()
        self.favored_channel = dict()
        for sock,mychan,fromchan,output_values,fmt,idx in self.subchannels.values():
            self.input_map[fromchan] = output_values
            self.favored_channel[fromchan] = None

        self._publisher = RAISPublisher(outputs_config)

    def updated(self, channel, idx):
        self.update_history (channel, idx)
        self.update_discriminator (channel, idx)
        self.output (channel)

    def update_history(self, channel, idx):
        # Check for lost feeds
        ts_name = channel + '_updated_local_time'
        d = getattr(self, ts_name)
        latest = time.time()
        d[idx] = latest
        interval = 0.1
        if 'gps' in self.input_map[channel][0]:
            interval = 1.0
        for a,tm in d.items():
            if a == idx:
                continue
            tdiff = latest - tm
            if tdiff > interval * RAISDiscriminator.LOST_SIGNAL_INTERVALS:
                self.append_history(idx, channel,
                        (RAISDiscriminator.HIST_LOST_SIGNAL, latest))
            elif self.get_latest_history(idx, channel)[0] == \
                    RAISDiscriminator.HIST_LOST_SIGNAL:
                self.append_history(idx, channel,
                        (RAISDiscriminator.HIST_ACQUIRED_SIGNAL, latest))

        # Check confidence from latest update
        for vname in self.input_map[channel]:
            if 'confidence' in vname:
                confidence = getattr(self,vname)[idx]
                if confidence < RAISDiscriminator.LOW_CONFIDENCE_THRESHOLD:
                    if self.get_latest_history(idx, channel)[0] == \
                            RAISDiscriminator.HIST_CONFIDENCE_FAIL:
                        if confidence < self.history[channel][idx][-1][1]:
                            self.history[channel][idx][-1] = \
                                    (RAISDiscriminator.HIST_CONFIDENCE_FAIL, confidence, latest)
                    else:
                        self.append_history(idx, channel,
                                (RAISDiscriminator.HIST_CONFIDENCE_FAIL,
                                    confidence, latest))
                elif self.get_latest_history(idx, channel)[0] == \
                        RAISDiscriminator.HIST_CONFIDENCE_FAIL:
                    self.append_history(idx, channel,
                            (RAISDiscriminator.HIST_CONFIDENCE_GOOD,
                                confidence, latest))

    def append_history(self, idx, channel, entry):
        if not channel in self.history:
            self.history[channel] = dict()
        if not idx in self.history[channel]:
            self.history[channel][idx] = list()
        self.history[channel][idx].append (entry)

    def get_latest_history(self, idx, channel):
        if not channel in self.history:
            return ('',0)
        if not idx in self.history[channel]:
            return ('',0)
        if len(self.history[channel][idx]) == 0:
            return ('',0)
        return self.history[channel][idx][-1]

    def update_discriminator(self, channel, idx):
        if self.favored_channel[channel] == None:
            self.favored_channel[channel] = idx    # Anything is better than nothing
        else:
            h = self.get_latest_history (idx, channel)

            # Find the prime output value dictionary
            for vname in self.input_map[channel]:
                if not ('confidence' in vname or 'timestamp' in vname):
                    data_vname = vname
                    break
            else:
                raise RuntimeError ("No data output found in channel %s"%channel)
            d = getattr(self, data_vname)
            # Find the median reading
            if len(d) > 2:
                # We've got 3 or more redundant sensor arrays
                readings = [r for r in d.values()]
                readings.sort()
                median = readings[len(d) / 2]
            else:
                median = None
            if h[0] == RAISDiscriminator.HIST_CONFIDENCE_FAIL or \
                    h[0] == RAISDiscriminator.HIST_LOST_SIGNAL:
                # Currently favored channel is suspect. See if there's another better
                options = [(self.score_history(ad, channel) +
                            self.score_variance (d, ad, median), ad)
                                    for ad in self.history[channel].keys()]
                options.sort(reverse=True)  # Descending scores, first is best
                if self.favored_channel[channel] != options[0][1]:
                    print ("RAISDiscriminator: favoring input pipeline %s for %s"%(
                                self.favored_channel[channel], channel))
                    self.favored_channel[channel] = options[0][1]

    def score_history (self, idx, channel):
        ret = 0.0
        last_entry = None
        if idx in self.history[channel]:
            for entry in self.history[channel][idx]:
                if entry[0] == RAISDiscriminator.HIST_CONFIDENCE_FAIL:
                    ret -= (10.0 - entry[1]) * RAISDiscriminator.CONFIDENCE_SCORE_WEIGHT
                if entry[0] == RAISDiscriminator.HIST_LOST_SIGNAL:
                    ret -= RAISDiscriminator.LOST_SIGNAL_SCORE_WEIGHT
                last_entry = entry
        if last_entry is not None:
            if last_entry[0] == RAISDiscriminator.HIST_CONFIDENCE_FAIL:
                ret -= (10.0 - last_entry[1]) * RAISDiscriminator.CONFIDENCE_SCORE_FINAL_WEIGHT
            if last_entry[0] == RAISDiscriminator.HIST_LOST_SIGNAL:
                ret -= RAISDiscriminator.LOST_SIGNAL_FINAL_SCORE_WEIGHT
        return ret

    def score_variance (self, d, idx, median):
        if (median is not None) and idx in d:
            variance = abs(d[idx] - median) / median
            return variance * RAISDiscriminator.VARIANCE_SCORE_WEIGHT
        else:
            return 0.0

    def output(self, channel):
        ts_name = channel + '_updated'
        #print ("channel %s updating"%channel)
        for vname in self.input_map[channel]:
            attrname = vname
            if 'timestamp' in vname:
                attrname = ts_name
            setattr (self._publisher, attrname,
                    getattr(self, attrname)[self.favored_channel[channel]])
        self._publisher.pub_channel (channel)

class RAISPublisher:
    def __init__(self, config):
        self.config = config
        self.outputs = dict()

    def pub_channel (self, channel):
        if not channel in self.outputs:
            self.outputs[channel] = MicroServerComs(channel, config=self.config)
        o = self.outputs[channel]
        ts_name = channel + '_updated'
        for vname in o.output_values:
            invname = vname
            if 'timestamp' == vname:
                invname = ts_name
            setattr (o, vname, getattr(self, invname))
        o.publish()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print ("RAIS Discriminator. Usage: RAISDiscriminator <input_config1> [input_confign...] <output_config>")
        sys.exit(-1)
    input_config = list()
    for i in range(1,len(sys.argv)-1):
        inp = open(sys.argv[i], "r")
        if not inp:
            print ("Cannot open file %s"%sys.argv[i])
            sys.exit(-2)
        input_config.append(yaml.load(inp))
    with open(sys.argv[-1], "r") as outp:
        output_config = yaml.load(outp)
        rd = RAISDiscriminator (input_config, output_config)
        rd.listen()

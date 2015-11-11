# Copyright (C) 2015  Garrett Herschleb
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

import logging, time

import FileConfig

control = None

logger=logging.getLogger(__name__)

class UnitTestControl:
    def __init__(self):
        global control
        control = self
        self.controls = [
                ("YokePitch", (-1.0, 1.0), 2.0),
                ("YokeRoll", (-1.0, 1.0), 2.0),
                ("Yaw", (-1.0, 1.0), 2.0),
                ("Throttle", (0.0, 1.0), 1.0),
                ("LeftTilt", (-10.0, 90.0), 100.0),
                ("RightTilt", (-0.1, .1), 0.2),
                ("VTOLEngineReleaseControl", (0,1), 1),
                ("ForwardEngineReleaseControl", (0,1), 1),
                ]
        self.last_val = dict()


    def SetChannel(self, channel, val):
        if channel < 0 or channel >= len(self.controls):
            raise RuntimeError ("Invalid channel set (%d)"%channel)
        channel_range = self.controls[channel][1]
        channel_range_size = abs(channel_range[1] - channel_range[0])
        self.last_val [self.controls[channel][0]] = val
        #print ("Set last_val[%s] = %g"%(self.controls[channel][0], val))
        logger.log (3, "Setting channel %s to %g", self.controls[channel][0], val)

    def SetPin(self, pin, val):
        if pin < 0 or pin >= len(self.controls):
            raise RuntimeError ("Invalid pin set (%d)"%pin)
        self.last_val [self.controls[pin][0]] = val
        logger.log (3, "Setting pin %s to %d", self.controls[pin][0], val)

    def __getitem__(self, key):
        return self.last_val[key]

    def SetThrottles(self, throttles):
        logger.log (3, "Setting throttles to %g,%g,%g,%g  %g,%g", throttles[0],
                throttles[1],
                throttles[2],
                throttles[3],
                throttles[4],
                throttles[5],
                )
        self.last_val ["Throttle"] = throttles[0]

    def initialize(self, filelines):
        return

    def GetLimits(self, channel):
        return self.controls[channel][1]

sensors = None

class UnitTestSensors:
    def __init__(self):
        global sensors
        self.sensor_suite = [(0, "Altitude"),
                (0, "Heading"),
                (0, "Roll"),
                (0, "RollRate"),
                (0, "Pitch"),
                (0, "PitchRate"),
                (0, "Yaw"),
                (0, "AirSpeed"),
                (0, "GroundSpeed"),
                (0, "ClimbRate"),
                (0, "Longitude"),
                (0, "Latitude"),
                (0, "MagneticDeclination"),
                (0, "TrueHeading"),
                (0, "OuterEnginePosition"),
                (0, "GroundTrack"),
                (0, "Battery"),
                ]
        self.previous_readings = [s[0] for s in self.sensor_suite]
        self.SamplesPerSecond = 10
        sensors = self

    def initialize(self, filelines):
        pass

    def Altitude(self):
        assert(self.sensor_suite[0][1] == "Altitude")
        return self.sensor_suite[0][0]

    def Heading(self):
        assert(self.sensor_suite[1][1] == "Heading")
        return self.sensor_suite[1][0]

    def Roll(self):
        assert(self.sensor_suite[2][1] == "Roll")
        return self.sensor_suite[2][0]

    def RollRate(self):
        assert(self.sensor_suite[3][1] == "RollRate")
        return self.sensor_suite[3][0]

    def Pitch(self):
        assert(self.sensor_suite[4][1] == "Pitch")
        return self.sensor_suite[4][0]

    def PitchRate(self):
        assert(self.sensor_suite[5][1] == "PitchRate")
        return self.sensor_suite[5][0]

    def Yaw(self):
        assert(self.sensor_suite[6][1] == "Yaw")
        return self.sensor_suite[6][0]

    def AirSpeed(self):
        assert(self.sensor_suite[7][1] == "AirSpeed")
        return self.sensor_suite[7][0]

    def GroundSpeed(self):
        assert(self.sensor_suite[8][1] == "GroundSpeed")
        return self.sensor_suite[8][0]

    def ClimbRate(self):
        assert(self.sensor_suite[9][1] == "ClimbRate")
        return self.sensor_suite[9][0]

    def Position(self):
        assert(self.sensor_suite[10][1] == "Longitude")
        assert(self.sensor_suite[11][1] == "Latitude")
        return (self.sensor_suite[10][0], self.sensor_suite[11][0])

    def HeadingRateChange(self):
        assert(self.sensor_suite[1][1] == "Heading")
        return ((self.sensor_suite[1][0] - self.previous_readings[1]) * self.SamplesPerSecond)

    def TrueHeading(self):
        assert(self.sensor_suite[13][1] == "TrueHeading")
        return self.sensor_suite[13][0]

    def MagneticDeclination(self):
        assert(self.sensor_suite[12][1] == "MagneticDeclination")
        return self.sensor_suite[12][0]

    def Time(self):
        return time.time()

    def GroundTrack(self):
        assert(self.sensor_suite[15][1] == "GroundTrack")
        return self.sensor_suite[15][0]

    def Battery(self):
        assert(self.sensor_suite[16][1] == "Battery")
        return self.sensor_suite[16][0]

    def OuterEnginePosition(self):
        assert(self.sensor_suite[14][1] == "OuterEnginePosition")
        return self.sensor_suite[14][0]

    def SetSensor(self, name, value):
        for snum in range(len(self.sensor_suite)):
            if name == self.sensor_suite[snum][1]:
                self.sensor_suite[snum] = (value, name)


class ResponseStep(FileConfig.FileConfig):
    def __init__(self):
        self.condition = None
        self.responses = list()
        self.pre_assertions = list()
        self.post_assertions = list()
        self.enter_time = 0.0

        list_actions = {"pre_assertions" : ("assert", self.add_pre_assert),
                        "post_assertions" : ("assert", self.add_post_assert),
                        "responses": ("response", self.add_response)
                       }
        double_arg_actions = {"condition": self.init_condition}
        FileConfig.FileConfig.__init__(self, None, double_arg_actions, list_actions=list_actions)

    def initialize(self, args, filelines):
        self.InitializeFromFileLines(filelines)

    def add_pre_assert(self, args, filelines):
        self.pre_assertions.append(' '.join(args[1:]))

    def add_post_assert(self, args, filelines):
        self.post_assertions.append(' '.join(args[1:]))

    def add_response(self, args, filelines):
        self.responses.append(' '.join(args[1:]))

    def init_condition(self, args, filelines):
        self.condition = ' '.join(args[1:])

    def Enter(self):
        self.enter_time = time.time()

    def TimeSinceEntry(self):
        return time.time() - self.enter_time

    def UpdateSensors(self):
        global control
        global sensors
        if eval(self.condition):
            if self.post_assertions:
                for a in self.post_assertions:
                    if not (eval(a)):
                        msg = "Response post assertion %s failed for step with condition (%s)"%(a, self.condition)
                        logger.error (msg)
                        raise RuntimeError (msg)
            for resp in self.responses:
                eval(resp)
            return True
        else:
            if self.pre_assertions:
                for a in self.pre_assertions:
                    if not (eval(a)):
                        msg = "Response pre assertion %s failed"%a
                        logger.error (msg)
                        raise RuntimeError (msg)
            return False

Responses = list()
CurrentStep = 0

def add_response(args, filelines):
    global Responses
    r = ResponseStep()
    r.initialize(args, filelines)
    Responses.append(r)


def ReadResponses(filename):
    global Responses
    f = open(filename, 'r')
    if not f:
        raise RuntimeError("Cannot open response file %s"%filename)
    responses = f.readlines()
    f.close()
    list_actions = {"steps" : ("step", add_response)}
    fc = FileConfig.FileConfig(list_actions=list_actions)
    fc.InitializeFromFileLines(responses)
    Responses[0].Enter()

def Update():
    global Reponses
    global CurrentStep
    if CurrentStep < len(Responses):
        if Responses[CurrentStep].UpdateSensors():
            CurrentStep += 1
            logger.info("Entering State %d"%CurrentStep)
            if CurrentStep < len(Responses):
                Responses[CurrentStep].Enter()
        return True
    else:
        return False

import math

import Common.util as util
from MicroServerComs import MicroServerComs

class GroundRoll(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "GroundRoll")

    def updated(self, channel):
        if self.a_z != 0:
            self.ground_roll = math.atan(float(self.a_x) / float(self.a_z)) * util.DEG_RAD
            self.timestamp = self.accelerometers_updated
            self.publish ()
            print ("GroundRoll: %d,%d => %g"%(self.a_z, self.a_x, self.ground_roll))


if __name__ == "__main__":
    gr = GroundRoll()
    gr.listen()

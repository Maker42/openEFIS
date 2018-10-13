import math

from Common.util import DEG_RAD
from MicroServerComs import MicroServerComs

class GroundRoll(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "GroundRoll")

    def updated(self, channel):
        if self.a_z != 0:
            self.ground_roll = (-math.atan2(float(self.a_z), float(self.a_x)) + math.pi / 2) * DEG_RAD
            self.timestamp = self.accelerometers_updated
            self.publish ()
            print ("GroundRoll: %d,%d => %g"%(self.a_z, self.a_x, self.ground_roll))


if __name__ == "__main__":
    gr = GroundRoll()
    gr.listen()

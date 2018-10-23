import math

from Common.util import DEG_RAD
from MicroServerComs import MicroServerComs

class GroundRoll(MicroServerComs):
    def __init__(self, accelerometer_calibration):
        self.accelerometer_calibration = accelerometer_calibration
        MicroServerComs.__init__(self, "GroundRoll")

    def updated(self, channel):
        if self.a_z != 0:
            if isinstance(self.accelerometer_calibration, dict):
                self.a_x = util.rate_curve (self.a_x, self.accelerometer_calibration['x'])
                self.a_y = util.rate_curve (self.a_y, self.accelerometer_calibration['y'])
                self.a_z = util.rate_curve (self.a_z, self.accelerometer_calibration['z'])
            self.ground_roll = (math.atan2(float(self.a_z), float(self.a_x)) - math.pi / 2) * DEG_RAD
            self.timestamp = self.accelerometers_updated
            self.publish ()
            print ("GroundRoll: %d,%d => %g"%(self.a_z, self.a_x, self.ground_roll))


if __name__ == "__main__":
    gr = GroundRoll()
    gr.listen()

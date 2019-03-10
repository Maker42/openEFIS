import math

from Common.util import DEG_RAD
from MicroServerComs import MicroServerComs

class GroundRoll(MicroServerComs):
    def __init__(self, accelerometer_calibration):
        self.accelerometer_calibration = accelerometer_calibration
        MicroServerComs.__init__(self, "GroundRoll")
        self.xoffset = 0

    def updated(self, channel):
        if channel == 'accelerometers':
            if self.a_z != 0:
                if isinstance(self.accelerometer_calibration, dict):
                    self.a_x = util.rate_curve (self.a_x,
                            self.accelerometer_calibration['x'])
                    self.a_z = util.rate_curve (self.a_z,
                            self.accelerometer_calibration['z'])
                self.a_x += self.xoffset
                self.ground_roll = (math.atan2(float(self.a_z),
                                    float(self.a_x)) - math.pi / 2) * DEG_RAD
                self.timestamp = self.accelerometers_updated
                self.publish ()
                print ("GroundRoll: %.2f,%.2f => %.1f"%(self.a_z, self.a_x, self.ground_roll))
        elif channel == 'systemcommand':
            if self.command.startswith( b'0att'):
                print ("Attitude: Received command to 0 out")
                self.xoffset = -self.a_x


if __name__ == "__main__":
    gr = GroundRoll()
    gr.listen()

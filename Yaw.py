
import Common.util as util
from MicroServerComs import MicroServerComs

class Yaw(MicroServerComs):
    def __init__(self, accel_factor=1.0):
        MicroServerComs.__init__(self, "Yaw")
        self.accel_factor = accel_factor

    def updated(self, channel):
        self.yaw = self.a_x * self.accel_factor
        self.timestamp = self.accelerometers_updated
        self.yaw_confidence = 10.0
        self.publish ()
        print ("Yaw: %g => %g"%(self.a_x, self.yaw))


if __name__ == "__main__":
    yaw = Yaw()
    yaw.listen()

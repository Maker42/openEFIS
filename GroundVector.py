
from MicroServerComs import MicroServerComs

class GroundVector(MicroServerComs):
    def __init__(self, conf_mult=1.0):
        MicroServerComs.__init__(self, "GroundVector")
        self.gps_lat = None
        self.gps_lng = None
        self.gps_ground_speed = None
        self.gps_ground_track = None
        self.gps_signal_quality = None
        self.confidence_multiplier = conf_mult

    def updated(self, channel):
        if channel == 'gpsfeed':
            self.ground_vector_confidence = self.gps_signal_quality * self.confidence_multiplier
            self.publish ()

if __name__ == "__main__":
    gv = GroundVector()
    gv.listen()

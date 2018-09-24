
import Common.util as util
from MicroServerComs import MicroServerComs

class TrackRate(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "TrackRate")
        self.last_ground_track = None
        self.last_time = None
        self.track_rate = None
        self._raw_track_rate = list()

    def updated(self, channel):
        if self.last_time is not None:
            timediff = self.gps_utc - self.last_time
            trackdiff = self.gps_ground_track - self.last_ground_track
            if trackdiff < -180:
                trackdiff += 360
            if trackdiff > 180:
                trackdiff -= 360
            current_track_rate = trackdiff / timediff
            self.track_rate = util.LowPassFilter (current_track_rate, self._raw_track_rate)
            self.publish ()
            print ("TrackRate: %d/%g => %g"%(trackdiff, timediff, self.track_rate))
        self.last_time = self.gps_utc
        self.last_ground_track = self.gps_ground_track


if __name__ == "__main__":
    tr = TrackRate()
    tr.listen()

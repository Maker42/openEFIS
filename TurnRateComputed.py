
from MicroServerComs import MicroServerComs

class TurnRateComputed(MicroServerComs):
    def __init__(self):
        MicroServerComs.__init__(self, "TurnRateComputed")
        self._raw_heading_rate = list()
        self.last_heading = None
        self.last_time = None
        self.filter_coefficient = .2
        self.turn_rate_computed = 0.0

    def updated(self, channel):
        self.timestamp = self.HeadingComputed_updated
        if self.last_heading is not None:
            timediff = self.timestamp - self.last_time
            current_heading_rate = (self.heading_computed - self.last_heading) / timediff
            self.turn_rate_computed += self.filter_coefficient * (current_heading_rate - self.turn_rate_computed)
            self.publish ()
            print ("TurnRateComputed: %g => %g"%(self.heading_computed, self.turn_rate_computed))
        self.last_time = self.timestamp
        self.last_heading = self.heading_computed

if __name__ == "__main__":
    pe = TurnRateComputed()
    pe.listen()

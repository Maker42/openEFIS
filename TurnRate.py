
from MicroServerComs import MicroServerComs

class TurnRate(MicroServerComs):
    def __init__(self, conf_mult=0.1):
        MicroServerComs.__init__(self, "TurnRate")
        self.turn_rate = None
        self.track_rate = None
        self.turn_rate_confidence = 0.0
        # Default 1 degree per minute
        self.confidence_multiplier = conf_mult

    def updated(self, channel):
        if channel == 'TurnRateComputed':
            self.timestamp = self.TurnRateComputed_updated
            self.turn_rate = self.turn_rate_computed
            if self.track_rate is not None:
                variance = abs(self.turn_rate - self.track_rate)
                self.turn_rate_confidence = 10.0 - variance * self.confidence_multiplier
            else:
                self.turn_rate_confidence = 9.0
            self.publish()
            print ("TurnRate: %g(%g)"%(self.turn_rate, self.turn_rate_confidence))


if __name__ == "__main__":
    turn_rate = TurnRate()
    turn_rate.listen()

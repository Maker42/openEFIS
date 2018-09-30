# Copyright (C) 2018  Garrett Herschleb
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

import yaml

from PitchEstimate import PitchEstimate
from GroundRoll import GroundRoll
from RollEstimate import RollEstimate
from RollRateEstimate import RollRateEstimate
from TurnRateComputed import TurnRateComputed
from HeadingComputed import HeadingComputed
from Pitch import Pitch
from Roll import Roll
from Yaw import Yaw
from Heading import Heading
from RollRate import RollRate
from HeadingTasEstimate import HeadingTasEstimate
from WindEstimate import WindEstimate
from PressureFactors import PressureFactors
from AirspeedComputed import AirspeedComputed
from AirspeedEstimate import AirspeedEstimate
from AltitudeComputed import AltitudeComputed
from TrackRate import TrackRate
from TurnRate import TurnRate
from Airspeed import Airspeed
from Altitude import Altitude
from ClimbRate import ClimbRate
from GroundVector import GroundVector
from ClimbRateEstimate import ClimbRateEstimate
from PitchRate import PitchRate
import InternalPublisher
import MicroServerComs
from PubSub import CONFIG_FILE

def run_service(so):
    so.listen()

if __name__ == "__main__":
    if MicroServerComs._pubsub_config is None:
        with open (CONFIG_FILE, 'r') as yml:
            MicroServerComs._pubsub_config = yaml.load(yml)
            yml.close()
    InternalPublisher.TheInternalPublisher = InternalPublisher.InternalPublisher(
            MicroServerComs._pubsub_config)
    service_objects = [
                 PitchEstimate()
                ,GroundRoll()
                ,RollEstimate()
                ,RollRateEstimate()
                ,TurnRateComputed()
                ,HeadingComputed()
                ,Pitch()
                ,Roll()
                ,Yaw()
                ,Heading()
                ,RollRate()
                ,HeadingTasEstimate()
                ,WindEstimate()
                ,PressureFactors()
                ,AirspeedComputed()
                ,AirspeedEstimate()
                ,AltitudeComputed()
                ,TrackRate()
                ,TurnRate()
                ,Airspeed()
                ,Altitude()
                ,ClimbRate()
                ,GroundVector()
                ,ClimbRateEstimate()
                ,PitchRate()
                ]
    InternalPublisher.TheInternalPublisher.listen()

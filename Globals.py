# Copyright (C) 2015  Garrett Herschleb
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


import datetime

LoggingPrefix = None
SimulationMode = None

SIM=False
LIVE_LOGGING = 0
SIM_REPLAY = 1
SIM_RECORD = 2

def datestamp():
    dt = datetime.datetime.now()
    return '%d-%02d-%02d_%02d-%02d-%02d'%(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second)


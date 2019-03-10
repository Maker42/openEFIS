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

FLIGHT_MODE_GROUND="ground"
FLIGHT_MODE_AIRBORN="airborn"
FLIGHT_MODE_TAKEOFF="takeoff"
FLIGHT_MODE_LANDING="landing"

TheAircraft = None

# Flight Information Exchange (FIX) database keynames

# User inputs
WAYPOINT_ID_KEY = "WPID"
WAYPOINT_LAT_KEY = "WPLAT"
WAYPOINT_LNG_KEY = "WPLNG"
WAYPOINT_ALT_KEY = "WPALT"
NEXT_WAYPOINT_ID_KEY = "NXTWPID"
NEXT_WAYPOINT_LAT_KEY = "NXTWPLAT"
NEXT_WAYPOINT_LNG_KEY = "NXTWPLNG"
NEXT_WAYPOINT_ALT_KEY = "NXTWPALT"
ALTITUDE_SOURCE_KEY = "ALTSRC"
SELECTED_ALTITUDE_KEY = "SELALT"
SELECTED_HEADING_KEY = "SELHDG"
SELECTED_AIRSPEED_KEY = "SELIAS"
SELECTED_CLIMB_RATE_KEY = "SELVS"
AP_ON_KEY = "APON"
FD_ON_KEY = "FDON"
HNAV_MODE_KEY = "HNVMODE"
VNAV_MODE_KEY = "VNVMODE"
START_STRATEGY_KEY = "STST"
MAGNETIC_DECLINATION_KEY = "MAGDCL"
SELECTED_TURN_RATE_KEY = "SELTR"
SELECTED_PITCH_KEY = "SELPITCH"
SELECTED_GLIDESLOPE_KEY = "SELGS"
SELECTED_WAYPOINT_KEY = "SELWP"
SET_0ATTITUDE_KEY="SET0ATT"
SET_0AIRSPEED_KEY="SET0ASP"

# FMS outputs
FD_PITCH_KEY = "FDPITCH"
FD_ROLL_KEY = "FDROLL"
FD_TARGET_HDG_KEY = "COURSE"
ETA_KEY = "ETA"


# Flight data inputs
IAS_KEY = "IAS"
ROLL_KEY = "ROLL"
PITCH_KEY = "PITCH"
ROLLRATE_KEY = "ROLLRATE"
PITCHRATE_KEY = "PITCHRATE"
HEAD_KEY = "HEAD"
#MAG_DECLINATION_KEY = "MAGDCL"
ALT_KEY = "ALT"
TALT_KEY = "TALT"
VS_KEY = "VS"
TRACK_KEY = "TRACK"
TRACKM_KEY = "TRACKM"
COURSE_KEY = "COURSE"
XTRACK_ERROR_KEY = "XTRACK"
PITCHSET_KEY = "PITCHSET"
YAW_KEY = "YAW"
CTLPTCH_KEY = "CTLPTCH"
CTLROLL_KEY = "CTLROLL"
CTLYAW_KEY = "CTLYAW"
CTLFLAP_KEY = "CTLFLAP"
CTLLBRK_KEY = "CTLLBRK"
CTLRBRK_KEY = "CTLRBRK"
THR_KEY = "THR"
MIX_KEY = "MIX"
FUELF_KEY = "FUELF"
MAP_KEY = "MAP"
FUELQ_KEY = "FUELQ"
LAT_KEY = "LAT"
LONG_KEY = "LONG"
TIMEZ_KEY = "TIMEZ"
TIMEZH_KEY = "TIMEZH"
TIMEZM_KEY = "TIMEZM"
TIMEZS_KEY = "TIMEZS"
TIMEL_KEY = "TIMEL"

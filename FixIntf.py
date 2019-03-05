#!/usr/bin/env python3
# Copyright (C) 2018-2019  Garrett Herschleb
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


import os
import sys,  time
import argparse
import logging
import importlib
import threading

import Airplane, Globals

import fix

args = None
wpchanged_time = None

def WAYPOINTS_changed(v):
    global wpchanged_time
    wpchanged_time = time.time()

def SELECTED_WAYPOINT_changed(v):
    craft.SetWaypointNumber(v)

def SELECTED_ALTITUDE_changed(v):
    craft.SelectedAltitude = v

def SELECTED_AIRSPEED_changed(v):
    craft.DesiredAirSpeed = v

def SELECTED_HEADING_changed(v):
    craft.DesiredTrueHeading = v + craft.Sensors().MagneticDeclination()

def AP_ON_changed(v):
    craft.SetServoEngagedState(v)

def HNAV_MODE_changed(v):
    craft.HnavMode = v

def VNAV_MODE_changed(v):
    craft.VnavMode = v

def START_STRATEGY_changed(v):
    craft.StartStrategy = v

def ALTITUDE_SOURCE_changed(v):
    craft.AltitudeSource = v

def SELECTED_CLIMB_RATE_changed(v):
    craft.DesiredClimbRate = v

def SELECTED_PITCH_changed(v):
    craft.SelectedPitch = v

def ReadWaypoints():
    waypoints = list()
    altitudes = list()
    for wp in range(256):
        waypoint_id_db = fix.db.get_item(Globals.WAYPOINT_ID_KEY + str(wp))
        if len(waypoint_id_db.value) > 0:
            waypoint_lat_db = fix.db.get_item(Globals.WAYPOINT_LAT_KEY + str(wp))
            waypoint_lng_db = fix.db.get_item(Globals.WAYPOINT_LNG_KEY + str(wp))
            waypoint_alt_db = fix.db.get_item(Globals.WAYPOINT_ALT_KEY + str(wp))
            waypoints.append ((waypoint_lng_db.value, waypoint_lat_db.value))
            altitudes.append (waypoint_alt_db.value)
    return waypoints,altitudes

def CreateWaypoints():
    for wp in range(256):
        waypoint_id_db = fix.db.get_item(Globals.WAYPOINT_ID_KEY + str(wp), True)
        waypoint_id_db.dtype = 'str'
        waypoint_lat_db = fix.db.get_item(Globals.WAYPOINT_LAT_KEY + str(wp), True)
        waypoint_lat_db.dtype = 'float'
        waypoint_lat_db.min = -90.0
        waypoint_lat_db.max =  90.0
        waypoint_lng_db = fix.db.get_item(Globals.WAYPOINT_LNG_KEY + str(wp), True)
        waypoint_lng_db.dtype = 'float'
        waypoint_lng_db.min = -180.0
        waypoint_lng_db.max =  180.0
        waypoint_alt_db = fix.db.get_item(Globals.WAYPOINT_ALT_KEY + str(wp), True)
        waypoint_alt_db.dtype = 'float'
        waypoint_alt_db.min = -1000.0
        waypoint_alt_db.max =  60000.0

def ConnectWaypoints():
    for wp in range(256):
        waypoint_id_db = fix.db.get_item(Globals.WAYPOINT_ID_KEY + str(wp))
        waypoint_lat_db = fix.db.get_item(Globals.WAYPOINT_LAT_KEY + str(wp))
        waypoint_lng_db = fix.db.get_item(Globals.WAYPOINT_LNG_KEY + str(wp))
        waypoint_alt_db = fix.db.get_item(Globals.WAYPOINT_ALT_KEY + str(wp))
        waypoint_id_db.valueChanged[str].connect(WAYPOINTS_changed)
        waypoint_lat_db.valueChanged[float].connect(WAYPOINTS_changed)
        waypoint_lng_db.valueChanged[float].connect(WAYPOINTS_changed)
        waypoint_alt_db.valueChanged[float].connect(WAYPOINTS_changed)

if '__main__' == __name__:
    opt = argparse.ArgumentParser(description='FMS interface to a FIX system')
    opt.add_argument('airplane_config', help='The airplane configuration')
    opt.add_argument('--log-prefix', default=None, help = 'Over-ride logging prefix')
    opt.add_argument('-l', '--log-level', type=int, default=logging.INFO, help = '1 = Maximum Logging. 100 = Absolute Silence. 40 = Errors only. 10 = Basic Debug')
    opt.add_argument('-f', '--fix-int-module', default='pyEfis.fix', help = 'The python module name to load for the FIX interface module')
    args = opt.parse_args()

    rootlogger = logging.getLogger()
    rootlogger.setLevel(args.log_level)

    Globals.SimulationMode = Globals.LIVE_LOGGING
    airplane = args.airplane_config.replace('.cfg', '')
    datestamp = Globals.datestamp()
    if args.log_prefix is not None:
        Globals.LoggingPrefix = args.log_prefix
    else:
        Globals.LoggingPrefix = os.path.join('Logs', airplane, datestamp)
    log_start = "airplane %s beginning on %s with logging prefix %s"%(airplane, datestamp, Globals.LoggingPrefix)
    try:
        os.makedirs(Globals.LoggingPrefix)
    except:
        pass
    rootlogger.addHandler(logging.FileHandler(os.path.join(Globals.LoggingPrefix, 'info.log')))
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG)
    rootlogger.addHandler(console_handler)
    rootlogger.log(99, log_start)
    start (args.airplane_config)

fms_thread = None
craft = None
run_fms = True
send_back_sensors = True

def start(airplane_config, snd_bk_snsrs=True):
    global fms_thread, craft, send_back_sensors
    send_back_sensors = snd_bk_snsrs
    rcfg = open (airplane_config,  'r')
    rlines = rcfg.readlines()
    rcfg.close()
    if not rlines:
        raise RuntimeError ('Empty config file: %s'%sys.argv[1])

    craft = Airplane.Airplane()
    craft.initialize(rlines)

    pitchdb = fix.db.get_item(Globals.FD_PITCH_KEY, True)
    pitchdb.min = -90
    pitchdb.max = 90
    rolldb = fix.db.get_item(Globals.FD_ROLL_KEY, True)
    rolldb.min = -90
    rolldb.max = 90

    altitude_source_db = fix.db.get_item(Globals.ALTITUDE_SOURCE_KEY, True)
    altitude_source_db.dtype = 'int'
    altitude_source = altitude_source_db.value
    altitude_source_db.valueChanged[int].connect(ALTITUDE_SOURCE_changed)
    selected_altitude_db = fix.db.get_item(Globals.SELECTED_ALTITUDE_KEY, True)
    selected_altitude_db.dtype = 'int'
    selected_altitude_db.min = -1000
    selected_altitude_db.max = 60000
    selected_altitude_db.value = 10000
    selected_altitude = selected_altitude_db.value
    selected_altitude_db.valueChanged[int].connect(SELECTED_ALTITUDE_changed)
    selected_airspeed_db = fix.db.get_item(Globals.SELECTED_AIRSPEED_KEY, True)
    selected_airspeed_db.dtype = 'int'
    selected_airspeed_db.min = 10
    selected_airspeed_db.max = 1000
    selected_airspeed_db.value = 120
    selected_airspeed = selected_airspeed_db.value
    selected_airspeed_db.valueChanged[int].connect(SELECTED_AIRSPEED_changed)
    selected_heading_db = fix.db.get_item(Globals.SELECTED_HEADING_KEY, True)
    selected_heading_db.dtype = 'int'
    selected_heading_db.min = 0
    selected_heading_db.max =  359
    selected_heading = selected_heading_db.value
    selected_heading_db.valueChanged[int].connect(SELECTED_HEADING_changed)
    selected_climb_rate_db = fix.db.get_item(Globals.SELECTED_CLIMB_RATE_KEY, True)
    selected_climb_rate_db.dtype = 'int'
    selected_climb_rate_db.min = -10000
    selected_climb_rate_db.max =  10000
    selected_climb_rate_db.value =  500
    selected_climb_rate = selected_climb_rate_db.value
    selected_climb_rate_db.valueChanged[int].connect(SELECTED_CLIMB_RATE_changed)
    ap_on_db = fix.db.get_item(Globals.AP_ON_KEY, True)
    ap_on_db.dtype = 'bool'
    fd_on_db = fix.db.get_item(Globals.FD_ON_KEY, True)
    fd_on_db.dtype = 'bool'
    ap_on_db.valueChanged[bool].connect(AP_ON_changed)
    hnav_mode_db = fix.db.get_item(Globals.HNAV_MODE_KEY, True)
    hnav_mode_db.dtype = 'int'
    hnav_mode = hnav_mode_db.value
    hnav_mode_db.valueChanged[int].connect(HNAV_MODE_changed)
    vnav_mode_db = fix.db.get_item(Globals.VNAV_MODE_KEY, True)
    vnav_mode_db.dtype = 'int'
    vnav_mode = vnav_mode_db.value
    vnav_mode_db.valueChanged[int].connect(VNAV_MODE_changed)
    start_strategy_db = fix.db.get_item(Globals.START_STRATEGY_KEY, True)
    start_strategy_db.dtype = 'int'
    start_strategy = start_strategy_db.value
    start_strategy_db.valueChanged[int].connect(START_STRATEGY_changed)
    selected_turn_rate_db = fix.db.get_item(Globals.SELECTED_TURN_RATE_KEY, True)
    selected_turn_rate = selected_turn_rate_db.value
    selected_pitch_db = fix.db.get_item(Globals.SELECTED_PITCH_KEY, True)
    selected_pitch_db.dtype = 'int'
    selected_pitch_db.min = 0
    selected_pitch_db.max = 40
    selected_pitch_db.value = 5
    selected_pitch = selected_pitch_db.value
    selected_pitch_db.valueChanged[int].connect(SELECTED_PITCH_changed)
    selected_glideslope_db = fix.db.get_item(Globals.SELECTED_GLIDESLOPE_KEY, True)
    selected_glideslope = selected_glideslope_db.value
    selected_waypoint_db = fix.db.get_item(Globals.SELECTED_WAYPOINT_KEY, True)
    selected_waypoint_db.dtype = 'int'
    selected_waypoint_db.min = 0
    selected_waypoint_db.max = 256
    selected_waypoint_db.value = 0
    selected_waypoint_db.valueChanged[int].connect(SELECTED_WAYPOINT_changed)

    CreateWaypoints()
    waypoints, WP_altitudes = ReadWaypoints()
    ConnectWaypoints()

    fms_thread = threading.Thread(target=thread_run, args=(
                    selected_altitude
                   ,selected_airspeed
                   ,selected_heading
                   ,selected_climb_rate
                   ,hnav_mode
                   ,vnav_mode
                   ,altitude_source
                   ,start_strategy
                   ,waypoints
                   ,WP_altitudes
                   ,selected_turn_rate
                   ,selected_pitch
                   ,selected_glideslope))
    fms_thread.start()

def stop():
    global run_fms, fms_thread
    run_fms = False
    fms_thread.join()

def thread_run( selected_altitude
               ,selected_airspeed
               ,selected_heading
               ,selected_climb_rate
               ,hnav_mode
               ,vnav_mode
               ,altitude_source
               ,start_strategy
               ,waypoints
               ,WP_altitudes
               ,selected_turn_rate
               ,selected_pitch
               ,selected_glideslope):
    global craft, run_fms, send_back_sensors, wpchanged_time
    pitchdb = fix.db.get_item(Globals.FD_PITCH_KEY)
    rolldb = fix.db.get_item(Globals.FD_ROLL_KEY)
    sensors = craft.Sensors()
    while not sensors.Ready():
        sensors.SendBarometer (fix.db.get_item("BARO").value)
        #print ("Sensors Not ready. Barometer %g"%(fix.db.get_item("BARO").value))
        time.sleep(1)
        if not run_fms:
            return
    craft.initialize_input(
                          selected_altitude
                         ,selected_airspeed
                         ,selected_heading + craft.Sensors().MagneticDeclination()
                         ,selected_climb_rate
                         ,hnav_mode
                         ,vnav_mode
                         ,altitude_source
                         ,start_strategy
                         ,waypoints
                         ,WP_altitudes
                         ,selected_turn_rate
                         ,selected_pitch
                         ,selected_glideslope)

    while run_fms:
        attitude = craft.Update()
        if isinstance(attitude,tuple):
            pitch,roll = attitude
            pitchdb.value = pitch
            rolldb.value = roll
        if send_back_sensors:
            sensors_to_fix(sensors)
        time.sleep(.1)
        if wpchanged_time is not None and time.time() > wpchanged_time+.3:
            waypoints,alts = ReadWaypoints()
            craft.SetWaypoints (waypoints, alts)
            craft.SetWaypointNumber(0)
            wpchanged_time = None

BARO_UPDATE_PERIOD=1.0
last_baro_time = time.time()-BARO_UPDATE_PERIOD
BAD_THRESHOLD=5.0
FAIL_THRESHOLD=2.0

def sensors_to_fix(sensors):
    global last_baro_time, givenbarometer
    PITCH = fix.db.get_item("PITCH")
    PITCH.value = sensors.Pitch()
    PITCH_conf = sensors.PitchConfidence()
    PITCH.bad = True if PITCH_conf < BAD_THRESHOLD else False
    PITCH.fail = True if PITCH_conf < FAIL_THRESHOLD else False
    PITCH.old = False
    ROLL = fix.db.get_item("ROLL")
    ROLL.value = sensors.Roll()
    ROLL_conf = sensors.RollConfidence()
    ROLL.bad = True if ROLL_conf < BAD_THRESHOLD else False
    ROLL.fail = True if ROLL_conf < FAIL_THRESHOLD else False
    ROLL.old = False
    YAW = fix.db.get_item("YAW")
    YAW.value = sensors.Yaw()
    YAW_conf = sensors.YawConfidence()
    YAW.bad = True if YAW_conf < BAD_THRESHOLD else False
    YAW.fail = True if YAW_conf < FAIL_THRESHOLD else False
    YAW.old = False
    lat,lng = sensors.Position()
    if lng is not None:
        gsq = sensors.GPSSignalQuality()
        LAT = fix.db.get_item("LAT")
        LAT.value = lat
        LAT.bad = True if gsq < BAD_THRESHOLD else False
        LAT.fail = True if gsq < FAIL_THRESHOLD else False
        LAT.old = False
        LONG = fix.db.get_item("LONG")
        LONG.value = lng
        LONG.bad = True if gsq < BAD_THRESHOLD else False
        LONG.fail = True if gsq < FAIL_THRESHOLD else False
        LONG.old = False
        TRACK = fix.db.get_item("TRACK")
        TRACK.value = sensors.GroundTrack()
        TRACK.bad = True if gsq < BAD_THRESHOLD else False
        TRACK.fail = True if gsq < FAIL_THRESHOLD else False
        TRACK.old = False
        GS = fix.db.get_item("GS")
        GS.value = sensors.GroundSpeed()
        GS.bad = True if gsq < BAD_THRESHOLD else False
        GS.fail = True if gsq < FAIL_THRESHOLD else False
        GS.old = False
    ALT = fix.db.get_item("ALT")
    ALT.value = sensors.Altitude()
    ALT_conf = sensors.AltitudeConfidence()
    ALT.bad = True if ALT_conf < BAD_THRESHOLD else False
    ALT.fail = True if ALT_conf < FAIL_THRESHOLD else False
    ALT.old = False
    airspeed = sensors.AirSpeed()
    if airspeed is not None:
        IAS = fix.db.get_item("IAS")
        IAS.value = airspeed
        IAS_conf = sensors.AirSpeedConfidence()
        IAS.bad = True if IAS_conf < BAD_THRESHOLD else False
        IAS.fail = True if IAS_conf < FAIL_THRESHOLD else False
        IAS.old = False
    VS = fix.db.get_item("VS")
    VS.value = sensors.ClimbRate()
    VS_conf = sensors.ClimbRateConfidence()
    VS.bad = True if VS_conf < BAD_THRESHOLD else False
    VS.fail = True if VS_conf < FAIL_THRESHOLD else False
    VS.old = False
    HEAD = fix.db.get_item("HEAD")
    HEAD.value = sensors.Heading()
    HEAD_conf = sensors.HeadingConfidence()
    HEAD.bad = True if HEAD_conf < BAD_THRESHOLD else False
    HEAD.fail = True if HEAD_conf < FAIL_THRESHOLD else False
    HEAD.old = False

    if last_baro_time+BARO_UPDATE_PERIOD < time.time():
        last_baro_time = time.time()
        sensors.SendBarometer (fix.db.get_item("BARO").value)

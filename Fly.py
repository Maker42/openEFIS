#   ## For unix, Start with #!/usr/bin/python
# Copyright (C) 2015-2018  Garrett Herschleb
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

import Airplane, Globals
import Common.PIDOptimizer as PIDOptimizer
import Common.Optimizer as Optimizer
import Test.UnitTestFixture as UnitTestFixture

args = None

if '__main__' == __name__:
    opt = argparse.ArgumentParser(description='Execute a flight plan')
    opt.add_argument('airplane_config', help='The airplane configuration')
    opt.add_argument('flight_plan', help='The flight plan file to execute')
    opt.add_argument('-w', '--way-points', help='Way points')
    opt.add_argument('-u', '--unit-test', default=None, help = 'File containing unit test responses')
    opt.add_argument('-r', '--replay', default=None, help = 'Specify log to replay in simulation')
    opt.add_argument('-l', '--log-prefix', default=None, help = 'Over-ride logging prefix')
    opt.add_argument('-m', '--home', default=None, help = 'Over-ride home directory for objects and procedures')
    opt.add_argument('-c', '--record', action='store_true', help = 'Create a recording in simulation')
    opt.add_argument('-p', '--pid', help = 'PID optimizer config file')
    opt.add_argument('-i', '--pickup', type=int, default=0, help = 'Pick up the flight plan from sequence number')
    opt.add_argument('--log-level', type=int, default=logging.INFO, help = '1 = Maximum Logging. 100 = Absolute Silence. 40 = Errors only. 10 = Basic Debug')
    opt.add_argument('-v', '--magnetic-variation', default=None, help='The magnetic variation(declination) of the current position')
    opt.add_argument('-a', '--altitude', default=None, type=int, help='The currently known altitude')
    opt.add_argument('-b', '--barometer', default=None, type=float, help='The given barometric pressure in inches of mercury')
    opt.add_argument('-s', '--wind-speed', default=None, type=int, help='The current wind speed in knots')
    opt.add_argument('--wind-heading', default=None, type=int, help='The current wind heading in degrees')
    args = opt.parse_args()

    if args.home:
        if os.path.isdir(args.home):
            os.environ['HOME'] = args.home
        else:
            print("Invalid home directory: " + args.home)
            sys.exit(1)
    rootlogger = logging.getLogger()
    rootlogger.setLevel(args.log_level)

    dist_path = '/usr/local/lib/python2.7/dist-packages'
    if os.path.isdir(dist_path):
        sys.path.append('/usr/local/lib/python2.7/dist-packages')
    Globals.SimulationMode = Globals.LIVE_LOGGING
    airplane = args.airplane_config.replace('.cfg', '')
    flight_plan = args.flight_plan.replace('.pln', '')
    if args.replay:
        Globals.SimulationMode = Globals.SIM_REPLAY
        Globals.LoggingPrefix = args.replay
        log_start = "REPLAY: airplane %s beginning flight_plan %s on with logging prefix %s"%(airplane, flight_plan, Globals.LoggingPrefix)
    else:
        if args.record:
            Globals.SimulationMode = Globals.SIM_RECORD
        datestamp = Globals.datestamp()
        Globals.LoggingPrefix = os.path.join('Logs', airplane, flight_plan, datestamp)
        log_start = "airplane %s beginning flight_plan %s on %s with logging prefix %s"%(airplane, flight_plan, datestamp, Globals.LoggingPrefix)
    try:
        os.makedirs(Globals.LoggingPrefix)
    except:
        pass
    rootlogger.addHandler(logging.FileHandler(os.path.join(Globals.LoggingPrefix, 'info.log')))
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG)
    rootlogger.addHandler(console_handler)
    rootlogger.log(99, log_start)

    rcfg = open (args.airplane_config,  'r')
    rlines = rcfg.readlines()
    rcfg.close()
    if not rlines:
        raise RuntimeError ('Empty config file: %s'%sys.argv[1])

    craft = Airplane.Airplane()
    craft.initialize(rlines, args.altitude, args.barometer, (args.wind_heading, args.wind_speed))
    if args.magnetic_variation != None:
        craft.KnownMagneticVariation(args.magnetic_variation)
    if args.altitude != None:
        craft.KnownAltitude(args.altitude)

    ffp = open(args.flight_plan, 'r')
    craft.FlightPlan = ffp.readlines()
    ffp.close()

    if args.way_points:
        wps = open(args.way_points, 'r')
        way_points = wps.readlines()
        wps.close()
        craft.WayPoints = eval('\n'.join(way_points))

    if args.pid:
        if os.path.exists(args.pid):
            file = open (args.pid, 'r')
            lines = file.readlines()
            file.close()
            pid_scoring = PIDOptimizer.PIDScoring()
            pid_scoring.initialize(lines)
            pid_scoring.Optimizer = Optimizer.IsoParameterOptimizer()
            parameters = PIDOptimizer.ReadOptParameters(lines)
            pid_scoring.Optimizer.Parameters = parameters
        else:
            raise RuntimeError("Scoring configuration file %s does not exist"%args.pid)

        pid_scoring.Optimizer.StartOptimize()
        craft.SetPIDScoring(pid_scoring)

    if args.unit_test:
        UnitTestFixture.ReadResponses(args.unit_test)
        UnitTestFixture.Update()

    if craft._sensors.AirSpeed() > 20:
        craft._flight_control._throttle_control.Set(.5)
        craft.ChangeMode (Globals.FLIGHT_MODE_AIRBORN)
    dispatch_command_number = 0
    if args.pickup > 0:
        dispatch_command_number = args.pickup
        if dispatch_command_number >= len(craft.FlightPlan):
            dispatch_command_number = len(craft.FlightPlan) - 1
    craft._flight_plan_index = dispatch_command_number
    craft.DispatchCommand (craft.FlightPlan[dispatch_command_number])
    while True:
        craft.Update()
        if args.unit_test:
            UnitTestFixture.Update()
        time.sleep(.1)

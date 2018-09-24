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

import logging, asyncore

import Common.FileConfig as FileConfig
import Globals
import Common.util as util

import SenseControl, CommandControl
import Xplane, SurfaceControl, AttitudeControl, FlightControl, AttitudeControlVTOL, GroundControl
import TakeoffControlVTOL, LandingControlVTOL, AttitudeVTOLEstimation
import MiddleEngineTiltControl, VTOLYawControl, SolenoidControl, ThrottleControl
import TakeoffControl, LandingControl

logger=logging.getLogger(__name__)

class Airplane(FileConfig.FileConfig):
    def __init__(self):
        # Airplane parameters:
        self.MaxAirSpeed = 10       # nautical miles per Hour
        self.StallSpeed = 90.0      # Knots
        self.GroundEffectHeight = 15.0

        # Operating parameters:
        self.BatteryMinReserve = 30
        self.ApproachEndpoints = list()
        self.RunwayAltitude = None
        self.RunwayTouchdownPoint = None
        self.RunwayHeading = 0
        self.RunwayLength = 0


        self.DesiredCourse = [(0,0), (0,0)]
        self.DesiredAltitude = 0
        self.WayPoints = list()
        self._next_waypoint = 0

        self._desired_heading = 0
        self.CurrentFlightMode = Globals.FLIGHT_MODE_GROUND
        self._was_moving = False

        self._sensors = None
        self._servo_controller = None
        self._aileron_control = None
        self._elevator_control = None
        self._rudder_control = None
        self._throttle_control = None
        self._gear_control = None
        self._flap_control = None
        self._yawvtol_control = None
        self._middle_engine_tilt_control = None
        self._VTOL_engine_release_control = None
        self._forward_engine_release_control = None

        self._attitude_control = None
        self._attitude_control_vtol = None
        self._attitude_vtol_estimation = None
        self._flight_control = None
        self._ground_control = None
        self._takeoff_control = None
        self._landing_control = None

        self._command_control = None
        self._return_state = 0

        self.FlightPlan = list()
        self._flight_plan_index = 0
        self.FlightPlanLoopStart = 0

        self._in_pid_optimization = ''      # Routing string, or null string if not optimizing
        self._pid_optimization_score = None
        self._pid_optimization_scoring = None
        self._pid_optimization_step = None

        self._current_directive_end_time = 0       # Time this directive will end for timed directives.

        double_arg_actions = {"RudderControl" : self.init_rudder_control,
                "ThrottleControl" : self.init_throttle_control,
                "AileronControl" : self.init_aileron_control,
                "ElevatorControl" : self.init_elevator_control,
                "GearControl" : self.init_gear_control,
                "FlapControl" : self.init_flap_control,
                "Sensors" : self.init_sensors,
                "ServoControl" : self.init_servo_control,
                "CommandControl" : self.init_command_control,
                "AttitudeControl" : self.init_attitude_control,
                "TakeoffControl" : self.init_takeoff_control,
                "LandingControl" : self.init_landing_control,
                "VTOLYawControl": self.init_yaw_vtol_control,
                "MiddleEngineTiltControl": self.init_middle_engine_tilt_control,
                "VTOLEngineReleaseControl": self.init_VTOL_engine_release_control,
                "ForwardEngineReleaseControl": self.init_forward_engine_release_control
        }

        single_arg_actions = {
                "AttitudeControl" : self.init_attitude_control,
                "FlightControl" : self.init_flight_control,
                "GroundControl" : self.init_ground_control,
                "TakeoffControl" : self.init_takeoff_control,
                "LandingControl" : self.init_landing_control,
                }

        FileConfig.FileConfig.__init__(self, single_arg_actions, double_arg_actions)

    def initialize(self, filelines):
        self.InitializeFromFileLines (filelines)

        self._aileron_control.SetServoController(self._servo_controller)
        self._elevator_control.SetServoController(self._servo_controller)
        self._rudder_control.SetServoController(self._servo_controller)
        self._throttle_control.SetServoController(self._servo_controller)
        if self._middle_engine_tilt_control:
            self._middle_engine_tilt_control.SetServoController(self._servo_controller)
        if self._VTOL_engine_release_control:
            self._VTOL_engine_release_control.SetServoController(self._servo_controller)
        if self._forward_engine_release_control:
            self._forward_engine_release_control.SetServoController(self._servo_controller)
        if self._gear_control:
            self._gear_control.SetServoController(self._servo_controller)
        if self._flap_control:
            self._flap_control.SetServoController(self._servo_controller)
        self.ComputeRunwayEndpoints()
        Globals.TheAircraft = self

    def ComputeRunwayEndpoints(self):
        if self.RunwayAltitude == None:
            self.RunwayAltitude = self._sensors.Altitude()
        if len(self.ApproachEndpoints) == 0:
            if self.RunwayTouchdownPoint != None:
                position = self.RunwayTouchdownPoint
            else:
                position = self._sensors.Position()
            if self.RunwayLength > 0:
                end = util.AddPosition(position, self.RunwayLength, self.RunwayHeading)
            else:
                end = position
            self.ApproachEndpoints = [position, end]

    def init_attitude_control(self, args, filelines):
        if not (self._aileron_control and self._elevator_control and self._rudder_control and self._sensors):
            raise RuntimeError("Must initialize fundamental controls before attitude control")
        if len(args) > 1 and args[1] == "AttitudeControlVTOL":
            if not self._attitude_control:
                raise RuntimeError ("Must initialize flight attitude control before VTOL attitude control")
            self._attitude_control_vtol = AttitudeControlVTOL.AttitudeControlVTOL (self._servo_controller,
                    self._yawvtol_control, self._elevator_control, self._attitude_control, self._sensors)
            self._attitude_control_vtol.initialize(filelines)
        else:
            self._attitude_control = AttitudeControl.AttitudeControl (self._aileron_control,
                    self._elevator_control, self._rudder_control, self._sensors)
            self._attitude_control.initialize(filelines)

    def init_flight_control(self, args, filelines):
        if not (self._attitude_control and self._throttle_control and self._sensors):
            raise RuntimeError("Must initialize fundamental controls before flight control")
        self._flight_control = FlightControl.FlightControl(self._attitude_control,
                self._throttle_control, self._sensors, self)
        self._flight_control.initialize(filelines)

    def init_ground_control(self, args, filelines):
        if not (self._rudder_control and self._throttle_control and self._sensors):
            raise RuntimeError("Must initialize fundamental controls before ground control")
        self._ground_control = GroundControl.GroundControl(self._rudder_control,
                self._throttle_control, self._sensors, self)
        self._ground_control.initialize(filelines)

    def init_takeoff_control(self, args, filelines):
        if not (self._aileron_control and self._elevator_control and self._rudder_control and self._sensors
                and self._throttle_control):
            raise RuntimeError("Must initialize fundamental controls before takeoff control")
        if len(args) > 1 and args[1] == "TakeoffControlVTOL":
            if not self._attitude_vtol_estimation:
                self._attitude_vtol_estimation = AttitudeVTOLEstimation.AttitudeVTOLEstimation()
            self._takeoff_control = TakeoffControlVTOL.TakeoffControlVTOL(self._attitude_control_vtol,
                    self._middle_engine_tilt_control, self._VTOL_engine_release_control,
                    self._sensors, self._attitude_vtol_estimation, self)
        else:
            self._takeoff_control = TakeoffControl.TakeoffControl(self._attitude_control,
                    self._throttle_control, self._rudder_control, self._gear_control,
                    self._flap_control, self._sensors, self)
        self._takeoff_control.initialize(filelines)

    def init_landing_control(self, args, filelines):
        if not (self._attitude_control and self._throttle_control and self._sensors):
            raise RuntimeError("Must initialize fundamental controls before landing control")
        if len(args) > 1 and args[1] == "LandingControlVTOL":
            if not self._attitude_vtol_estimation:
                self._attitude_vtol_estimation = AttitudeVTOLEstimation.AttitudeVTOLEstimation()
            self._landing_control = LandingControlVTOL.LandingControlVTOL(self._attitude_control_vtol,
                    self._middle_engine_tilt_control, self._forward_engine_release_control,
                    self._sensors, self._attitude_vtol_estimation, self)
        else:
            self._landing_control = LandingControl.LandingControl(self._flight_control,
                    self._attitude_control, self._throttle_control,
                    self._gear_control, self._flap_control, self._sensors, self)
        self._landing_control.initialize(filelines)

    def init_yaw_vtol_control(self, args, filelines):
        self._yawvtol_control = eval(' '.join(args[1:]))
        self._yawvtol_control.initialize(self._middle_engine_tilt_control)

    def init_middle_engine_tilt_control(self, args, filelines):
        self._middle_engine_tilt_control = eval(' '.join(args[1:]))

    def init_VTOL_engine_release_control(self, args, filelines):
        self._VTOL_engine_release_control = eval(' '.join(args[1:]))

    def init_forward_engine_release_control(self, args, filelines):
        self._forward_engine_release_control = eval(' '.join(args[1:]))

    def init_rudder_control(self, args, filelines):
        self._rudder_control = eval(' '.join(args[1:]))

    def init_throttle_control(self, args, filelines):
        self._throttle_control = eval(' '.join(args[1:]))

    def init_aileron_control(self, args, filelines):
        self._aileron_control = eval(' '.join(args[1:]))

    def init_elevator_control(self, args, filelines):
        self._elevator_control = eval(' '.join(args[1:]))

    def init_gear_control(self, args, filelines):
        self._gear_control = eval(' '.join(args[1:]))

    def init_flap_control(self, args, filelines):
        self._flap_control = eval(' '.join(args[1:]))

    def init_command_control(self, args, filelines):
        self._command_control = eval(' '.join(args[1:]))
        self._command_control.initialize(filelines)

    def init_sensors(self, args, filelines):
        self._sensors = eval(' '.join(args[1:]))
        self._sensors.initialize(filelines)

    def init_servo_control(self, args, filelines):
        self._servo_controller = eval(' '.join(args[1:]))
        self._servo_controller.initialize(filelines)

    def KnownAltitude(self, alt):
        self._sensors.KnownAltitude(alt)

    def KnownMagneticVariation(self, mv):
        self._sensors.KnownMagneticVariation(mv)

    # Notifies that the take off roll has accompished enough speed that flight controls
    # have responsibility and authority. Activates PIDs.
    def Taxi(self, desired_location):
        assert (self.CurrentFlightMode == Globals.FLIGHT_MODE_GROUND)
        if isinstance(desired_location,list):
            self.DesiredCourse = desired_location
        else:
            self.DesiredCourse = [self._sensors.Location(), desired_location]
        self._ground_control.Taxi(self.DesiredCourse)

    # Inputs specify end of runway
    def Takeoff(self, length=0, heading=-1, end=None, soft_field=False):
        assert (self.CurrentFlightMode == Globals.FLIGHT_MODE_GROUND)
        self.RunwayTouchdownPoint = self._sensors.Position()
        if end:
            self.ApproachEndpoints = [self.RunwayTouchdownPoint, end]
        else:
            self.ApproachEndpoints = list()
        if length:
            self.RunwayLength = length
        if heading >= 0:
            self.RunwayHeading = heading
        else:
            self.RunwayHeading = self._sensors.TrueHeading()
        self.RunwayAltitude = self._sensors.Altitude()
        self.ComputeRunwayEndpoints()
        self.ChangeMode(Globals.FLIGHT_MODE_TAKEOFF)
        self.DesiredCourse = self.ApproachEndpoints
        self._takeoff_control.Takeoff (self.ApproachEndpoints, soft_field)
        if soft_field:
            self._elevator_control.Set(.7)
        else:
            self._elevator_control.Set(-.05)
        logger.info("Taking off on runway (%g,%g) - (%g,%g), %g feet alt",
                self.DesiredCourse[0][0], self.DesiredCourse[0][1], 
                self.DesiredCourse[1][0], self.DesiredCourse[1][1], 
                self.RunwayAltitude)

    def NextWayPoint(self, next_waypoint, desired_altitude=0, desired_airspeed=0):
        assert (self.CurrentFlightMode == Globals.FLIGHT_MODE_AIRBORN)
        last_waypoint = self.DesiredCourse[1]
        self.DesiredCourse = [last_waypoint, next_waypoint]
        heading,distance,_ = util.TrueHeadingAndDistance (self.DesiredCourse)
        ret = "Flying to waypoint %g nm away, bearing %g degrees, with altitude %g"%(
                    distance, heading, desired_altitude, desired_airspeed)
        logger.info(ret)
        if desired_altitude:
            self.DesiredAltitude = desired_altitude
        if desired_airspeed:
            self.DesiredAirSpeed = desired_airspeed
        self._flight_control.FollowCourse(self.DesiredCourse, self.DesiredAltitude)
        return ret

    def Swoop(self, low_alt, high_alt, min_airspeed = 0):
        self._flight_control.Swoop(low_alt, high_alt, min_airspeed)
        return "Swooping down to %g and back to %g"%(low_alt, high_alt)

    def FlyCourse(self, course, desired_altitude=0, desired_airspeed=0, rounding=False):
        assert (self.CurrentFlightMode == Globals.FLIGHT_MODE_AIRBORN)
        self.DesiredCourse = course
        heading,_a,_ = util.TrueHeadingAndDistance (self.DesiredCourse)
        curpos = self._sensors.Position()
        dist = [curpos, course[1]]
        _a,distance,_ = util.TrueHeadingAndDistance (dist)
        ret = "Flying course with destination waypoint %g nm away, bearing %g degrees, with altitude %g, airspeed %g"%(
                    distance, heading, desired_altitude, desired_airspeed)
        logger.info (ret)
        if desired_altitude:
            self.DesiredAltitude = desired_altitude
        if desired_airspeed:
            self.DesiredAirSpeed = desired_airspeed
        self._flight_control.FlyCourse (course, desired_altitude, desired_airspeed, rounding)
        return ret

    def FlyTo(self, next_waypoint, desired_altitude=0, desired_airspeed=0):
        assert (self.CurrentFlightMode == Globals.FLIGHT_MODE_AIRBORN)
        last_waypoint = self._sensors.Position()
        self.DesiredCourse = [last_waypoint, next_waypoint]
        heading,distance,_ = util.TrueHeadingAndDistance (self.DesiredCourse)
        ret = "Flying to waypoint %g nm away, bearing %g degrees, with altitude %g, airspeed %g"%(
                    distance, heading, desired_altitude, desired_airspeed)
        logger.info (ret)
        if desired_altitude:
            self.DesiredAltitude = desired_altitude
        if desired_airspeed:
            self.DesiredAirSpeed = desired_airspeed
        self._flight_control.FlyTo (next_waypoint, desired_altitude, desired_airspeed)
        return ret

    def TurnTo(self, degrees, roll, desired_altitude=0):
        ret = "Turning to %g degrees with roll %g at altitude %d"%(degrees, roll, desired_altitude)
        logger.info (ret)
        assert (self.CurrentFlightMode == Globals.FLIGHT_MODE_AIRBORN)
        if desired_altitude:
            self.DesiredAltitude = desired_altitude
        self._flight_control.TurnTo (degrees, roll, self.DesiredAltitude)
        return ret

    def Turn(self, degrees, roll=0, desired_altitude=0):
        ret = "Turning %g degrees with roll %g at altitude %d"%(degrees, roll, desired_altitude)
        logger.info (ret)
        assert (self.CurrentFlightMode == Globals.FLIGHT_MODE_AIRBORN)
        if desired_altitude:
            self.DesiredAltitude = desired_altitude
        self._flight_control.Turn (degrees, roll, self.DesiredAltitude)
        return ret

    def ChangeAltitude(self, desired_altitude):
        assert (self.CurrentFlightMode == Globals.FLIGHT_MODE_AIRBORN)
        ret = "New altitude %g"%desired_altitude
        logger.info (ret)
        self.DesiredAltitude = desired_altitude
        self._flight_control.DesiredAltitude = desired_altitude
        return ret

    def StraightAndLevel(self, dtime, desired_altitude=0, desired_airspeed=0, desired_heading=None):
        ret = "Flying Straight and Level for %g seconds, alt=%s, airspeed=%s, heading=%s"%(
                dtime, str(desired_altitude),
                str(desired_airspeed), str(desired_heading))
        logger.info (ret)
        if desired_altitude:
            self.DesiredAltitude = desired_altitude
        self._flight_control.StraightAndLevel(desired_altitude, desired_airspeed, desired_heading)
        if dtime == 0:
            self._flight_control.NotifyWhenNominal()
        else:
            self._current_directive_end_time = self._sensors.Time() + dtime
        return ret

    def SensorSnapshot(self):
        return self._sensors.Snapshot()

    # Inputs specify touchdown point and (exact) heading and altitude of runway
    def Land(self, touchdown=None, length=0, heading=-1, end=None, runway_altitude=None,
                    outer_marker=None, outer_altitude=0,
                    middle_marker=None, right_pattern=False, pattern_alt=0):
        if (self.CurrentFlightMode == Globals.FLIGHT_MODE_GROUND):
            self._throttle_control.Set(0)
        else:
            if touchdown:
                self.RunwayTouchdownPoint = touchdown
                if end:
                    self.ApproachEndpoints = [self.RunwayTouchdownPoint, end]
                else:
                    self.ApproachEndpoints = list()
            if length:
                self.RunwayLength = length
            if heading >= 0:
                self.RunwayHeading = heading
            if runway_altitude != None:
                self.RunwayAltitude = runway_altitude
            self.ComputeRunwayEndpoints()
            self.ChangeMode (Globals.FLIGHT_MODE_LANDING)
            self.DesiredCourse = self.ApproachEndpoints
            logger.info("Landing on runway (%g,%g) - (%g,%g), %g feet alt",
                    self.DesiredCourse[0][0], self.DesiredCourse[0][1], 
                    self.DesiredCourse[1][0], self.DesiredCourse[1][1], 
                    self.RunwayAltitude)
            self._landing_control.Land(self.DesiredCourse, self.RunwayAltitude,
                   outer_marker, outer_altitude, middle_marker, right_pattern, pattern_alt)
    
    def CompleteRunway(self):
        self.FlyCourse(self.ApproachEndpoints, self.RunwayAltitude+1000, self.StallSpeed * 1.5, rounding=False)

    def PickupFromPlan(self, step_number):
        if step_number < 0:
            ret = "Invalid step number %d"%step_number
        elif step_number >= len(self.FlightPlan):
            ret = "Invalid step number %d (max %d)"%(step_number, len(self.FlightPlan)-1)
        else:
            self._flight_plan_index = step_number -1
            self.SendNextCommand()
            ret = "Starting plan from step %d"%step_number
        return ret

    def Update(self):
        if self.has_crashed():
            logger.error("Airplane crashed")
            self._throttle_control.Set(0)
            return -1
        if not self._was_moving and self._sensors.GroundSpeed() > 0:
            self._was_moving = True
        if self.CurrentFlightMode == Globals.FLIGHT_MODE_AIRBORN:
            self._flight_control.Update()
            if self._current_directive_end_time != 0 and self._current_directive_end_time <= self._sensors.Time():
                # The latest optimization step was running but has completed. Go to next directive
                self._current_directive_end_time = 0
                logger.debug("Completed last timed directive")
                self.GetNextDirective()
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_GROUND:
            self._ground_control.Update()
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_LANDING:
            self._landing_control.Update()
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_TAKEOFF:
            self._takeoff_control.Update()
        if self._command_control:
            asyncore.loop(0.001, count=1)
        return self._return_state

    def ChangeMode(self, newmode):
        logger.debug ("Change mode from %s to %s", self.CurrentFlightMode, newmode)
        self._sensors.FlightMode(self.CurrentFlightMode,
                True if isinstance(self._takeoff_control,TakeoffControlVTOL.TakeoffControlVTOL)
                     else False)
        if self.CurrentFlightMode == newmode:
            return

        last_desired_pitch = 0
        if self.CurrentFlightMode == Globals.FLIGHT_MODE_AIRBORN:
            last_desired_pitch = self._flight_control.Stop()
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_GROUND:
            if self._ground_control:
                self._ground_control.Stop()
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_LANDING:
            self._landing_control.Stop()
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_TAKEOFF:
            last_desired_pitch = self._takeoff_control.Stop()

        self.CurrentFlightMode = newmode

        logger.debug ("Starting mode %s", newmode)
        if self.CurrentFlightMode == Globals.FLIGHT_MODE_AIRBORN:
            self._flight_control.Start(last_desired_pitch)
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_GROUND:
            self._ground_control.Start()
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_LANDING:
            self._landing_control.Start(last_desired_pitch)
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_TAKEOFF:
            self._takeoff_control.Start()

        self._return_state = 1

    def SetPIDScoring(self, scoring_object):
        self._pid_optimization_scoring = scoring_object

    def PIDOptimizationStart(self, which_pid, outfile=None):
        if not self._pid_optimization_scoring:
            # Optimization has completed. Skip this instruction and go to next
            self.GetNextDirective()
            return
        self._in_pid_optimization = which_pid
        routing_to = self._in_pid_optimization.split('.')[0]
        params = self._pid_optimization_scoring.Optimizer.OptimizeIteration(self._pid_optimization_score)
        logger.debug("PID optimization start with %s", str(params))
        if not params:
            print ("Optimization Complete. Params = %s, score = %g"%(
                str(self._pid_optimization_scoring.Optimizer.GetBestParams()),
                self._pid_optimization_scoring.Optimizer.GetBestScore()))
            self._in_pid_optimization = ""
            self._pid_optimization_scoring = None
            return
        if routing_to == "flight":
            step = self._flight_control.PIDOptimizationStart (which_pid[7:], params, self._pid_optimization_scoring, outfile)
            assert(step)
            self._current_directive_end_time = self._sensors.Time() + step.periods
        else:
            raise RuntimeError("non-flight PID optimization unimplemented")

    def PIDOptimizationNext(self):
        if not self._pid_optimization_scoring:
            # Optimization has completed. Skip this instruction and go to next
            logger.warning ("Ordered PID optimization step with no optimizer")
            self.GetNextDirective()
            return
        routing_to = self._in_pid_optimization.split('.')[0]
        if routing_to == "flight":
            ret = self._flight_control.PIDOptimizationNext()
        else:
            raise RuntimeError("non-flight PID optimization unimplemented")
        if isinstance(ret,float) or isinstance(ret,int):
            self._pid_optimization_score = ret
            logger.debug("PID optimization score %g", ret)
            self.GetNextDirective()
        else:
            self._current_directive_end_time = self._sensors.Time() + ret.periods
            logger.debug("PID optimization start next with time period %g", ret.periods)

    def GetNextDirective(self):
        if self._sensors.Battery() < self.BatteryMinReserve - self.RTBBatteryNeeded():
            self.Land()
        elif (self.CurrentFlightMode == Globals.FLIGHT_MODE_AIRBORN or
              self.CurrentFlightMode == Globals.FLIGHT_MODE_GROUND):
            ccdir = None
            if self._command_control:
                ccdir = self._command_control.GetNextDirective()
            if not ccdir:
                self.SendNextCommand()
            else:
                self.DispatchCommand(ccdir)
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_LANDING:
            self.ChangeMode (Globals.FLIGHT_MODE_GROUND)
        elif self.CurrentFlightMode == Globals.FLIGHT_MODE_TAKEOFF:
            self.ChangeMode (Globals.FLIGHT_MODE_AIRBORN)
            self.SendNextCommand()

    def RTBBatteryNeeded(self):
        # TODO: Calculate based on distance from base
        return 0

    def SendNextCommand(self):
        while True:
            self._flight_plan_index += 1
            if self._flight_plan_index >= len(self.FlightPlan):
                if (self.FlightPlanLoopStart  >= len(self.FlightPlan)):
                    print ("Fatal: No Command Available. Returning to Base.")
                    return self.Land()
                self._flight_plan_index = self.FlightPlanLoopStart
            command = self.FlightPlan[self._flight_plan_index]
            command = command.strip()
            if not self.DispatchCommand (command):
                break


    def DispatchCommand(self, command):
        nonblocking_commands = ["ChangeAltitude"]
        blocking_commands = ["Turn", "NextWayPoint", "Taxi", "Takeoff", "FlyTo", "Land", 
                "PIDOptimizationStart", "PIDOptimizationNext", "StraightAndLevel", "FlyCourse",
                "CompleteRunway"]
        logger.info ("Executing command %s", command)
        if command.startswith('#'):
            return True
        for c in nonblocking_commands:
            if command.startswith (c):
                eval ("self." + command)
                return True
        else:
            for c in blocking_commands:
                if command.startswith (c):
                    eval ("self." + command)
                    break
            else:
                print ("Fatal: Invalid Command %s. Returning to Base."%(command, ))
                self.Land()
        return False

    def has_crashed(self):
        return ((self._sensors.GroundSpeed() == 0) and (self._was_moving))

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

import logging

logger=logging.getLogger(__name__)

AUTOMATIC = 1
MANUAL = 0
DIRECT  = 0
REVERSE  = 1

class PID:
    def __init__(self, setPoint, Kp, Ki, Kd, direction, millis):
        self.mySetpoint = setPoint
        self.inAuto = False
        
        self.SetOutputLimits(0, 255)				#default output limit corresponds to 
                                                    #the arduino pwm limits

        self.SampleTime = 10					#default Controller Sample Time is 0.1 seconds

        self.controllerDirection = DIRECT
        self.SetTunings(Kp, Ki, Kd)
        self.SetControllerDirection(direction)

        self.lastTime = millis-self.SampleTime
        self.outMin = 0
        self.outMax = 1023

     
    """ Compute() **********************************************************************
     *     This, as they say, is where the magic happens.  this function should be called
     *   every time "void loop()" executes.  the function will decide for itself whether a new
     *   pid Output needs to be computed.  returns true when the output is computed,
     *   false when nothing has been done.
     **********************************************************************************/ 
    """
    # returns: double
    def Compute(self, inp, millis):
       if(not self.inAuto):
           return self.lastOutput
       now = millis
       timeChange = (now - self.lastTime)
       if(timeChange>=self.SampleTime):
          # Compute all the working error variables*/
          error = self.mySetpoint - inp
          self.ITerm += (self.ki * error)
          if(self.ITerm > self.outMax):
            self.ITerm = self.outMax
          elif(self.ITerm < self.outMin):
            self.ITerm = self.outMin
          dInput = (inp - self.lastInput)
     
          # Compute PID Output*/
          self.lastOutput = self.kp * error + self.ITerm- self.kd * dInput
          
          if(self.lastOutput > self.outMax):
            self.lastOutput = self.outMax
          elif(self.lastOutput < self.outMin):
            self.lastOutput = self.outMin
          logger.log (1, "setpoint = %g, error = %g, iterm = %g, dInput=%g, out = %g",
                  self.mySetpoint, error, self.ITerm, dInput, self.lastOutput)
          
          # Remember some variables for next time*/
          self.lastInput = inp
          self.lastTime = now
       return self.lastOutput


    """ SetTunings(...)*************************************************************
     * This function allows the controller's dynamic performance to be adjusted. 
     * it's called automatically from the constructor, but tunings can also
     * be adjusted on the fly during normal operation
     ******************************************************************************/"""
    # returns: void
    def SetTunings(self, Kp, Ki, Kd):
        if (Kp<0 or Ki<0 or Kd<0):
            return
     
        self.dispKp = Kp
        self.dispKi = Ki
        self.dispKd = Kd
       
        SampleTimeInSec = float(self.SampleTime)/1000.0
        self.kp = Kp
        self.ki = Ki * SampleTimeInSec
        self.kd = Kd / SampleTimeInSec
     
        if(self.controllerDirection == REVERSE):
          self.kp = (0 - self.kp)
          self.ki = (0 - self.ki)
          self.kd = (0 - self.kd)
      
    """ SetSampleTime(...) *********************************************************
     * sets the period, in Milliseconds, at which the calculation is performed	
     ******************************************************************************"""
    # returns: void
    def SetSampleTime(self, NewSampleTime):
        if (NewSampleTime > 0):
          ratio = NewSampleTime / float(self.SampleTime)
          self.ki *= ratio
          self.kd /= ratio
          self.SampleTime = NewSampleTime
     
    """ SetOutputLimits(...)****************************************************
     *     This function will be used far more often than SetInputLimits.  while
     *  the input to the controller will generally be in the 0-1023 range (which is
     *  the default already,)  the output will be a little different.  maybe they'll
     *  be doing a time window and will need 0-8000 or something.  or maybe they'll
     *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
     *  here.
     **************************************************************************"""
    # returns: void
    def SetOutputLimits(self, Min, Max):
       if(Min >= Max):
            return
       self.outMin = Min
       self.outMax = Max
     
       if(self.inAuto):
           if(self.ITerm > self.outMax):
               self.ITerm= self.outMax
           elif(self.ITerm < self.outMin):
               self.ITerm= self.outMin
           if(self.lastOutput > self.outMax):
               self.lastOutput= self.outMax
           elif(self.lastOutput < self.outMin):
               self.lastOutput= self.outMin

    """ SetMode(...)****************************************************************
     * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
     * when the transition from manual to auto occurs, the controller is
     * automatically initialized
     ******************************************************************************"""
    # returns: void
    def SetMode(self, Mode, _lastInput, _lastOutput):
        newAuto = (Mode == AUTOMATIC)
        if(newAuto == (not self.inAuto)):
        # we just went from manual to auto*/
            self.Initialize(_lastInput, _lastOutput)
        self.inAuto = newAuto
     
    """ Initialize()****************************************************************
     *	does all the things that need to happen to ensure a bumpless transfer
     *  from manual to automatic mode.
     ******************************************************************************/"""
    # returns: void
    def Initialize(self, _lastInput, _lastOutput):
       self.ITerm = _lastOutput
       self.lastOutput = _lastOutput
       self.lastInput = _lastInput
       if(self.ITerm > self.outMax):
           self.ITerm = self.outMax
       elif(self.ITerm < self.outMin):
           self.ITerm = self.outMin

    """ SetControllerDirection(...)*************************************************
     * The PID will either be connected to a DIRECT acting process (+Output leads 
     * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
     * know which one, because otherwise we may increase the output when we should
     * be decreasing.  This is called from the constructor.
     ******************************************************************************"""
    # returns: void
    def SetControllerDirection(self, Direction):
       if(self.inAuto and Direction != self.controllerDirection):
          self.kp = (0 - self.kp)
          self.ki = (0 - self.ki)
          self.kd = (0 - self.kd)
       self.controllerDirection = Direction

    """ Status Funcions*************************************************************
     * Just because you set the Kp=-1 doesn't mean it actually happened.  these
     * functions query the internal state of the PID.  they're here for display 
     * purposes.  this are the functions the PID Front-end uses for example
     ******************************************************************************"""
    # returns: double
    def GetKp(self):
        return  self.dispKp
    # returns: double
    def GetKi(self):
        return  self.dispKi
    # returns: double
    def GetKd(self):
        return  self.dispKd
    # returns: int
    def GetMode(self):
        return  AUTOMATIC if self.inAuto else MANUAL
    # returns: int
    def GetDirection(self):
        return self.controllerDirection

    def SetSetPoint (self, setpoint):
        self.mySetpoint = setpoint

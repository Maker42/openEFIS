# Copyright (C) 2016  Garrett Herschleb
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

import time, logging, copy

import PID, FuzzyController
import FileConfig
import util

logger=logging.getLogger(__name__)

class ControlComplex:
    def __init__(self, ms, pid_tuning, sample_period, pid_output_limits,
                       fuzzy_main_config, fuzzy_measured):
        kp,ki,kd = pid_tuning
        self._pid = PID.PID(0, kp, ki, kd, PID.DIRECT, ms)
        self._pid.SetSampleTime (sample_period)
        mn,mx = pid_output_limits
        self._pid.SetOutputLimits (mn, mx)
        self._next_update_time = ms
        self._last_parameters = None
        self._sample_period = sample_period
        self._last_output = None
        self._fuzzy = None
        if fuzzy_main_config:
            fuzzy_config = list()
            gfile = open (fuzzy_main_config, 'r')
            fuzzy_config = gfile.readlines()
            gfile.close()
            if fuzzy_measured:
                mfile = open(fuzzy_measured, 'r')
                fuzzy_config += mfile.readlines()
                mfile.close()
            self._fuzzy = FuzzyController.FuzzyController()
            self._fuzzy.initialize(fuzzy_config)
        self._fuzzy_active = True
        self._in_pid_optimization = False
        self._pid_optimization_scoring = None
        self._pid_optimization_goal = None
        self._journal_file = None

    # inputs: list of variables that effect the outcome.
    #         inputs[0] = Primary input that PID uses
    #         inputs[-1] (last in the array) = set point (desired outcome)
    #         Therefore inputs must be a list at least 2 elements long
    def Compute(self, inputs, ms, experience_file, new_pid_limits=None):
        if ms >= self._next_update_time:
            if self._last_parameters != None and experience_file:
                self._last_parameters[-1] = inputs[0] # Desired value become actual measured value
                if self._fuzzy.CompareSets (inputs, self._last_parameters, (1, len(inputs)-1)):
                    # Record only if the conditions throughout the period are relatively static
                    ofile = open (experience_file, 'a+')
                    mrule = FuzzyController.MeasuredRule(self._last_parameters, self._last_output)
                    mrule.Record (ofile)
                    ofile.close()
                else:
                    logger.debug ("Measured rule not recorded because conditions changed too much")
            self._next_update_time = ms + self._sample_period
            output = None
            if not self._in_pid_optimization:
                # Try first to get an answer from the experienced fuzzy engine
                if self._fuzzy:
                    output = self._fuzzy.Compute (inputs)
                    if output != None:
                        if not self._fuzzy_active:
                            self._pid.SetMode (PID.MANUAL, inputs[0], self._last_output)
                        self._fuzzy_active = True
                        logger.debug ("Got pitch from experience engine: %g", output)
                    else:
                        logger.log (5, "No answer from experience engine. Trying PID")
            # If no answer from experience, compute with PID
            if output == None:
                if self._fuzzy_active:
                    last_out = self._last_output
                    if last_out == None:
                        last_out = 0
                    if self._last_parameters == None:
                        last_input = inputs[0]
                    else:
                        last_input = self._last_parameters[0]
                    self._pid.SetMode (PID.AUTOMATIC, last_input, self._last_output)
                if not self._in_pid_optimization:
                    self._pid.SetSetPoint (inputs[-1])
                self._fuzzy_active = False
                if new_pid_limits != None:
                    rmn,rmx = new_pid_limits
                    self._pid.SetOutputLimits(rmn, rmx)
                output = self._pid.Compute (inputs[0], ms)
            if self._in_pid_optimization:
                self._pid_optimization_scoring.IncrementScore(inputs[0], output, self._pid_optimization_goal, self._journal_file)
            elif self._journal_file:
                self._journal_file.write(",%g,%g,%g"%(inputs[-1], inputs[0], output))
            self._last_output = output
            self._last_parameters = inputs
            return output
        else:
            return self._last_output

    def SetMode (self, mode, inp, outp):
        if (not self._fuzzy_active) or (mode == PID.MANUAL):
            self._pid.SetMode (mode, inp, outp)
        self._last_output = outp

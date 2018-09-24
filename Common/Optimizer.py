# Copyright (C) 2012-2016  Garrett Herschleb
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

import FileConfig
import logging

logger = logging.getLogger(__name__)

# Class OptParameter:
#  Parameter control class for parameters being optimized.
# To use this class, instantiate an object, and assign the public properties
# Add each object to a list, and assign the list to the optimizer for use.
class OptParameter(FileConfig.FileConfig):
    def __init__(self):
        # Public properties
        self.Min = 0.0              # The minimum value this parameter may have
        self.Max = 0.0              # The maximum value this parameter may have
        self.SmallestStepSize = 1   # The smallest increment/decrement movement this parameter may have
        self.SearchStart = None     # The value from which to start (your best guess for the best value)
        self.Identifier = None      # What to call this parameter

        # Private properites
        self._best_value = 0
        FileConfig.FileConfig.__init__(self)

    def StartSearch(self):
        assert(self.Min != self.Max)
        if None != self.SearchStart:
            if (self.SearchStart < self.Min):
                logger.error ("Search start for parameter %s is %g < %g", self.Identifier, self.SearchStart, self.Min)
            assert(self.SearchStart >= self.Min)
            if (self.SearchStart > self.Max):
                logger.error ("Search start for parameter %s is %g > %g", self.Identifier, self.SearchStart, self.Max)
            assert(self.SearchStart <= self.Max)
            self._best_value = self.SearchStart
        else:
            self._best_value = (self.Min + self.Max) / 2;
        if (self.SmallestStepSize <= 0):
            logger.error ("Smallest step size for parameter %s is %g", self.Identifier, self.SmallestStepSize)
            assert(self.SmallestStepSize > 0)

    def AssignBest(self, params):
        self._best_value = params[self.Identifier]

    def InRange(self, offset):
        v = self._best_value + offset
        return False if (v < self.Min or v > self.Max) else True

    def SetStart(self,v):
        logger.debug("SetStart: Parameter[%s] = %s", self.Identifier, str(v))
        self._best_value = v

    def RegisterNewBest(self, offset):
        self._best_value += offset

    def GetOffsetValue(self,offset):
        return  (self._best_value + offset)

    def GetBestValue(self):
        return self._best_value

# Optimizer:
#  Class used for finding an optimal set of parameters that produce the best score.
# Usage:
#  1. Create an Optimizer object (myoptimizer for example).
#  2. Create a list of OptParameter objects, and assign it to myoptimizer.Parameters.
#     The parameters will be given to the scoring function in the order they are in the list.
#  4. Call myoptimizer.StartOptimize
#  5. Call myoptimizer.OptimizeIteration repeatedly until it returns None, indicating the end of
#     the search.
#
#     The optimize function may be called repeatedly
#     with varying OptParameter controls for increasingly refined results, or to search solution
#     space from a different starting point to overcome large local minimas.
class Optimizer:
    def __init__(self):
        # Public Properties
        self.Parameters = list()
        self.KeepNRunnersUp = 3

        # Private Properties
        self._best_moved = False
        self._best_score = None
        self.runners_up = list()
        self.iterations = 0

    def GetIterationCount(self):
        return self.iterations

    def GetBestParams(self, runner_up=-1):
        if runner_up < 0:
            params = dict()
            for p in self.Parameters:
                params[p.Identifier] = p.GetBestValue()
        else:
            params = self.runners_up[runner_up][1]
        return params

    def GetBestScore(self, runner_up=-1):
        if runner_up < 0:
            return self._best_score
        else:
            return self.runners_up[runner_up][0]

    def register_runner_up(self, score, params):
        self.runners_up.append ((score, params))
        self.runners_up.sort(None, None, True)
        if len(self.runners_up) > self.KeepNRunnersUp:
            del self.runners_up[-1]

    def ResetSearch(self, runner_up=-1):
        self._best_moved = False
        if runner_up >= 0:
            if runner_up >= len(self.runners_up):
                return False
            for p in self.Parameters:
                p.SetStart(self.runners_up[runner_up][1][p.Identifier])
            self._best_score = self.runners_up[runner_up][0]
        return True

class IsoParameterOptimizer(Optimizer):
    def __init__(self):
        Optimizer.__init__(self)
        self.StepSizeMultiplier = 2
        self.MaxDownwardTrend = 3

    def StartOptimize(self):
        self.current_pnum = 0
        self.plen = len(self.Parameters)
        for p in self.Parameters:
            p.StartSearch()
        self._best_moved = False
        self.param_increment = self.Parameters[self.current_pnum].SmallestStepSize
        self.next_parameter_offset = 0
        self.last_parameter_offset = 0
        self.parameter_best_score = self._best_score
        self.parameter_best_offset = 0
        self.downward_trend = 0
        self.prev_score = None

    # Optimize: Find the highest scoring parameter set
    # Returns: Next set of parameters if the search should continue, otherwise None
    def OptimizeIteration(self,
            last_score                          # The score from the last set of parameters returned.
                                                # If this is the first optimizer iteration, this should be None
            ):
        self.iterations += 1
        ret = None
        if (last_score == None):
            ret = self.get_next_iteration_params()
        else:
            ret = self.parameter_iteration(last_score)
        return ret

    # parameter_iteration:
    # Search around the latest local minima with velocity gravitation pulling us in the most
    # improved offset.
    # Returns the next set of parameters to test, or None if the searching has found a best.
    def parameter_iteration(self, last_score):
        should_continue = True
        if self.last_parameter_offset == 0:
            # First iteration scored. Record the baseline score and begin search
            self.parameter_best_score = last_score
            self.parameter_best_offset = 0
            self.next_parameter_offset = self.param_increment
        else:
            if self.parameter_best_score == None or last_score > self.parameter_best_score:
                self.parameter_best_offset = self.last_parameter_offset
                self.parameter_best_score = last_score
                logger.debug("new optimize parameter '%s' offset=%g",
                        self.Parameters[self.current_pnum].Identifier,
                        self.parameter_best_offset)
            # Check to see if we need to abandon this direction of search and start another.
            # Is the slope of the scores going very negative, or is there an established down trend?
            if self.prev_score != None and last_score < self.prev_score:
                self.downward_trend += 1
            else:
                self.downward_trend = 0
            if ((self._best_score != None and last_score < self._best_score * 2) or
                    (self.downward_trend > self.MaxDownwardTrend) or
                    (not self.Parameters[self.current_pnum].InRange(self.next_parameter_offset))):
                param_cont = True
                if self.next_parameter_offset > 0:
                    self.param_increment = -self.Parameters[self.current_pnum].SmallestStepSize
                    self.next_parameter_offset = self.param_increment
                    if (not self.Parameters[self.current_pnum].InRange(self.next_parameter_offset)):
                        param_cont = False
                else:
                    param_cont = False
                if not param_cont:
                    if self._best_score == None or self.parameter_best_score > self._best_score:
                        # New global minima found! Yeah!
                        self._best_score = self.parameter_best_score
                        self._best_moved = True
                        self.register_best (self.current_pnum, self.parameter_best_offset)
                        logger.debug("optimize parameter '%s' best=%g, offset=%g",
                                self.Parameters[self.current_pnum].Identifier,
                                self.Parameters[self.current_pnum].GetBestValue(),
                                self.parameter_best_offset)
                    logger.info("optimize parameter iteration: _best_moved=%s", str(self._best_moved))

                    # Move to next parameter
                    self.current_pnum += 1
                    if self.current_pnum >= self.plen:
                        # One round of searching on all parameters done.
                        self.current_pnum = 0
                        for p in self.Parameters:
                            logger.debug("Parameter %s = %g"%(p.Identifier, p._best_value))
                        if not self._best_moved:
                            should_continue = False
                        self._best_moved = False
                    self.param_increment = self.Parameters[self.current_pnum].SmallestStepSize
                    self.next_parameter_offset = self.param_increment
                    if (not self.Parameters[self.current_pnum].InRange(self.next_parameter_offset)):
                        self.param_increment = -self.Parameters[self.current_pnum].SmallestStepSize
                        self.next_parameter_offset = self.param_increment
                    self.parameter_best_score = None
                    self.parameter_best_offset = 0
        if should_continue:
            ret = self.get_next_iteration_params()
            self.last_parameter_offset = self.next_parameter_offset
            self.next_parameter_offset += self.param_increment
            self.param_increment *= self.StepSizeMultiplier
            self.prev_score = last_score
        else:
            ret = None
        return ret

    def get_next_iteration_params(self):
        params = dict()
        for i in range(self.current_pnum):
            v = self.Parameters[i].GetOffsetValue(0)
            params[self.Parameters[i].Identifier] = v
        #print ("reading self.Parameters[%d] out of %s"%(self.current_pnum, self.Parameters))
        v = self.Parameters[self.current_pnum].GetOffsetValue(self.next_parameter_offset)
        params[self.Parameters[self.current_pnum].Identifier] = v
        for i in range(self.current_pnum+1,len(self.Parameters),1):
            v = self.Parameters[i].GetOffsetValue(0)
            params[self.Parameters[i].Identifier] = v
        return params

    def register_best(self, pnum, offset):
        oldparams = dict()
        oldscore = self._best_score
        for i in range(pnum):
            oldparams[self.Parameters[i].Identifier] = self.Parameters[i].GetBestValue()
            self.Parameters[i].RegisterNewBest(0)
        oldparams[self.Parameters[pnum].Identifier] = self.Parameters[pnum].GetBestValue()
        self.Parameters[pnum].RegisterNewBest(offset)
        for i in range(pnum+1,len(self.Parameters),1):
            oldparams[self.Parameters[i].Identifier] = self.Parameters[i].GetBestValue()
            self.Parameters[i].RegisterNewBest(0)
        self.register_runner_up (oldscore, oldparams)


class RadiusSearchOptimizer(Optimizer):
    def __init__(self):
        Optimizer.__init__(self)
        self.last_params = None

    def OptimizeIteration(self, last_score):
        # Searching a little farther around for anything better than the last local minima
        should_continue = True
        self.iterations += 1
        if last_score != None:
            self.register_score (last_score, self.last_params)
            should_continue = self.increment_params(self.param_positions)

            logger.info("optimize radius iteration: _best_moved=%s",str(self._best_moved))
        if should_continue:
            self.last_params = self.get_search_params()
            return self.last_params
        else:
            return None

    # InitSearch:
    #  Prepare state variables for searching evenly spaced points in the solution space.
    #  Each parameter is searched with nsteps number of points in the range. The range of
    #  search can be given in two ways:
    # Option 1:
    #       Search the area around the current best point. Specify a non-zero radius, which
    #       will be multiplied by the SmallestStepSize of each parameter.
    # Option 2:
    #       Search the whole solution space evenly, between the min and max of each parameter.
    #       For this option, specify a radius of 0.
    def InitSearch(self, nsteps=5, radius=100):
        self.search_steps = nsteps
        if radius > 0:
            rad_inc = int(nsteps/2)
            self.search_min = [p.GetOffsetValue(-radius * p.SmallestStepSize) for p in self.Parameters]
            self.search_increment = [int(radius / rad_inc) * p.SmallestStepSize for p in self.Parameters]
        else:
            self.search_increment = [(p.Max-p.Min)/(nsteps+1) for p in self.Parameters]
            self.search_min = list()
            map(lambda p,i: self.search_min.append(p.Min+i), self.Parameters,
                                      self.search_increment)
        self.param_positions = [0 for i in range(len(self.Parameters))]

    # get_search_params: Gets the current parameter set for a search iteration
    def get_search_params(self):
        params = dict()
        for param in range(len(self.Parameters)):
            params[self.Parameters[param].Identifier] = (self.search_min[param] +
                    self.search_increment[param]*self.param_positions[param])
        return params

    # register_score: Records the given score from the last run
    def register_score(self, score, params):
        if self._best_score == None or score > self._best_score:
            oldparams = dict()
            log_line = 'SearchIteration Best (%g) Moved to:'%score
            for p in self.Parameters:
                log_line += ' %s=%g'%(p.Identifier, params[p.Identifier])
                oldparams[p.Identifier] = p.GetBestValue()
                p.AssignBest(params)
            logger.debug(log_line)
            self.register_runner_up (self._best_score, oldparams)
            self._best_score = score
            self._best_moved = True
        elif (len(self.runners_up) < self.KeepNRunnersUp) or (score > self.runners_up[-1][0]):
            log_line = 'SearchIteration new runner up (%g) Moved to:'%score
            for p in self.Parameters:
                log_line += ' %s=%g'%(p.Identifier, params[p.Identifier])
            logger.debug(log_line)
            self.register_runner_up (score, params)


    # Perform a single nested loop increment for a search radius
    # Return False if all nested loops are complete, and True if you should continue
    def increment_params(self, param_positions):
        for i in range(len(param_positions)):
            param_positions[i] += 1
            if param_positions[i] < self.search_steps:
                break
            else:
                param_positions[i] = 0
        else:
            return False
        return True

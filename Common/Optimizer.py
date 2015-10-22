# Copyright (C) 2012-2015  Garrett Herschleb
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
        self.VelocityAdjustmentFactor = 2.0

        # Private properites
        self.current_velocity = 0.0
        self.local_best_value = 0
        self.global_best_value = 0
        self.search_offset = 0
        FileConfig.FileConfig.__init__(self)

    def StartSearch(self):
        assert(self.Min != self.Max)
        if None != self.SearchStart:
            self.local_best_value = self.SearchStart
        else:
            self.local_best_value = (self.Min + self.Max) / 2;
        self.global_best_value = self.local_best_value
        self.current_velocity = 0.0

    def RegisterNewGlobalBest(self, direction):
        self.global_best_value = self.GetValue(direction)

    def AssignGlobalBest(self, params):
        self.global_best_value = params[self.Identifier]

    def ResetSearch(self):
        self.current_velocity = 0.0
        self.local_best_value = self.global_best_value

    def SetStart(self,v):
        self.current_velocity = 0.0
        logger.debug("SetStart: Parameter[%s] = %s", self.Identifier, str(v))
        self.local_best_value = v

    def RegisterNewLocalBest(self, direction):
        self.local_best_value = self.GetValue(direction)
        self.current_velocity += (direction * self.SmallestStepSize * self.VelocityAdjustmentFactor)
        if direction != 0:
            logger.debug("New velocity for parameter %s: %g"%(self.Identifier, self.current_velocity))

    def GetValue(self,direction):
        if 0 == direction:
            return  (self.local_best_value)
        else:
            value = self.local_best_value + self.SmallestStepSize * direction + self.current_velocity
            if value < self.Min:
                value = self.Min
            elif value > self.Max:
                value = self.Max
            return  (value)

    def GetGlobalBestValue(self):
        return self.global_best_value

# Optimizer:
#  Class used for finding an optimal set of parameters that produce the best score.
#  Includes parameter movement momentum to resist local minimas.
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
        self.local_best_moved = False
        self.global_best_moved = False
        self.local_best_score = None
        self.global_best_score = None
        self.runners_up = list()

    def StartOptimize(self):
        self.iterations = 0
        self.current_pnum = 0
        self.plen = len(self.Parameters)
        for p in self.Parameters:
            p.StartSearch()
        self.local_best_score = None
        self.global_best_score = None
        self.searching_radius = False
        self.global_best_moved = False
        self.local_best_moved = False
        self.iterations = 1
        self.param_direction = -1
        self.parameter_best_score = self.local_best_score
        self.parameter_best_direction = 0

    def GetGlobalBestParams(self, runner_up=-1):
        if runner_up < 0:
            params = dict()
            for p in self.Parameters:
                params[p.Identifier] = p.GetGlobalBestValue()
        else:
            params = self.runners_up[runner_up][1]
        return params

    def GetGlobalBestScore(self, runner_up=-1):
        if runner_up < 0:
            return self.global_best_score
        else:
            return self.runners_up[runner_up][0]

    # Optimize: Find the highest scoring parameter set
    # Returns: Next set of parameters if the search should continue, otherwise None
    def OptimizeIteration(self,
            last_score,                         # The score from the last set of parameters returned.
                                                # If this is the first optimizer iteration, this should be None
            allow_search_radius=False,          # True if the optimizer should search in a large circle around
                                                # the area of the last local minima in order to search for a new
                                                # and unrelated global minima.
            allow_velocity_decrements=False     # True if the optimizer should search with decreasing velocity
                                                # multiplication factors in order to find more refined results.
                                                # So far in the author's experiments, neither of these flags
                                                # yields very positive results. Your mileage may vary.
            ):
        ret = None
        restart_iterative_search = False
        if self.searching_radius:
            ret = self.SearchIteration(last_score)
            if not ret:
                if self.global_best_moved:
                    self.global_best_moved = False
                    self.local_best_moved = False
                    restart_iterative_search = True
                elif not self.decrement_velocity(allow_velocity_decrements):
                    return None     # Finally give up all searching. We've done our best
        if not ret:
            if (last_score == None) or restart_iterative_search:
                ret = self.get_next_iteration_params()
            else:
                ret = self.parameter_iteration(last_score)
                if not ret:
                    if allow_search_radius:
                        self.global_best_moved = False
                        self.local_best_moved = False
                        self.searching_radius = True
                        self.InitSearch()
                        ret = self.SearchIteration(None)
                    else:
                        # Can't find anything better. Go to the next adjustment factor
                        if self.decrement_velocity(allow_velocity_decrements):
                            self.global_best_moved = False
                            self.local_best_moved = False
                            ret = self.get_next_iteration_params()
                        else:
                            ret = None     # Finally give up all searching. We've done our best
        return ret


    # Register the last score and compute the next 
    # score should be None for the first iteration
    def SearchIteration(self, score):
        # Searching a little farther around for anything better than the last local minima
        if score != None:
            self.searching_radius = self.process_search_score(score)

            self.iterations += 1
            logger.info("optimize radius iteration: local_best_moved=%s, global_best_moved=%s"%(
                    str(self.local_best_moved),
                    str(self.global_best_moved)))
        if self.searching_radius:
            return self.get_search_params()
        else:
            return None

    # parameter_iteration:
    # Search around the latest local minima with velocity gravitation pulling us in the most
    # improved direction.
    # Returns the next set of parameters to test, or None if the searching has found a best.
    def parameter_iteration(self, last_score):
        should_continue = True
        if last_score == None:
            # First iteration. Get the baseline score
            self.param_direction = 0
        elif self.param_direction == 0:
            # First iteration scored. Record the baseline score and begin search
            self.parameter_best_score = last_score
            self.parameter_best_direction = 0
            self.parameter_direction = -1
            self.iterations += 1
        else:
            if self.parameter_best_score == None or last_score > self.parameter_best_score:
                self.parameter_best_direction = self.param_direction
                self.parameter_best_score = last_score
            self.iterations += 1
            self.param_direction += 2
            if self.param_direction > 1:
                # Search both directions for this parameter. See if it improved anything
                self.param_direction = -1
                if self.local_best_score == None or self.parameter_best_score > self.local_best_score:
                    # New local minima found
                    self.local_best_score = self.parameter_best_score
                    self.local_best_moved = True
                    logger.debug ("New local best for parameter %s: %g, direction %d"%(
                        self.Parameters[self.current_pnum].Identifier,
                        self.Parameters[self.current_pnum].GetValue(self.parameter_best_direction),
                        self.parameter_best_direction))
                    self.register_local_best (self.current_pnum, self.parameter_best_direction)
                if self.global_best_score == None or self.parameter_best_score > self.global_best_score:
                    # New global minima found! Yeah!
                    self.global_best_score = self.parameter_best_score
                    self.global_best_moved = True
                    self.register_global_best (self.current_pnum, self.parameter_best_direction)
                logger.info("optimize parameter iteration: local_best_moved=%s, global_best_moved=%s"%(
                        str(self.local_best_moved),
                        str(self.global_best_moved)))

                # Move to next parameter
                self.current_pnum += 1
                if self.current_pnum >= self.plen:
                    # One round of searching on all parameters done.
                    self.current_pnum = 0
                    for p in self.Parameters:
                        logger.debug("Parameter %s = %g, velocity %g"%(p.Identifier, p.local_best_value, p.current_velocity))
                    if not self.local_best_moved:
                        # Not finding any new improvements around here.
                        # Search around for any better ground a few hops away
                        should_continue = False
                    self.global_best_moved = False
                    self.local_best_moved = False
        if should_continue:
            return self.get_next_iteration_params()
        else:
            return None

    def decrement_velocity(self, allow_velocity_decrements):
        if self.Parameters[0].VelocityAdjustmentFactor < 1 or (not allow_velocity_decrements):
            # At the end of fine tuning. We've found our best answer
            logger.info("Solution found in %d iterations"%self.iterations)
            # Search terminated. Return global best
            ##################################################################
            return False
            ##################################################################

        for p in self.Parameters:
            p.VelocityAdjustmentFactor /= 1.5
            p.current_velocity = 0
        logger.info ("New velocity adjustment factor: %g"%self.Parameters[0].VelocityAdjustmentFactor)
        return True


    def get_next_iteration_params(self):
        params = dict()
        for i in range(self.current_pnum):
            v = self.Parameters[i].GetValue(0)
            params[self.Parameters[i].Identifier] = v
        #print ("reading self.Parameters[%d] out of %s"%(self.current_pnum, self.Parameters))
        v = self.Parameters[self.current_pnum].GetValue(self.param_direction)
        params[self.Parameters[self.current_pnum].Identifier] = v
        for i in range(self.current_pnum+1,len(self.Parameters),1):
            v = self.Parameters[i].GetValue(0)
            params[self.Parameters[i].Identifier] = v
        return params

    def ResetSearch(self, runner_up=-1):
        self.current_pnum = 0
        self.global_best_moved = False
        self.local_best_moved = False
        self.param_direction = -1
        for p in self.Parameters:
            p.ResetSearch()
        if runner_up >= 0:
            if runner_up >= len(self.runners_up):
                return False
            for p in self.Parameters:
                p.SetStart(self.runners_up[runner_up][1][p.Identifier])
            self.local_best_score = self.runners_up[runner_up][0]
        return True

    # InitSearch:
    #  Prepare state variables for searching evenly spaced points in the solution space.
    #  Each parameter is searched with nsteps number of points in the range. The range of
    #  search can be given in two ways:
    # Option 1:
    #       Search the area around the current global best point. Specify a non-zero radius, which
    #       will be multiplied by the SmallestStepSize of each parameter.
    # Option 2:
    #       Search the whole solution space evenly, between the min and max of each parameter.
    #       For this option, specify a radius of 0.
    def InitSearch(self, nsteps=5, radius=100):
        for p in self.Parameters:
            p.ResetSearch()
        self.search_steps = nsteps
        if radius > 0:
            rad_inc = int(nsteps/2)
            self.search_min = [p.GetValue(-radius) for p in self.Parameters]
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

    # process_search_score: Records the given score from the last run, and increments the search
    # to the next parameter set.
    # Returns True if the search is still ongoing, and false once the entire search area is complete.
    def process_search_score(self, score):
        if self.global_best_score == None or score > self.global_best_score:
            oldparams = dict()
            log_line = 'SearchIteration Global Best (%g) Moved to:'%score
            for p in self.Parameters:
                log_line += ' %s=%g'%(p.Identifier, params[p.Identifier])
                oldparams[p.Identifier] = p.GetGlobalBestValue()
                p.AssignGlobalBest(params)
            logger.debug(log_line)
            self.register_runner_up (self.global_best_score, oldparams)
            self.global_best_score = score
            self.global_best_moved = True
        elif (len(self.runners_up) < self.KeepNRunnersUp) or (score > self.runners_up[-1][0]):
            log_line = 'SearchIteration new runner up (%g) Moved to:'%score
            for p in self.Parameters:
                log_line += ' %s=%g'%(p.Identifier, params[p.Identifier])
            logger.debug(log_line)
            self.register_runner_up (score, params)

        return self.increment_params(self.param_positions)

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

    def register_runner_up(self, score, params):
        self.runners_up.append ((score, params))
        self.runners_up.sort(None, None, True)
        if len(self.runners_up) > self.KeepNRunnersUp:
            del self.runners_up[-1]

    def register_global_best(self, pnum, direction):
        for i in range(pnum):
            self.Parameters[i].RegisterNewGlobalBest(0)
        self.Parameters[pnum].RegisterNewGlobalBest(direction)
        for i in range(pnum+1,len(self.Parameters),1):
            self.Parameters[i].RegisterNewGlobalBest(0)

    def register_local_best(self, pnum, direction):
        for i in range(pnum):
            self.Parameters[i].RegisterNewLocalBest(0)
        self.Parameters[pnum].RegisterNewLocalBest(direction)
        for i in range(pnum+1,len(self.Parameters),1):
            self.Parameters[i].RegisterNewLocalBest(0)

def assign_global_best(p,v):
    p.global_best_value=v

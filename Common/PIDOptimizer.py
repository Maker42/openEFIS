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

import sys, os, logging

if '__main__' == __name__:
    sys.path.append (os.path.join ('..', 'FaceProcessing'))
    sys.path.append (os.path.join ('..', 'Common'))

import FileConfig
import PID
import Optimizer

STEP_TYPE_START='START'
STEP_TYPE_CHANGE_GOAL='CHANGE_GOAL'
STEP_TYPE_SUSPEND='SUSPEND'
STEP_TYPE_RESUME='RESUME'
STEP_TYPE_MANUAL_INPUT='MANUAL_INPUT'

class PIDScoring(FileConfig.FileConfig):
    def __init__(self, ref=None):
        if not ref:
            self.TestSteps = list()

            # Relative importance of PID behaviors
            self.VariancePenalty = 1
            self.MaxPositiveFirstDerivative = 1
            self.MaxNegativeFirstDerivative = 1
            self.PositiveFirstDerivativePenalty = 1
            self.NegativeFirstDerivativePenalty = 1
            self.SecondDerivativePenalty = 1
            self.GoalCrossingPenalty = 1
            self.Optimizer = Optimizer.Optimizer()
        else:
            self.TestSteps = ref.TestSteps

            # Relative importance of PID behaviors
            self.VariancePenalty = ref.VariancePenalty
            self.MaxPositiveFirstDerivative = ref.MaxPositiveFirstDerivative
            self.MaxNegativeFirstDerivative = ref.MaxNegativeFirstDerivative
            self.PositiveFirstDerivativePenalty = ref.PositiveFirstDerivativePenalty
            self.NegativeFirstDerivativePenalty = ref.NegativeFirstDerivativePenalty
            self.SecondDerivativePenalty = ref.SecondDerivativePenalty
            self.GoalCrossingPenalty = ref.GoalCrossingPenalty
            self.Optimizer = ref.Optimizer

        # Internal State
        self._last_out = 0
        self._last_first_derivative = 0
        self._last_second_derivative = 0
        self._last_less_than_goal = False
        self._accumulated_score = 0
        self._current_step = 0

        list_actions = {"TestSteps" : ("TestStep", self.make_test_step), 
                        "parameters" : ("parameter", self.add_parameter)}
        FileConfig.FileConfig.__init__(self,list_actions=list_actions)

    def make_test_step(self, args, filelines):
        if len(args) != 2:
            raise RuntimeError("TestStep must have a step type argument")
        newstep = TestStep()
        newstep.initialize(args, filelines)
        self.TestSteps.append(newstep)

    def add_parameter(self, args, filelines):
        newparam = Optimizer.OptParameter()
        newparam.InitializeFromFileLines(filelines)
        self.Optimizer.Parameters.append (newparam)

    def initialize(self, filelines):
        self.InitializeFromFileLines (filelines)

    def IncrementScore(self, inp, outp, goal, outfile):
        self._accumulated_score -= (abs(goal-outp) * self.VariancePenalty)
        first_derivative = outp - self._last_out
        if first_derivative > self.MaxPositiveFirstDerivative:
            self._accumulated_score -= ((first_derivative - self.MaxPositiveFirstDerivative) * self.PositiveFirstDerivativePenalty)
        elif first_derivative < self.MaxNegativeFirstDerivative:
            self._accumulated_score -= ((self.MaxNegativeFirstDerivative - first_derivative) * self.NegativeFirstDerivativePenalty)
        second_derivative = self._last_first_derivative - first_derivative
        self._accumulated_score -= (abs(second_derivative) * self.SecondDerivativePenalty)
        less_than_goal = (outp < goal)
        if self._last_less_than_goal != less_than_goal:
            self._accumulated_score -= self.GoalCrossingPenalty
        if outfile:
            outfile.write("%g,%g,%g,%g,%g\n"%(inp,outp,goal,first_derivative, second_derivative))
        self._last_first_derivative = first_derivative
        self._last_out = outp
        self._last_less_than_goal = less_than_goal

    def InitializeScoring(self,fd=0,sd=0):
        self._current_step = 0
        step = self.GetNextStep()
        inp,outp,g = step.input_value
        self._last_out = outp
        self._last_first_derivative = fd
        self._last_second_derivative = sd
        self._accumulated_score = 0.0
        return step

    def GetScore(self):
        return self._accumulated_score

    def GetNextStep(self):
        if self._current_step >= len(self.TestSteps):
            return None
        ret = self.TestSteps[self._current_step]
        self._current_step += 1
        return ret

class TestStep(FileConfig.FileConfig):
    def __init__(self):
        self.step_type = ''
        self.input_value = 0
        self.periods = 1
        FileConfig.FileConfig.__init__(self)

    def initialize(self, args, filelines):
        assert (len(args) == 2)
        self.step_type = args[1]
        self.InitializeFromFileLines (filelines)


def OptimizePID(scoring_object, mut, lines):
    scoring_object.Optimizer.StartOptimize()
    score = None
    while True:
        params = scoring_object.Optimizer.OptimizeIteration(score)
        if not params:
            break
        score = mut.Score(scoring_object, params, None)
    return scoring_object.Optimizer.GetGlobalBestParams(),scoring_object.Optimizer.GetGlobalBestScore()

# A simple theory cart for unit testing
class Cart(FileConfig.FileConfig):
    def __init__(self):
        self.MotorForceCoefficient = 1
        self.MotorMuS = .001
        self.MotorMuD = .01
        self.Mass = 10
        self.Velocity = 0
        self.InputMax = 255
        self.InputMin = 255
        FileConfig.FileConfig.__init__(self)

    def initialize(self, filelines):
        self.InitializeFromFileLines (filelines)

    def InitialConditions (self, inp, outp):
        self.Velocity = outp

    def ComputeNext (self, inp):
        # F = MA
        if self.Velocity == 0:
            friction = self.MotorMuS * self.Mass
        else:
            friction = self.MotorMuD * self.Mass * abs(self.Velocity)
        F = self.MotorForceCoefficient * inp - friction
        A = F / self.Mass
        self.Velocity += A
        return self.Velocity

    def Score(self, Scoring, parameters, outfilename):
        outfile = None
        if outfilename:
            outfile = open(outfilename, 'w')

        test_step = Scoring.InitializeScoring(0,0)
        inp,outp,goal = test_step.input_value
        # Pid Under Test: put
        ms = 0
        put = PID.PID (goal,parameters['P'], parameters['I'], parameters['D'], PID.DIRECT, ms)
        put.SetMode (PID.AUTOMATIC, outp, inp)
        put.SetOutputLimits (self.InputMin, self.InputMax)

        self.InitialConditions (inp, outp)
        if outfile:
            outfile.write("Input,Output,Goal,FirstDerivative,SecondDerivative\n")
        while True:
            put.SetSetPoint (goal)
            for i in range(test_step.periods):
                ms += 10
                inp = put.Compute (outp, ms)
                outp = self.ComputeNext (inp)
                Scoring.IncrementScore(inp,outp,goal,outfile)
            test_step = Scoring.GetNextStep()
            if not test_step:
                break
            if test_step.step_type == STEP_TYPE_CHANGE_GOAL:
                goal = test_step.input_value
            else:
                raise RuntimeError("Unimplemented test step type %s"%test_step.step_type)
        return Scoring.GetScore()

if '__main__' == __name__:
    rootlogger = logging.getLogger()
    rootlogger.setLevel(1)

    if os.path.exists('info.log'):
        os.unlink('info.log')
    file_handler = logging.FileHandler('info.log')
    file_handler.setLevel(logging.DEBUG)
    rootlogger.addHandler(file_handler)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    rootlogger.addHandler(console_handler)

    test_file = sys.argv[1]
    if test_file:
        if os.path.exists(test_file):
            file = open (test_file, 'r')
            lines = file.readlines()
            file.close()
            scoring_object = PIDScoring()
            scoring_object.initialize(lines)
            MachineUnderTest = Cart()
            MachineUnderTest.initialize (lines)
        else:
            raise RuntimeError ("Command line argument must be a test configuration file")
    else:
        raise RuntimeError ("A configuration test file must be provided on the command line")

    parameters,score = OptimizePID (scoring_object, MachineUnderTest, lines)
    print ("Best parameters are %g,%g,%g with a score of %g"%(parameters['P'], parameters['I'], parameters['D'], score))
    if len(sys.argv) >= 3:
        print ("Creating output %s"%sys.argv[2])
        MachineUnderTest.Score (scoring_object, parameters, sys.argv[2])
    else:
        print ("Got fewer args %s"%sys.argv)


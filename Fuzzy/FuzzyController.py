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

import time, logging

import FileConfig

import efll

logger=logging.getLogger(__name__)

class FuzzyController(FileConfig.FileConfig):
    def __init__(self):
        self._general_engine = efll.Fuzzy() # The fuzzy engine with general guessed values that work in most
                                        # circumstances. It is used as a fallback when the specific measured
                                        # engine fails to recognize (fire) the antecedent input sets, or in other
                                        # words when we're in a circumstance not yet encountered.

        self._measured_engine = efll.Fuzzy()   # The fuzzy engine built from sets of previously measured behavior
        self._general_rule_index = 1
        self._measured_rule_index = 1

        self.InputAntecedentsWidth = list()  # How wide the input fuzzy set for each antecedent measured point.
        self.OutputConsquentWidth = 1.0

        self._measured_inputs = list()
        self._measured_input_sets = list()
        self.GeneralInputs = dict()
        self.GeneralOutputSets = dict()
        self._general_output = efll.FuzzyOutput(1)
        self._measured_output = efll.FuzzyOutput(1)
        self._outputs_registered = False
        self.MeasuredRules = list()
        # The following lists are for object retention, and preventing the garbage collector from deleting them
        self._consequents = list()
        self._antecedents = list()
        self._output_sets = list()
        self._rules = list()

        list_actions = {'MeasuredRules' : ('mrule', self._new_measured_rule),
                'GeneralRules' : ('grule', self._new_general_rule),
                'GeneralInputs' : ('input', self._new_general_input),
                'GeneralOutputSets' : ('oset', self._new_general_output_set)}
        FileConfig.FileConfig.__init__(self, list_actions=list_actions)

    def _new_measured_rule(self, args, filelines):
        rule = MeasuredRule()
        rule.initialize(args, filelines)
        if len(self._measured_inputs) == 0:
            i = 1
            for iset in rule.InputCenters:
                self._measured_inputs.append (efll.FuzzyInput(i))
                self._measured_engine.addFuzzyInput(self._measured_inputs[-1])
                i += 1
            self._measured_engine.addFuzzyOutput(self._measured_output)

        if len(self._measured_inputs) != len(rule.InputCenters):
            raise RuntimeError ("Number of input centers (%d) != number of inputs (%d)"%(len(rule.InputCenters),
                                len(self._measured_inputs)))
        i = 0
        fsets = list()
        for iset in rule.InputCenters:
            fsets.append(efll.FuzzySet(iset - self.InputAntecedentsWidth[i], iset, iset, iset + self.InputAntecedentsWidth[i]))
            i += 1
        i = 0
        for fset in fsets:
            self._measured_inputs[i].addFuzzySet(fset)
            self._measured_input_sets.append(fset)
            i += 1
        last_ant = efll.FuzzyRuleAntecedent()
        self._antecedents.append(last_ant)
        if len(fsets) > 1:
            last_ant.joinWithAND(fsets[0], fsets[1])
            for input_number in range(2,len(fsets)):
                next_ant = efll.FuzzyRuleAntecedent()
                next_ant.joinWithAND (last_ant, fsets[input_number])
                self._antecedents.append(next_ant)
                last_ant = next_ant
        else:
            last_ant.joinSingle(fsets[0])

        consequent = efll.FuzzyRuleConsequent()
        self._consequents.append(consequent)
        outset = efll.FuzzySet(rule.OutputValue - self.OutputConsquentWidth, rule.OutputValue, rule.OutputValue,
                               rule.OutputValue + self.OutputConsquentWidth)
        self._measured_output.addFuzzySet (outset)
        consequent.addOutput (outset)
        self._output_sets.append(outset)
        self._consequents.append(consequent)
        try:
            consequent.addOutput(outset)
        except Exception,e:
            logger.error ("Output set consequent failed with error %s", str(e))
            return
        rule = efll.FuzzyRule(self._measured_rule_index, last_ant, consequent)
        self._rules.append(rule)
        self._measured_engine.addFuzzyRule (rule)
        self._measured_rule_index += 1
        return

    def _new_general_rule(self, args, filelines):
        if not self._outputs_registered:
            self._general_engine.addFuzzyOutput(self._general_output)
            self._outputs_registered = True
        rule = GeneralRule()
        rule.initialize(args, filelines)

        input_sets = list()
        input_number = 0
        for inp in rule.SpecifiedInputs:
            dot = inp.find('.')
            if dot >= 0:
                inpname,setname = inp.split('.', 1)
                try:
                    input_sets.append (self.GeneralInputs[inpname].Sets[setname])
                except Exception,e:
                    logger.error ("Input set lookup (%s,%s) for rule %d got error %s",
                            inpname, setname, self._general_rule_index, str(e))
                    return
            else:
                logger.error ("Input set %d must be a dotted name specification in rule %d",
                        input_number, self._general_rule_index)
                return
            input_number += 1
            # Find the input 
        last_ant = efll.FuzzyRuleAntecedent()
        self._antecedents.append(last_ant)
        if len(input_sets) > 1:
            last_ant.joinWithAND(input_sets[0], input_sets[1])
            for input_number in range(2,len(input_sets)):
                next_ant = efll.FuzzyRuleAntecedent()
                next_ant.joinWithAND (last_ant, input_sets[input_number])
                self._antecedents.append(next_ant)
                last_ant = next_ant
        else:
            last_ant.joinSingle(input_sets[0])

        consequent = efll.FuzzyRuleConsequent()
        self._consequents.append(consequent)
        try:
            consequent.addOutput(self.GeneralOutputSets[rule.ImpliedOutput])
        except Exception,e:
            logger.error ("Output set consequent failed with error %s", str(e))
            return
        general_rule = efll.FuzzyRule(self._general_rule_index, last_ant, consequent)
        self._rules.append(general_rule)
        self._general_rule_index += 1
        self._general_engine.addFuzzyRule (general_rule)

    def _new_general_input(self, args, filelines):
        inp = GeneralInput(len(self.GeneralInputs) + 1)
        inp.initialize(args, filelines)
        self.GeneralInputs[inp.Name] = inp
        logger.debug ("Add fuzzy input %s", str(inp._general_input))
        self._general_engine.addFuzzyInput (inp._general_input)

    def _new_general_output_set(self, args, filelines):
        oset = GeneralOutputSet ()
        oset.initialize (args, filelines)
        self.GeneralOutputSets[oset.Name] = oset._general_oset
        self._output_sets.append(oset._general_oset)
        self._general_output.addFuzzySet (oset._general_oset)

    def initialize(self, filelines):
        self.InitializeFromFileLines (filelines)

    def Compute(self, inputs, enable_general=False):
        i = 1
        for inp in inputs:
            self._measured_engine.setInput (i, inp)
            i += 1
        self._measured_engine.fuzzify()
        if self._measured_engine.anyFiredRule():
            return self._measured_engine.defuzzify(1)

        # No measured data for this circumstance
        if enable_general:
            i = 1
            for inp in inputs:
                if not self._general_engine.setInput (i, inp):
                    raise RuntimeError("Failed to set fuzzy input %d for measured engine"%i)
                i += 1
            self._general_engine.fuzzify()
            if not self._general_engine.anyFiredRule():
                raise RuntimeError("Input outside the defined boundaries of the general fuzzy engine")
            return self._general_engine.defuzzify(1)
        else:
            return None

    def CompareSets(self, inp1, inp2, irange):
        b,e = irange
        for i in range(b,e):
            s1 = efll.FuzzySet(inp1[i] - self.InputAntecedentsWidth[i],
                               inp1[i], inp1[i],
                               inp1[i] + self.InputAntecedentsWidth[i])
            s1.calculatePertinence (inp2[i])
            if s1.getPertinence() == 0:
                return False
        else:
            return True


class MeasuredRule(FileConfig.FileConfig):
    def __init__(self, inputs = None, output = None):
        self.InputCenters = inputs
        self.OutputValue = output
        FileConfig.FileConfig.__init__(self)

    def initialize(self, args, filelines):
        self.InitializeFromFileLines (filelines)

    def Record(self, ofile):
        ofile.write ('mrule\n')
        ofile.write ('  InputCenters %s\n'%str(self.InputCenters))
        ofile.write ('  OutputValue %g\n\n'%self.OutputValue)


class GeneralRule(FileConfig.FileConfig):
    def __init__(self):
        self.ImpliedOutput = None
        self.SpecifiedInputs = list()
        FileConfig.FileConfig.__init__(self)

    def initialize(self, args, filelines):
        self.InitializeFromFileLines (filelines)


class GeneralInput(FileConfig.FileConfig):
    def __init__(self, input_index):
        self.Name = None
        self.Sets = dict()
        self._general_input = efll.FuzzyInput(input_index)
        list_actions={'InputSets' : ('set', self._new_input_set)}
        FileConfig.FileConfig.__init__(self, list_actions=list_actions)

    def initialize(self, args, filelines):
        if len(args) > 1:
            self.Name = args[1]
        self.InitializeFromFileLines(filelines)
        logger.debug ("New General input %s with sets %s", str(self), str(self.Sets))


    def _new_input_set(self, args, filelines):
        if len(args) > 2:
            name = args[1]
            numbers = args[2:]
            while len(numbers) < 4:
                numbers.append(numbers[-1])
            try:
                numbers = [float(n) for n in numbers]
            except Exception, e:
                logger.error ("Error %s: General rule input sets must be floating point numbers. Got %s",
                    str(e), " ".join(numbers))
                return
            if self.Name == None:
                logger.error ("Input set must have a name")
                return
            newset = efll.FuzzySet(numbers[0], numbers[1], numbers[2], numbers[3])
            self.Sets[name] = newset
            self._general_input.addFuzzySet(newset)
        else:
            logger.error ("General input must have at least two argument (name and number) Got: %s"%("".join(numbers)))

class GeneralOutputSet(FileConfig.FileConfig):
    def __init__(self):
        self.Name = None
        self._general_oset = None
        FileConfig.FileConfig.__init__(self)

    def initialize(self, args, filelines):
        if len(args) < 3:
            logger.error("Output set must have arguments for name and 4 parameters. Got %s", " ".join(args))
        else:
            self.Name = args[1]
            numbers = args[2:]
            while len(numbers) < 4:
                numbers.append(numbers[-1])
            try:
                numbers = [float(n) for n in numbers]
            except Exception, e:
                logger.error ("Error %s: General rule input sets must be floating point numbers. Got %s",
                    str(e), " ".join(numbers))
            if self.Name == None:
                logger.error ("Input set must have a name")
                return
            self._general_oset = efll.FuzzySet(numbers[0], numbers[1], numbers[2], numbers[3])

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

import sys, os

for p in sys.path:
    if 'Common' in p:
        break
else:
    sys.path.append (os.path.join ('..', 'Common'))
    sys.path.append (os.path.join ('..', 'Test'))
    sys.path.append (os.path.join ('..', 'Fuzzy'))

from FuzzyController import MeasuredRule

rules = dict()

def GetBinNumber(inp, inc):
    ret = inp / inc
    ret = int(round(ret))
    return str(ret)

def GetBinID(rule, cell_size):
    if len(cell_size) != len(rule.InputCenters):
        raise RuntimeError ("cell size list does not have the same size as inputs")
    bnn = GetBinNumber (rule.InputCenters[0], cell_size[0])
    for index in range(1,len(cell_size)):
        bnn += '_'
        bnn += GetBinNumber(rule.InputCenters[index], cell_size[index])
    return bnn

def AddRule(rule, cell_size):
    bnn = GetBinID(rule, cell_size)
    if rules.has_key(bnn):
        rules[bnn].append(rule)
    else:
        rules[bnn] = [rule]

def FillBuffer(lines, f):
    nlines = 1000 - len(lines)
    while nlines > 0:
        l = f.readline()
        lines.append(l)
        nlines -= 1


def ReadRules(filename, cell_size):
    f = open(filename, 'r')
    lines = list()
    while True:
        FillBuffer(lines, f)
        rule = MeasuredRule()
        if len(lines) == 0:
            break
        while lines[0].find('mrule') < 0:
            del lines[0]
            if len(lines) == 0:
                break
        if len(lines) < 2:
            break
        del lines[0]
        rule.initialize(list(), lines)
        AddRule (rule, cell_size)
    f.close()

def ConsolidateRules():
    consolidated_rules = dict()
    for key in rules.iterkeys():
        rule_group = rules[key]
        means = list()
        if len(rule_group) > 1:
            print ("%s contains %d rules:"%(key, len(rule_group)))
        for index in range(len(rule_group[0].InputCenters)):
            centers = [c.InputCenters[index] for c in rule_group]
            sm = reduce(lambda x,y: x+y, centers)
            mean = sm / len(rule_group)
            means.append(mean)
            if len(rule_group) > 1:
                print ("centers[%d] = %s, mean = %g"%(index, str(centers), mean))
        centers = [c.OutputValue for c in rule_group]
        sm = reduce(lambda x,y: x+y, centers)
        output_mean = sm / len(rule_group)
        if len(rule_group) > 1:
            print ("output_centers = %s, mean = %g"%(str(centers), output_mean))
        consolidated_rules[key] = MeasuredRule (means, output_mean)
    return consolidated_rules

def WriteRules(crules, filename):
    print ("Writing out %d rules"%len(crules))
    f = open(filename, 'w')
    f.write ("MeasuredRules\n")
    for v in crules.itervalues():
        v.Record (f)
    f.close()

if len(sys.argv) < 4:
    print ("Usage: ExperienceConsolidation.py experience_file consolidated_file cell_sizes")
    sys.exit(-1)

cell_sizes=eval(sys.argv[3])

ReadRules(sys.argv[1], cell_sizes)
crules = ConsolidateRules()
WriteRules(crules, sys.argv[2])

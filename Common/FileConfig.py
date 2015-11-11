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

import logging

logger = logging.getLogger(__name__)

_object_stack = list()

class FileConfig:
    def __init__(self, single_arg_actions=None, double_arg_actions=None, list_actions=None, comment_action=None, ending_arg=None):
        self._single_arg_actions = single_arg_actions
        self._double_arg_actions = double_arg_actions
        self._list_actions = list_actions
        self._comment_action = comment_action
        self._ending_arg = ending_arg

    def InitializeFromFileLines(self, filelines):
        if len(filelines) > 0:
            logger.debug("start reading for %s at %s", str(self), filelines[0])
        while len(filelines) > 0:
            args = filelines[0].split()
            if len(args) == 0:
                del filelines[0]
                continue
            elif args[0][0] == '#':
                del filelines[0]
                if self._comment_action:
                    self._comment_action (args)
                continue
            # Remove any trailing comments
            item = 0
            for a in args:
                if '#' in a:
                    break
                item += 1
            if item < len(args):
                if '#' == args[item][0]:
                    args = args[0:item]
                else:
                    args = args[0:item+1]
                    pos = args[item].find('#')
                    args[item] = args[0:pos]
            line_not_processed = False
            if len(args) == 1:
                if self._single_arg_actions != None and args[0] in self._single_arg_actions:
                    del filelines[0]
                    _object_stack.append(self)
                    self._single_arg_actions[args[0]] (args, filelines)
                    del _object_stack[-1]
                elif self._list_actions != None and args[0] in self._list_actions:
                    del filelines[0]
                    self._process_list (args, filelines)
                elif self.is_ending_arg(args):
                    del filelines[0]
                    break
                else:
                    line_not_processed = True
                    logger.debug("unrecognized configuration word %s for %s", args[0], str(self))
                    break
            else:
                if self._double_arg_actions != None and str(args[0]) in self._double_arg_actions:
                    del filelines[0]
                    _object_stack.append(self)
                    self._double_arg_actions[args[0]] (args, filelines)
                    del _object_stack[-1]
                elif args[0] == '+log_name' and hasattr(self, 'log_name') and hasattr(self, 'coordinate_space'):
                    del filelines[0]
                    self.log_name = self.coordinate_space.GetFullName() + args[1]
                elif args[0] == 'log_name+' and hasattr(self, 'log_name') and hasattr(self, 'coordinate_space'):
                    del filelines[0]
                    self.log_name = args[1] + self.coordinate_space.GetFullName()
                elif self._list_actions != None and args[0] in self._list_actions:
                    del filelines[0]
                    self._process_list (args, filelines)
                elif hasattr(self,  args[0]):
                    del filelines[0]
                    setattr(self,  args[0], eval(' '.join(args[1:])))
                elif self.is_ending_arg(args):
                    del filelines[0]
                    break
                else:
                    logger.debug("unrecognized configuration word %s for %s", args[0], str(self))
                    line_not_processed = True
                    break
            if self.is_ending_arg(args):
                if line_not_processed:
                    del filelines[0]
                break
        if (len(filelines) > 0):
            logger.debug("end reading for %s at %s", str(self), filelines[0])
        else:
            logger.debug("end reading for %s at EOF", str(self))


    def is_ending_arg(self, args):
        if self._ending_arg != None:
            if isinstance(self._ending_arg, str):
                if args[0] == self._ending_arg:
                    return True
            elif args[0] in self._ending_arg:
                return True
        return False


    def _process_list(self, args, filelines):
        element_name,action = self._list_actions[args[0]]

        while len(filelines) > 0:
            args = filelines[0].split()
            if len(args) == 0:
                del filelines[0]
            elif args[0][0] == '#':
                del filelines[0]
                if self._comment_action:
                    self._comment_action (args)
            elif len(args) == 1:
                if args[0] == element_name:
                    del filelines[0]
                    action (args, filelines)
                elif self._single_arg_actions != None and args[0] in self._single_arg_actions:
                    del filelines[0]
                    self._single_arg_actions[args[0]] (args, filelines)
                else:
                    break
            elif args[0] == element_name:
                del filelines[0]
                action (args, filelines)
            else:
                break

    def AddActions(self, single_arg_actions=None, double_arg_actions=None, list_actions=None, comment_action=None, ending_arg=None):
        if single_arg_actions:
            if self._single_arg_actions:
                self._single_arg_actions += single_arg_actions
            else:
                self._single_arg_actions = single_arg_actions

        if double_arg_actions:
            if self._double_arg_actions:
                self._double_arg_actions += double_arg_actions
            else:
                self._double_arg_actions = double_arg_actions

        if list_actions:
            if self._list_actions:
                self._list_actions += list_actions
            else:
                self._list_actions = list_actions

        if comment_action:
            self._comment_action = comment_action

        if ending_arg:
            self._ending_arg = ending_arg


def read_coordinate_space(args, filelines):
    obj = _object_stack[-1]
    obj.coordinate_space.initialize (filelines)
    if hasattr(obj, 'log_name') and (not obj.log_name) and hasattr(obj, 'coordinate_space'):
        fullname = obj.coordinate_space.GetFullName()
        if fullname.startswith ("TheWorkSpace.robot."):
            fullname = fullname.replace ("TheWorkSpace.robot.", "")
        obj.log_name = fullname


def read_coordinate_oneline (args, filelines):
    obj = _object_stack[-1]
    obj.coordinate.x,  obj.coordinate.y,  obj.coordinate.z = eval (' '.join(args[1:]))

def read_orientation (args, filelines):
    obj = _object_stack[-1]
    obj.orientation.initialize (filelines)

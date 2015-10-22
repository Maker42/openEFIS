# Copyright (C) 2012-2014  Garrett Herschleb
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

import time, math

M_PI = math.pi

def read_cont_expr (first_args, lines):
    ret = None
    line = ' '.join(first_args)
    while not ret:
        try:
            ret = eval(line)
            return ret
        except:
            if len (lines) == 0:
                raise RuntimeError ('Incomplete expression: ' + line)
            line += lines[0]
            del lines[0]

def millis():
    t = time.time()
    # TODO: Handle positive/negative wrap
    return int((t * 1000)%2147483648)

def rotate2d(angle,  x,  y):
    xprime = x * math.cos(angle) - y * math.sin(angle)
    yprime = y * math.cos(angle) + x * math.sin(angle)
    return xprime, yprime

# Computes true heading from a given course
def TrueHeading(course):
    lng1,lat1 = course[0]
    lng2,lat2 = course[1]
    dlng = lng2 - lng1
    dlat = lat2 - lat2
    return math.atan2(dlng, dlat) * 180 / M_PI

# Get a magnetic heading from a true heading or course and the current position
def MagneticHeading(true_heading_or_course, current_position):
    if isinstance(true_heading_or_course,list):
        true_heading = TrueHeading(true_heading_or_course)
    else:
        true_heading = true_heading_or_course
    # mag_north_position = ()
    # TODO:
    raise RuntimeError("Not Implemented")

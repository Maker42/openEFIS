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

import time, math, copy, logging

import Spatial

logger=logging.getLogger(__name__)

M_PI = math.pi
NAUT_MILES_PER_METER = .0005399568
# m / s^2 * nm / m
# nm / s^2 * 3600 s / hour
# nm / s * h
# knots / s
G_IN_KNOTS_PER_SEC = 9.98 * NAUT_MILES_PER_METER * 3600.0
RAD_DEG = M_PI / 180.0
DEG_RAD = 180.0 / M_PI

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

def millis(t):
    # TODO: Handle positive/negative wrap
    return int((t * 1000)%2147483648)

def rotate2d(angle,  x,  y):
    xprime = x * math.cos(angle) - y * math.sin(angle)
    yprime = y * math.cos(angle) + x * math.sin(angle)
    return xprime, yprime

def get_polar_deltas(course):
    lng1,lat1 = course[0]
    lng2,lat2 = course[1]
    dlng = lng2 - lng1
    dlat = lat2 - lat1
    return (dlng,dlat)

MAX_COURSE_HOP_LENGTH_DEG = 1.0/3.0
MAX_COURSE_HOP_LENGTH_RAD = MAX_COURSE_HOP_LENGTH_DEG * M_PI/180.0

def GetRelLng(lat1):
    rel_lng_inc = Spatial.Polar(0.0,lat1,1.0).to3(robot_coordinates=False)
    rel_lng = Spatial.Polar(MAX_COURSE_HOP_LENGTH_RAD,lat1,1.0).to3(robot_coordinates=False)
    rel_lng.sub(rel_lng_inc)

    rel_lat_inc = Spatial.Polar(0.0,lat1,1.0).to3(robot_coordinates=False)
    rel_lat = Spatial.Polar(0.0,lat1+MAX_COURSE_HOP_LENGTH_RAD,1.0).to3(robot_coordinates=False)
    rel_lat.sub(rel_lat_inc)
    return rel_lng.norm() / rel_lat.norm()

def GetAdjustedPolarDeltas(course):
    dlng,dlat = get_polar_deltas(course)
    rel_lng = GetRelLng(course[0][1])
    return (dlng*rel_lng, dlat)

# Computes true heading from a given course
def TrueHeadingAndDistance(course, periodic_logging=None, frequency=10):
    dlng,dlat = get_polar_deltas(course)
    if dlng > MAX_COURSE_HOP_LENGTH_DEG or dlat > MAX_COURSE_HOP_LENGTH_DEG:
        expanded_course = ExpandCourseList(course, MAX_COURSE_HOP_LENGTH_DEG)
        dlng,dlat = get_polar_deltas(expanded_course)
    lat1 = course[0][1] * M_PI / 180.0

    # Determine how far is a longitude increment relative to latitude at this latitude
    relative_lng_length = GetRelLng(lat1)
    dlng *= relative_lng_length

    heading = atan_globe(dlng, dlat) * 180 / M_PI
    distance = math.sqrt(dlng * dlng + dlat * dlat) * 60.0      # Multiply by 60 to convert from degrees to nautical miles.
    if periodic_logging:
        log_occasional_info (periodic_logging, "course (%g,%g)->(%g,%g) d=(%g,%g), rel_lng=%g, distance=%g"%(
            course[0][0], course[0][1],
            course[1][0], course[1][1],
            dlng, dlat, relative_lng_length, distance
            ), frequency)

    return heading, distance, relative_lng_length

def TrueHeading (course, periodic_logging=None, frequency=10):
    ret,a,b = TrueHeadingAndDistance(course, periodic_logging, frequency)
    return ret

def deg_min_to_radians(deg,minutes):
    return ((d + m/60.0) * M_PI / 180.0)

def get_intermediate_points(begin, end, degrees_per_hop):
    radians_per_hop = degrees_per_hop * M_PI / 180.0
    begin = (begin[0] * M_PI / 180.0, begin[1] * M_PI / 180.0)       # Convert to radians for spatial math
    end = (end[0] * M_PI / 180.0, end[1] * M_PI / 180.0)
    last_waypoint = Spatial.Polar(begin[0], begin[1], 1.0)
    org = last_waypoint.to3(robot_coordinates=False)
    route_vec = Spatial.Polar(end[0], end[1], 1.0).to3(robot_coordinates=False)
    r = Spatial.Ray(org, pos2=route_vec)
    route_vec.sub(org)
    total_time = route_vec.norm()
    cartinc1 = Spatial.Polar(0.0,0.0,1.0).to3(robot_coordinates=False)
    incvec = Spatial.Polar(0.0,radians_per_hop,1.0).to3(robot_coordinates=False)
    incvec.sub(cartinc1)
    inc_time = incvec.norm()

    ret = list()
    tm = 0.0
    while True:
        # For determining the time increment:
        radius = last_waypoint.rad
        # time_increment / radius = tan (radians_per_hop)
        time_increment = math.tan(radians_per_hop) * radius
        tm += time_increment
        if tm >= total_time:
            break
        waypoint = r.project(tm).to_polar(robot_coordinates=False)
        ret.append ((waypoint.theta * 180.0 / M_PI, waypoint.phi * 180.0 / M_PI))
        last_waypoint = waypoint
    return ret

def ExpandCourseList(course, degrees_per_hop):
    last_point = 0
    ret = [course[0]]
    for this_point in range(1,len(course)):
        ret += get_intermediate_points(course[last_point], course[this_point], degrees_per_hop)
        ret.append (course[this_point])
    return ret

log_sources = dict()

def log_occasional_info(source, string, frequency=10):
    global log_sources
    if not (source in log_sources):
        log_sources[source] = 0
    else:
        if log_sources[source] == 0:
            logger.info (source + ": " + string)
        log_sources[source] += 1
        if log_sources[source] >= frequency:
            log_sources[source] = 0

def get_rate(current, desired, time_divisor, limits):
    error = desired - current
    rate = error / time_divisor
    if isinstance(limits,tuple) or isinstance(limits,list):
        mn = limits[0]
        mx = limits[1]
    else:
        mn = -limits
        mx = limits

    if rate > mx:
        rate = mx
    if rate < mn:
        rate = mn
    return rate

def rate_curve(x, curve_pieces):
    last_piece = 0
    for piece in range(1,len(curve_pieces)):
        if x >= curve_pieces[last_piece][0] and x < curve_pieces[piece][0]:
            x0 = curve_pieces[last_piece][0]
            x1 = curve_pieces[piece][0]
            if x >= x0 and x < x1:
                y0 = curve_pieces[last_piece][1]
                y1 = curve_pieces[piece][1]
                xpiece = (x - x0) / (x1 - x0)
                y = xpiece * (y1 - y0) + y0
                logger.log(3, "rate_curve: x=%g [%g,%g]==> %g ; y = %g [%g,%g]",
                        x, x0, x1, xpiece, y, y0, y1)
                return y
        last_piece = piece
    else:
        return curve_pieces[-1][1]

def AddPosition(position, distance, direction, rel_lng = 0.0):
    if not rel_lng:
        rel_lng = GetRelLng(position[1])
    direction *= M_PI / 180.0
    dlat = distance * math.sin(direction) / 60.0
    dlng = disatnce * math.cos(direction) * rel_lng / 60.0
    position = (position[0] + dlng, position[1] + dlat)
    return position


def atan_globe(lng, lat):
    return math.atan2(lng,lat)

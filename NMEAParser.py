# Copyright (C) 2015-2018  Garrett Herschleb
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

import time
from calendar import timegm

import logging

logger=logging.getLogger(__name__)

def CheckNMEAChecksum(checksum, s):
    for i in range(2,len(s)):
        checksum ^= ord(s[i])
    return False if checksum != 0 else True

METER_FOOT = 0.3048
FOOT_METER = 1.0 / METER_FOOT
TWELVE_HOURS = 60 * 60 * 12

def ParseNMEAStrings(nmea, data_container):
    ret = False
    if isinstance(nmea, str) and nmea.startswith("$G") and nmea.find('*') > 0:
        chki = nmea.index('*')
        body = nmea[:chki]
        checkstring = nmea[chki+1:]
        checkstring = checkstring.strip()
        if len(checkstring) == 2:
            try:
                checksum = int(float.fromhex(checkstring))
            except:
                logger.debug("Bad checksum sentence from GPS: %s", nmea)
                return False
        else:
            logger.debug("Mal-formed GPS sentence: %s", nmea)
            return False
        if CheckNMEAChecksum(checksum,body):
            logger.debug("Invalid checksum for GGA message %s"%nmea)
            return False
    else:
        return False
    sentences = body.split('$')
    for sentence in sentences:
        if sentence.startswith("GPGGA"):
            fields = sentence.split(',')
            try:
                data_container.gps_signal_quality = int(fields[6])
                if data_container.gps_signal_quality > 0:
                    utc_hh = int(fields[1][0:2])
                    utc_mm = int(fields[1][2:4])
                    utc_ss = float(fields[1][4:])
                    data_container.gps_utc = make_unix_time (utc_hh, utc_mm, utc_ss)
                    lat = float(fields[2]) / 100.0
                    if fields[3] == 'S':
                        lat *= -1.0
                    data_container.gps_lat = lat
                    lng = float(fields[4]) / 100.0
                    if fields[5] == 'W':
                        lng *= -1.0
                    data_container.gps_lng = lng
                    alt = float(fields[9])
                    if fields[10] == 'M':
                        alt *= FOOT_METER
                    data_container.gps_altitude = alt
                    logger.log(2, "NMEA got GGA: time = %g, pos=%g,%g, alt=%d",
                            data_container.gps_utc,
                            data_container.gps_lat,
                            data_container.gps_lng,
                            data_container.gps_altitude)
                    ret = True
                    data_container.HaveNewPosition = True
                    data_container.GGAUpdate = True
                else:
                    data_container.gps_utc = None
                    data_container.gps_lat = None
                    data_container.gps_lng = None
                    data_container.gps_altitude = None
                    data_container.gps_signal_quality = 0
                    data_container.HaveNewPosition = False
                    logger.error("NMEA got GGA with no signal quality")
            except Exception as e:
                logger.debug("Unexpected GGA from GPS: %s (%s)", str(fields), str(e))
        elif sentence.startswith("GPRMC"):
            fields = sentence.split(',')
            try:
                if fields[2] == "A":
                    data_container.gps_ground_speed = float(fields[7])
                    if len(fields[8]) > 0:
                        data_container.gps_ground_track = float(fields[8])
                    else:
                        data_container.gps_ground_track = 0
                    logger.log(2, "NMEA got RMC: speed=%d, track=%d",
                        data_container.gps_ground_speed,
                        data_container.gps_ground_track)
                    data_container.HaveNewGroundTrack = True
                    ret = True
                    data_container.RMCUpdate = True
                else:
                    data_container.gps_ground_speed = None
                    data_container.gps_ground_track = None
                    data_container.HaveNewGroundTrack = False
                    logger.debug("NMEA got void RMC")
            except Exception as e:
                logger.debug("Unexpected RMC from GPS: %s (%s)", str(fields), str(e))
    return ret

def make_unix_time (utc_hh, utc_mm, utc_ss):
    gmtime = time.gmtime()
    gmtime = (gmtime.tm_year, gmtime.tm_mon, gmtime.tm_mday,
            gmtime.tm_hour, gmtime.tm_min, 0,
            gmtime.tm_wday, gmtime.tm_yday, gmtime.tm_isdst)
    ret = timegm (gmtime)
    epoch = time.time()
    if epoch - ret > TWELVE_HOURS:
        # oops, skipped a day. Adjust.
        ret -= 2 * TWELVE_HOURS
    elif ret - epoch > TWELVE_HOURS:
        ret += 2 * TWELVE_HOURS
    ret += utc_ss
    return ret

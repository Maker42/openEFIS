
import re, logging

logger=logging.getLogger(__name__)

re_uint = r"(\d+)"
re_float = r"([\-]?\d+\.?\d*)"
re_hex = r"([\da-fA-F]+)"

re_gga = re.compile(r"\$GPGGA," +
        re_float + "," +                # Group 1:      UTC time
        re_float + ",(N|S)," +          # Group 2,3:    Latitude
        re_float + ",(E|W)," +          # Group 4,5:    Longitude
        re_uint + "," +                 # Group 6:      Fix quality
        re_uint + "," +                 # Group 7:      Number of SVs
        re_uint + "," +                 # Group 8:      HDOP
        re_float + ",(M|F)," +          # Group 9,10:   MSL height (meters or feet)
        r".*\*" + re_hex)               # Group 11:     Checksum

re_rmc = re.compile(r"\$GPRMC," +
        re_float + "," +                # Group 1:      UTC time
        r"(A|V)," +                     # Group 2:      Active or Void
        re_float + ",(N|S)," +          # Group 3,4:    Latitude
        re_float + ",(E|W)," +          # Group 5,6:    Longitude
        re_float + "," +                # Group 7:      Speed in Knots
        re_float + "," +                # Group 8:      Track Angle in degrees (true)
        re_uint + "," +                 # Group 9:      Date
        re_float + "," +                # Group 10:     Magnetic Variation in degrees
        r"\*" + re_hex)                 # Group 11:     Checksum

def CheckNMEAChecksum(checksum, s):
    for i in range(2,len(s)):
        checksum ^= ord(s[i])
    return False if checksum != 0 else True

METERS_FOOT = 0.3048
FEET_METER = 1.0 / METERS_FOOT

def ParseNMEAStrings(nmea, data_container):
    ret = False
    if nmea.startswith("$G") and nmea.find('*') > 0:
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
                data_container.signal_quality = int(fields[6])
                if data_container.signal_quality > 0:
                    data_container.utc = float(fields[1])
                    lat = float(fields[2]) / 100.0
                    if fields[3] == 'S':
                        lat *= -1.0
                    data_container.latitude = lat
                    lng = float(fields[4]) / 100.0
                    if fields[5] == 'W':
                        lng *= -1.0
                    data_container.longitude = lng
                    alt = float(fields[9])
                    if fields[10] == 'M':
                        alt *= FEET_METER
                    data_container.gps_altitude = alt
                    logger.log(2, "NMEA got GGA: time = %g, pos=%g,%g, alt=%g",
                            data_container.utc,
                            data_container.latitude,
                            data_container.longitude,
                            data_container.gps_altitude)
                    ret = True
                else:
                    logger.debug("NMEA got GGA with no signal quality")
            except:
                logger.debug("Unexpected GGA from GPS: %s", str(fields))
        elif sentence.startswith("GPRMC"):
            fields = sentence.split(',')
            try:
                if fields[2] == "A":
                    data_container.ground_speed = float(fields[7])
                    data_container.ground_track = float(fields[8])
                    if len(fields) > 10 and len(fields[10]) > 0:
                        data_container.magnetic_variation = float(fields[10])
                    else:
                        # TODO: Fill in magnetic variation with user input or big lookup table
                        data_container.magnetic_variation = 0.0
                    logger.log(2, "NMEA got RMC: speed=%g, track=%g, var=%g",
                        data_container.ground_speed,
                        data_container.ground_track,
                        data_container.magnetic_variation)
                    ret = True
                else:
                    logger.debug("NMEA got void RMC")
            except:
                logger.debug("Unexpected RMC from GPS: %s", str(fields))
    return ret

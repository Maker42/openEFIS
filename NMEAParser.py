
import re, logging

logger=logging.getLogger(__name__)

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
                data_container.SignalQuality = int(fields[6])
                if data_container.SignalQuality > 0:
                    data_container.Utc = float(fields[1])
                    lat = float(fields[2]) / 100.0
                    if fields[3] == 'S':
                        lat *= -1.0
                    data_container.Latitude = lat
                    lng = float(fields[4]) / 100.0
                    if fields[5] == 'W':
                        lng *= -1.0
                    data_container.Longitude = lng
                    alt = float(fields[9])
                    if fields[10] == 'M':
                        alt *= FEET_METER
                    data_container.GpsAltitude = alt
                    logger.log(2, "NMEA got GGA: time = %g, pos=%g,%g, alt=%g",
                            data_container.Utc,
                            data_container.Latitude,
                            data_container.Longitude,
                            data_container.GpsAltitude)
                    ret = True
                    data_container.HaveNewPosition = True
                else:
                    data_container.Utc = None
                    data_container.Latitude = None
                    data_container.Longitude = None
                    data_container.GpsAltitude = None
                    data_container.SignalQuality = 0
                    logger.debug("NMEA got GGA with no signal quality")
            except:
                logger.debug("Unexpected GGA from GPS: %s", str(fields))
            data_container.GGAUpdate = True
        elif sentence.startswith("GPRMC"):
            fields = sentence.split(',')
            try:
                if fields[2] == "A":
                    data_container.GroundSpeed = float(fields[7])
                    data_container.GroundTrack = float(fields[8])
                    if len(fields) > 10 and len(fields[10]) > 0:
                        data_container.MagneticVariation = float(fields[10])
                    else:
                        # TODO: Fill in magnetic variation with user input or big lookup table
                        data_container.MagneticVariation = 0.0
                    logger.log(2, "NMEA got RMC: speed=%g, track=%g, var=%g",
                        data_container.GroundSpeed,
                        data_container.GroundTrack,
                        data_container.MagneticVariation)
                    data_container.HaveNewGroundTrack = True
                    ret = True
                else:
                    data_container.GroundSpeed = None
                    data_container.GroundTrack = None
                    data_container.MagneticVariation = None
                    logger.debug("NMEA got void RMC")
            except:
                logger.debug("Unexpected RMC from GPS: %s", str(fields))
            data_container.RMCUpdate = True
    return ret

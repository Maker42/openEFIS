#include <SoftwareSerial.h>
#include "Adafruit_BMP085_U.h"
#include "Adafruit_L3GD20.h"
#include "Adafruit_LSM303_U.h"

Adafruit_BMP085_Unified   bmpSensor(0);
Adafruit_L3GD20           gyroSensor;
Adafruit_LSM303_Accel_Unified   accSensor;
Adafruit_LSM303_Mag_Unified   magSensor;

bool bmpSensorEnabled = false;
bool gyroSensorEnabled = false;
bool accSensorEnabled = false;
bool magSensorEnabled = false;

const byte rxPin = 2;
const byte txPin = 4;

// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
// Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
// Can't fix position faster than 5 times a second!


#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html

#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22"
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C"
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"
#define LOCUS_OVERLAP 0
#define LOCUS_FULLSTOP 1

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"

// standby command & boot successful message
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36"  // Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D"

// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31"

// request for updates on antenna status 
#define PGCMD_ANTENNA "$PGCMD,33,1*6C" 
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D" 

// set up a new serial object
SoftwareSerial gpsSerial (rxPin, txPin);

char    line[2][256];
int     current_line = 0, last_line = 1, fillat = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, HIGH);
  gpsSerial.begin(9600);
  delay(100);
  gpsSerial.println("");
  gpsSerial.println("");
  gpsSerial.println("");
  gpsSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gpsSerial.println(PMTK_ENABLE_WAAS);
  gpsSerial.println(PMTK_SET_NMEA_UPDATE_1HZ);
  line[current_line][0] = 0;
  line[last_line][0] = 0;
  fillat = 0;
  Serial.begin(115200);
  bmpSensorEnabled = bmpSensor.begin();
  gyroSensorEnabled = gyroSensor.begin();
  accSensorEnabled = accSensor.begin();
  magSensorEnabled = magSensor.begin();
}

void loop() {
  float   pressure, temperature;
  sensors_event_t   event;
  
  // put your main code here, to run repeatedly:
  while (gpsSerial.available())
  {
    char c = gpsSerial.read();
    line[current_line][fillat++] = c;
    //Serial.print ("line[");
    //Serial.print (current_line);
    //Serial.print ("][");
    //Serial.print (fillat-1);
    //Serial.print ("] = ");
    //Serial.println ((int)c);
    if ((c == '\n') || (fillat >= sizeof(line[0])-1))
    {
      line[current_line][fillat++] = 0;
      int   temp = last_line;
      last_line = current_line;
      current_line = temp;
      fillat = 0;
      Serial.write(line[last_line]);
    }
  }

  static char   attitudeString[256];
  int           attitudeIndex = sprintf(attitudeString, "$S10DOF,");
  // Get 10DOF readings
  if (bmpSensorEnabled)
  {
    bmpSensor.getPressure(&pressure);
    bmpSensor.getTemperature(&temperature);
    attitudeIndex += sprintf (attitudeString+attitudeIndex, "%f,%f,", pressure, temperature);
  }
  else
  {
    attitudeIndex += sprintf (attitudeString+attitudeIndex, ",,");
  }
  if (gyroSensorEnabled)
  {
    gyroSensor.read();
    attitudeIndex += sprintf (attitudeString+attitudeIndex, "%f,%f,%f",
                                gyroSensor.data.x,    // TODO: Assign proper x,y,z to pitch, roll, yaw.
                                gyroSensor.data.y,
                                gyroSensor.data.z);
  }
  else
  {
    attitudeIndex += sprintf (attitudeString+attitudeIndex, ",,,");
  }
  if (accSensorEnabled)
  {
    accSensor.getEvent(&event);
    attitudeIndex += sprintf (attitudeString+attitudeIndex, "%f,%f,%f",
                                event.acceleration.x,
                                event.acceleration.y,
                                event.acceleration.z);
  }
  else
  {
    attitudeIndex += sprintf (attitudeString+attitudeIndex, ",,,");
  }
  if (magSensorEnabled)
  {
    accSensor.getEvent(&event);
    attitudeIndex += sprintf (attitudeString+attitudeIndex, "%f,%f,%f",
                                event.magnetic.x,
                                event.magnetic.y,
                                event.magnetic.z);    
  }
  else
  {
    attitudeIndex += sprintf (attitudeString+attitudeIndex, ",,,");
    event.timestamp = millis();
  }
  attitudeIndex += sprintf (attitudeString+attitudeIndex, "%lu\n", event.timestamp);
  Serial.write(attitudeString);
  delay (10);
}

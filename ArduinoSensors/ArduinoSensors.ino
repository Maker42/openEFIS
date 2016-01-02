#include <SoftwareSerial.h>
#include <CmdMessenger.h>
#include "Adafruit_BMP085_U.h"
#include "Adafruit_L3GD20.h"
#include "Adafruit_LSM303_U.h"
#include "CheckSum.h"
#include "CmdLog.h"

// TODO: Make sure cmdMessenger doesn't block for sending commands

Adafruit_BMP085_Unified   bmpSensor(0);
Adafruit_L3GD20           gyroSensor;
Adafruit_LSM303_Accel_Unified   accSensor;
Adafruit_LSM303_Mag_Unified   magSensor;

bool bmpSensorEnabled = false;
bool gyroSensorEnabled = false;
bool accSensorEnabled = false;
bool magSensorEnabled = false;

const byte gpsRxPin = 2;
const byte gpsTxPin = 4;

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

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"

#define CMD_ANALOG_SET_CHANNEL  1
#define CMD_DIGITAL_SET_CHANNEL 2
#define CMD_10DOF_SENSOR_DATA   3
#define CMD_GPS_DATA            4
#define CMD_SET_THROTTLES       6

#define MAX_INDEPENDENT_THROTTLES 6
#define MAX_PWM_VALUE           255

CmdMessenger cmdMessenger (Serial, '^', ';', '\\');

// set up a new serial object
//SoftwareSerial gpsSerial (gpsRxPin, gpsTxPin);

char    gps_line[2][80];
int     current_gps_line = 0, last_gps_line = 1, gps_index = 0;

#define NACHANS  6
int     analog_pin_mapping[NACHANS] = {2, 3, 4, 5, 6, 7};
#define NDCHANS  2
int     digital_pin_mapping[NDCHANS] = {25, 26};

int     throttle_pin_mapping[MAX_INDEPENDENT_THROTTLES] = {8, 9, 10, 11, 12, 13};

char    output_line[80];

void SetThrottles()
{
  int16_t   nthrottles, val[MAX_INDEPENDENT_THROTTLES], chksum = -1;
  bool      valid = true;
  int       i;

  nthrottles = cmdMessenger.readInt16Arg();
  if ((nthrottles <= 0) || (nthrottles > MAX_INDEPENDENT_THROTTLES)) valid = false;
  else
  {
    for (i = 0;i < nthrottles; i++)
    {
      val[i] = cmdMessenger.readInt16Arg();
      if ((val[i] < 0) || (val[i] > MAX_PWM_VALUE)) valid = false;
    }
  }
  chksum = cmdMessenger.readInt16Arg();
  CheckSumStart(chksum);
  CheckSumDigest (chksum, sizeof(nthrottles), &nthrottles);
  for (i = 0; i < nthrottles; i++)
  {
      CheckSumDigest(chksum, sizeof (val[0]), val + i);
  }
  if (!CheckSumIsValid(chksum))     valid = false;

  if (valid)
  {
    for (i = 0; i < nthrottles; i++)
    {
        analogWrite(throttle_pin_mapping[i], val[i]);
    }
  }
  if (!valid)
  {
    sprintf (output_line, "Invalid Throttle: %d, %d, %d", nthrottles, val[0], chksum);
    cmdLog (cmdMessenger, 30, output_line);
  }
}

void AnalogSetChannel()
{
  int16_t   chan = -1, val = -1, chksum = -1;
  bool      valid = true;

  chan = cmdMessenger.readInt16Arg();
  val = cmdMessenger.readInt16Arg();
  chksum = cmdMessenger.readInt16Arg();
  CheckSumStart(chksum);
  CheckSumDigest(chksum, sizeof (chan), &chan);
  CheckSumDigest(chksum, sizeof (val), &val);
  if (!CheckSumIsValid(chksum))     valid = false;

  if (valid && (chan >= 0) && (val >= 0) && (val <= MAX_PWM_VALUE))
  {
      if (chan < NACHANS)
      {
        analogWrite(analog_pin_mapping[chan], val);
        sprintf (output_line, "ASet[%d] = %d", chan, val);
        cmdLog (cmdMessenger, 1, output_line);
        valid = true;
      } else valid = false;
  } else valid = false;
  if (!valid)
  {
    sprintf (output_line, "Invalid ASet: %d, %d, %d", chan, val, chksum);
    cmdLog (cmdMessenger, 30, output_line);
  }
}

void DigitalSetChannel()
{
  int16_t   chan, val, chksum;
  bool      valid = true;

  chan = cmdMessenger.readInt16Arg();
  val = cmdMessenger.readInt16Arg();
  chksum = cmdMessenger.readInt16Arg();
  CheckSumStart(chksum);
  CheckSumDigest(chksum, sizeof (chan), &chan);
  CheckSumDigest(chksum, sizeof (val), &val);
  if (!CheckSumIsValid(chksum))     valid = false;

  if (valid && (chan >= 0) && (val >= 0) && (val < 256))
  {
      if (chan < NDCHANS)
      {
        digitalWrite(digital_pin_mapping[chan], val == 0 ? 0 : 1);
        sprintf (output_line, "DSet[%d] = %d", chan, val);
        cmdLog (cmdMessenger, 1, output_line);
        valid = true;
      } else valid = false;
  } else valid = false;
  if (!valid)
  {
    cmdLog (cmdMessenger, 30, "Invalid Digital Cmd");
  }
}

void BadMsgHandler()
{
  cmdLog (cmdMessenger, 30, "Invalid message ID received");
}

void setup() {
  // put your setup code here, to run once:
  pinMode(gpsRxPin, INPUT);
  pinMode(gpsTxPin, OUTPUT);
  digitalWrite(gpsTxPin, HIGH);
//  gpsSerial.begin(9600);
//  delay(100);
//  gpsSerial.println("");
//  gpsSerial.println("");
//  gpsSerial.println("");
//  //gpsSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//  gpsSerial.println(PMTK_ENABLE_WAAS);
  //gpsSerial.println(PMTK_SET_NMEA_UPDATE_1HZ);
  gps_line[current_gps_line][0] = 0;
  gps_line[last_gps_line][0] = 0;
  gps_index = 0;
  Serial.begin(115200);
  bmpSensorEnabled = bmpSensor.begin();
  gyroSensorEnabled = gyroSensor.begin();
  accSensorEnabled = accSensor.begin();
  magSensorEnabled = magSensor.begin();

  for (int i = 0; i < NACHANS; i++)
  {
    pinMode(analog_pin_mapping[i], OUTPUT);
    analogWrite (analog_pin_mapping[i], 128);
  }
  for (int i = 0; i < NDCHANS; i++)
  {
    pinMode(digital_pin_mapping[i], OUTPUT);
    digitalWrite(digital_pin_mapping[i], 0);
  }

  cmdMessenger.attach(CMD_ANALOG_SET_CHANNEL, AnalogSetChannel);
  cmdMessenger.attach(CMD_DIGITAL_SET_CHANNEL, DigitalSetChannel);
  cmdMessenger.attach(BadMsgHandler);
  cmdMessenger.printLfCr();
}

void loop() {
  float   pressure, temperature;
  sensors_event_t   event;
  
  // put your main code here, to run repeatedly:
#if 0
  while (gpsSerial.available())
  {
    char c = gpsSerial.read();
    gps_line[current_gps_line][gps_index++] = c;
    //Serial.print ("gps_line[");
    //Serial.print (current_gps_line);
    //Serial.print ("][");
    //Serial.print (gps_index-1);
    //Serial.print ("] = ");
    //Serial.println ((int)c);
    if ((c == '\n') || (gps_index >= sizeof(gps_line[0])-1))
    {
      gps_line[current_gps_line][gps_index++] = 0;
      int   temp = last_gps_line;
      last_gps_line = current_gps_line;
      current_gps_line = temp;
      gps_index = 0;
      cmdMessenger.sendCmd(CMD_GPS_DATA, gps_line[last_gps_line]);
    }
  }
#endif

  cmdMessenger.feedinSerialData();

  int           attitudeIndex = 0;
  // Get 10DOF readings
  if (bmpSensorEnabled)
  {
    bmpSensor.getPressure(&pressure);
    bmpSensor.getTemperature(&temperature);
    attitudeIndex += sprintf (output_line+attitudeIndex, "%f,%f", pressure, temperature);
  }
  else
  {
    attitudeIndex += sprintf (output_line+attitudeIndex, ",");
  }
  if (gyroSensorEnabled)
  {
    gyroSensor.read();
    attitudeIndex += sprintf (output_line+attitudeIndex, ",%f,%f,%f",
                                gyroSensor.data.x,    // TODO: Assign proper x,y,z to pitch, roll, yaw.
                                gyroSensor.data.y,
                                gyroSensor.data.z);
  }
  else
  {
    attitudeIndex += sprintf (output_line+attitudeIndex, ",,,");
  }
  if (accSensorEnabled)
  {
    accSensor.getEvent(&event);
    attitudeIndex += sprintf (output_line+attitudeIndex, ",%f,%f,%f",
                                event.acceleration.x,
                                event.acceleration.y,
                                event.acceleration.z);
  }
  else
  {
    attitudeIndex += sprintf (output_line+attitudeIndex, ",,,");
  }
  if (magSensorEnabled)
  {
    accSensor.getEvent(&event);
    attitudeIndex += sprintf (output_line+attitudeIndex, ",%f,%f,%f",
                                event.magnetic.x,
                                event.magnetic.y,
                                event.magnetic.z);    
  }
  else
  {
    attitudeIndex += sprintf (output_line+attitudeIndex, ",,,");
    event.timestamp = millis();
  }
  attitudeIndex += sprintf (output_line+attitudeIndex, ",%lu", event.timestamp);
  int16_t   chksum = 0;
  CheckSumStart(chksum);
  CheckSumDigest (chksum, attitudeIndex, output_line);
  cmdMessenger.sendCmdStart (CMD_10DOF_SENSOR_DATA);
  cmdMessenger.sendCmdEscArg(output_line);
  cmdMessenger.sendCmdArg(chksum);
  cmdMessenger.sendCmdEnd();
  delay (10);
}

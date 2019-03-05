// Copyright (C) 2015-2019  Garrett Herschleb
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>

#include <EEPROM.h>
#include <avr/wdt.h>

#include "Adafruit_BMP085_U.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_L3GD20.h"
#include "Adafruit_LSM303_U.h"
#include "MPU9250.h"

#include "llc.h"

// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger (Serial, '^', ';', '/');

Adafruit_BMP085_Unified   bmpSensor(0);
Adafruit_BMP280           bmp280Sensor;
Adafruit_L3GD20           gyroSensor;
Adafruit_LSM303_Accel_Unified   accSensor;
Adafruit_LSM303_Mag_Unified   magSensor;
Stream *gps = NULL;                    // Serial data stream

bool bmpSensorEnabled = false;
bool bmp280SensorEnabled = false;
bool gyroSensorEnabled = false;
bool accSensorEnabled = false;
bool magSensorEnabled = false;
bool mpu9250Enabled = false;

char    gps_line[2][120];
int     current_gps_line = 0, last_gps_line = 1, gps_index = 0;

uint8_t function2chan[num_functions];

char    output_line[120];

sChannel    channels[MAX_CHANNELS];

void Onsetup_digital_sensor()
{
  int16_t   chan, pin, pullup;
  int32_t   period;
  chan = cmdMessenger.readInt16Arg();
  if ((chan >= (int16_t)NELEMENTS(channels)) || (chan < 0))
  {
    cmdMessenger.sendCmd(nack, "Digital Invalid channel");
  } else
  {
    pin = cmdMessenger.readInt16Arg();
    period = cmdMessenger.readInt32Arg();
    pullup = cmdMessenger.readInt16Arg();
    channels[chan].pin = pin;
    channels[chan].period = period;
    channels[chan].function = 'd';
    channels[chan].next_time = millis() + period;
    if (pullup)
    {
        pinMode (pin, INPUT_PULLUP);
    } else
    {
        pinMode (pin, INPUT);
    }
    cmdMessenger.sendCmd(ack);
  }
}

void Onsetup_analog_sensor()
{
  int16_t   chan, pin;
  int32_t   period;
  chan = cmdMessenger.readInt16Arg();
  if ((chan >= (int16_t)NELEMENTS(channels)) || (chan < 0))
  {
    cmdMessenger.sendCmd(nack, "Analog Invalid channel");
  } else
  {
    pin = cmdMessenger.readInt16Arg();
    period = cmdMessenger.readInt32Arg();
    channels[chan].function = 'n';
    channels[chan].pin = pin;
    channels[chan].period = period;
    channels[chan].filter_coefficient[0] = cmdMessenger.readFloatArg();
    channels[chan].filter_coefficient[1] = cmdMessenger.readFloatArg();
    channels[chan].rejection_band = cmdMessenger.readFloatArg();
    channels[chan].secondary_band = cmdMessenger.readFloatArg();
    channels[chan].secondary_filter_duration = (unsigned)cmdMessenger.readInt32Arg();

    channels[chan].secondary_filter_count = 0;
    channels[chan].sample_count = 0;
    channels[chan].reject_count = 0;
    channels[chan].secondary_use_count = 0;
    channels[chan].next_time = millis() + period;
    channels[chan].state[0] = analogRead(pin);
    cmdMessenger.sendCmd(ack);
  }
}

bool initialize_i2c (int16_t chan)
{
  bool      good = true, fail, mag_data_valid;
  float     pressure, temperature;
  sensors_event_t   event;
  int       retry;

  channels[chan].secondary_filter_count = 0;
  channels[chan].sample_count = 0;
  channels[chan].reject_count = 0;
  channels[chan].secondary_use_count = 0;
  switch (channels[chan].function)
  {
    case 'a':
      if (accSensorEnabled)
      {
          for (retry = 10; retry != 0; retry--)
          {
              if (accSensor.getEvent(&event)) break;
              delayMicroseconds(500);
              accSensor.begin();
          }
          if (retry > 0)
          {
              channels[chan].state[0] = event.orientation.x;
              channels[chan].state[1] = event.orientation.y;
              channels[chan].state[2] = event.orientation.z;
              function2chan[accel_function] = chan;
              cmdLog (30, "acc channel ready");
          } else
          {
            cmdLog (99, "Failed to initialize acc");
          }
      } else if (mpu9250Enabled)
      {
          float     ax, ay, az, gx, gy, gz, mx, my, mz;
          for (retry = 10; retry != 0; retry--)
          {
              if (MPU9250_read9dof (ax, ay, az, gx, gy, gz, mx, my, mz, mag_data_valid, fail)) break;
              delayMicroseconds(500);
              MPU9250_begin(GFS_250DPS, AFS_4G);
          }
          if (retry > 0)
          {
              channels[chan].state[0] = ax;
              channels[chan].state[1] = ay;
              channels[chan].state[2] = az;
              function2chan[accel_function] = chan;
              cmdLog (30, "9250 acc channel ready");
          } else
          {
            cmdLog (99, "Failed to initialize acc");
          }
      } else good = false;
      break;
    case 'r':
      if (gyroSensorEnabled)
      {
          for (retry = 10; retry != 0; retry--)
          {
              if (gyroSensor.read()) break;
              delayMicroseconds(500);
              gyroSensor.begin();
          }
          if (retry > 0)
          {
              channels[chan].state[0] = gyroSensor.data.x;
              channels[chan].state[1] = gyroSensor.data.y;
              channels[chan].state[2] = gyroSensor.data.z;
              function2chan[rotation_function] = chan;
              cmdLog (30, "gyro channel ready");
          } else
          {
            cmdLog (99, "Failed to initialize gyro");
          }
      } else if (mpu9250Enabled)
      {
          float     ax, ay, az, gx, gy, gz, mx, my, mz;
          for (retry = 10; retry != 0; retry--)
          {
              if (MPU9250_read9dof (ax, ay, az, gx, gy, gz, mx, my, mz, mag_data_valid, fail)) break;
              delayMicroseconds(500);
              MPU9250_begin(GFS_250DPS, AFS_4G);
          }
          if (retry > 0)
          {
              channels[chan].state[0] = gx;
              channels[chan].state[1] = gy;
              channels[chan].state[2] = gz;
              function2chan[rotation_function] = chan;
              cmdLog (30, "9250 gyro channel ready");
          } else
          {
            cmdLog (99, "Failed to initialize acc");
          }
      } else good = false;
      break;
    case 'm':
      if (magSensorEnabled)
      {
          for (retry = 10; retry != 0; retry--)
          {
              if (magSensor.getEvent(&event)) break;
              delayMicroseconds(500);
              magSensor.begin();
          }
          if (retry > 0)
          {
              channels[chan].state[0] = event.magnetic.x;
              channels[chan].state[1] = event.magnetic.y;
              channels[chan].state[2] = event.magnetic.z;
              function2chan[magnetic_function] = chan;
              cmdLog (30, "mag channel ready");
          } else
          {
            cmdLog (99, "Failed to initialize mag");
          }
      } else if (mpu9250Enabled)
      {
          float     ax, ay, az, gx, gy, gz, mx, my, mz;
          for (retry = 10; retry != 0; retry--)
          {
              if (MPU9250_read9dof (ax, ay, az, gx, gy, gz, mx, my, mz, mag_data_valid, fail) &&
                        mag_data_valid) break;
              delayMicroseconds(500);
              MPU9250_begin(GFS_250DPS, AFS_4G);
          }
          if (retry > 0)
          {
              channels[chan].state[0] = mx;
              channels[chan].state[1] = my;
              channels[chan].state[2] = mz;
              function2chan[magnetic_function] = chan;
              cmdLog (30, "9250 mag channel ready");
          } else
          {
            cmdLog (99, "Failed to initialize acc");
          }
      } else good = false;
      break;
    case 'p':
      if (bmpSensorEnabled || bmp280SensorEnabled)
      {
          for (retry = 10; retry != 0; retry--)
          {
              if (bmpSensorEnabled && bmpSensor.getPressure(&pressure)) break;
              if (bmp280SensorEnabled && bmp280Sensor.readPressure(&pressure)) break;
              delayMicroseconds(500);
              if (bmpSensorEnabled) bmpSensor.begin();
              else                  bmp280Sensor.begin();
          }
          if (retry > 0)
          {
              channels[chan].state[0] = pressure;
              function2chan[pressure_function] = chan;
              cmdLog (30, "pressure channel ready");
          } else
          {
            cmdLog (99, "Failed to initialize pressure");
          }
      } else good = false;
      break;
    case 't':
      if (bmpSensorEnabled || bmp280SensorEnabled)
      {
          for (retry = 10; retry != 0; retry--)
          {
              if (bmpSensorEnabled && bmpSensor.getTemperature(&temperature)) break;
              if (bmp280SensorEnabled && bmp280Sensor.readTemperature(&temperature)) break;
              delayMicroseconds(500);
              if (bmpSensorEnabled) bmpSensor.begin();
              else                  bmp280Sensor.begin();
          }
          if (retry > 0)
          {
              channels[chan].state[0] = temperature;
              function2chan[temp_function] = chan;
              cmdLog (30, "temperature channel ready");
          } else
          {
            cmdLog (99, "Failed to initialize temp");
          }
      } else good = false;
      break;
    default:
        good = false;
  }
  if (good) channels[chan].next_time = millis() + channels[chan].period;
  return good;
}


void Onsetup_i2c_sensor()
{
  int16_t   chan;

  if (output_line[0] != 0)
  {
    cmdLog (50, output_line);
    output_line[0] = 0;
  }
  chan = cmdMessenger.readInt16Arg();
  if ((chan >= (int16_t)NELEMENTS(channels)) || (chan < 0))
  {
    cmdMessenger.sendCmd(nack, "I2C Invalid channel");
  } else
  {
      bool good = true;
      channels[chan].function = cmdMessenger.readCharArg();
      channels[chan].period = cmdMessenger.readInt32Arg();
      channels[chan].filter_coefficient[0] = cmdMessenger.readFloatArg();
      channels[chan].filter_coefficient[1] = cmdMessenger.readFloatArg();
      channels[chan].secondary_band = cmdMessenger.readFloatArg();
      channels[chan].rejection_band = cmdMessenger.readFloatArg();
      channels[chan].secondary_filter_duration = (unsigned)cmdMessenger.readInt32Arg();
      good = initialize_i2c (chan);
      if (good)
      {
          cmdMessenger.sendCmd(ack);
          //wdt_enable(WDTO_500MS);
      } else cmdMessenger.sendCmd(nack, "Unavailable I2C function");
  }
}

void Onsetup_spi_sensor()
{
    cmdMessenger.sendCmd(nack, "SPI sensors unimplemented");
}

void Onsetup_serial_sensor()
{
  int16_t   chan;
  char      *stype;

  chan = cmdMessenger.readInt16Arg();
  if ((chan >= (int16_t)NELEMENTS(channels)) || (chan < 0))
  {
    cmdMessenger.sendCmd(nack, "Serial Invalid channel");
  } else
  {
      bool good = true;
      channels[chan].function = cmdMessenger.readCharArg();
      channels[chan].pin = cmdMessenger.readInt16Arg();     // Actually port #
      stype = cmdMessenger.readStringArg();
      strncpy (channels[chan].stype, stype, sizeof(channels[chan].stype));
      good = initialize_serial (chan);
      if (good) cmdMessenger.sendCmd(ack);
  }
}

bool initialize_serial (int16_t chan)
{
  bool      good = true;
  switch (channels[chan].function)
  {
    case 'g':
      function2chan[gps_function] = chan;
      break;
    default:
        good = false;
        cmdMessenger.sendCmd(nack, "Invalid Serial funciton");
        break;
  }
  if (good)
  {
    switch (channels[chan].pin)
    {
      case 1:
        gps = &Serial1;
        Serial1.begin(9600);
        if (!strncmp (channels[chan].stype, "ubx", 3))
        {
            gps->println("$PUBX,41,1,0003,0003,57600,0*2F");
            Serial1.end();
            Serial1.begin(57600);
        }
        break;
      case 2:
        gps = &Serial2;
        Serial2.begin(9600);
        if (!strncmp (channels[chan].stype, "ubx", 3))
        {
            gps->println("$PUBX,41,1,0003,0003,57600,0*2F");
            Serial2.end();
            Serial2.begin(57600);
        }
        break;
      case 3:
        gps = &Serial3;
        Serial3.begin(9600);
        if (!strncmp (channels[chan].stype, "ubx", 3))
        {
            gps->println("$PUBX,41,1,0003,0003,57600,0*2F");
            Serial3.end();
            Serial3.begin(57600);
        }
        break;
      default:
        good = false;
        cmdMessenger.sendCmd(nack, "Invalid Serial Port Number");
    }
    //gps->write (PMTK_SET_BAUD_9600);
    delay(100);
    if (!strncmp (channels[chan].stype, "ubx", 3))
    {
        gps->println ("$PUBX,40,GLL,0,0,0,0,0,0*5C");
        gps->println ("$PUBX,40,GSA,0,0,0,0,0,0*4E");
        gps->println ("$PUBX,40,GSV,0,0,0,0,0,0*59");
        gps->println ("$PUBX,40,VTG,0,0,0,0,0,0*5E");
        gps->println ("$PUBX,40,RMC,0,1,0,0,0,0*46");
    } else if (!strncmp (channels[chan].stype, "mtk", 3))
    {
        gps->write (PMTK_SET_NMEA_OUTPUT_RMCGGA);
        gps->write (PMTK_SET_NMEA_UPDATE_1HZ);
        gps->write (PMTK_API_SET_FIX_CTL_1HZ);
    }
  }
  return good;
}

void Onsetup_analog_output()
{
  int16_t   pin, value;
  pin = cmdMessenger.readInt16Arg();
  value = cmdMessenger.readInt16Arg();
  pinMode (pin, OUTPUT);
  analogWrite (pin, value);
}

void Onsetup_digital_output()
{
  int16_t   pin, value;
  pin = cmdMessenger.readInt16Arg();
  value = cmdMessenger.readInt16Arg();
  pinMode (pin, OUTPUT);
  digitalWrite (pin, value);
}

void Onset_analog_output()
{
  int16_t   pin, value;
  pin = cmdMessenger.readInt16Arg();
  value = cmdMessenger.readInt16Arg();
  analogWrite (pin, value);
}

void Onset_digital_output()
{
  int16_t   pin, value;
  pin = cmdMessenger.readInt16Arg();
  value = cmdMessenger.readInt16Arg();
  digitalWrite (pin, value);
}

void restore_from_eeprom()
{
    int         addr, i, j;
    uint8_t    *data;
    int16_t     nchans;

    addr = sizeof(uint32_t);
    EEPROM.get(addr, nchans);
    addr += sizeof (nchans);
    for (i = 0; i < nchans; i++)
    {
        data = (uint8_t*) &(channels[i]);
        for (j = 0; j < CHANNEL_CONFIGURATION_SIZE; j++)
        {
            *data++ = EEPROM.read (addr++);
        }
        switch (channels[i].function)
        {
            case 'a':
            case 'r':
            case 'm':
            case 'p':
            case 't':
                sprintf (output_line, "Restoring channel %d as i2c function 0x%02x", i, channels[i].function);
                cmdLog (99, output_line);
                initialize_i2c(i);
                break;
            case 'g':
                sprintf (output_line, "Restoring channel %d as GPS function 0x%02x", i, channels[i].function);
                cmdLog (99, output_line);
                initialize_serial(i);
                break;
            default:
                sprintf (output_line, "Invalid channel %d function 0x%02x from EEPROM",
                        i, channels[i].function);
                cmdLog (99, output_line);
                break;
        }
    }
}

const uint32_t    magic_number = 0xfeedface;

void Onsave_configuration()
{
  int16_t   nchans;

  nchans = cmdMessenger.readInt16Arg();
  if ((nchans > (int16_t)NELEMENTS(channels)) || (nchans < 0))
  {
    cmdMessenger.sendCmd(nack, "Invalid number of channels");
  } else
  {
    int       addr, i, j;
    uint8_t  *data;
    addr = 0;
    EEPROM.put (addr, magic_number);
    addr += sizeof (magic_number);
    EEPROM.put (addr, nchans);
    addr += sizeof (nchans);
    for (i = 0; i < nchans; i++)
    {
        data = (uint8_t*) &(channels[i]);
        for (j = 0; j < CHANNEL_CONFIGURATION_SIZE; j++)
        {
            EEPROM.update (addr, *data++);
            addr++;
        }
    }
    cmdMessenger.sendCmd(ack);
    cmdLog(99, "Configuration Saved");
  }
}

void check_eeprom()
{
    uint32_t    eemagic_number;
    EEPROM.get (0, eemagic_number);
    if (magic_number == eemagic_number)
    {
        restore_from_eeprom();
    } else
    {
        sprintf (output_line, "Invalid EEPROM data: 0x%08lx", eemagic_number);
        cmdLog(99, output_line);
    }
}

void OnUnknownCommand()
{
    cmdMessenger.sendCmd(nack, "Unknown Command");
}

// Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(setup_digital_sensor, Onsetup_digital_sensor);
  cmdMessenger.attach(setup_analog_sensor, Onsetup_analog_sensor);
  cmdMessenger.attach(setup_i2c_sensor, Onsetup_i2c_sensor);
  cmdMessenger.attach(setup_spi_sensor, Onsetup_spi_sensor);
  cmdMessenger.attach(setup_serial_sensor, Onsetup_serial_sensor);
  cmdMessenger.attach(setup_analog_output, Onsetup_analog_output);
  cmdMessenger.attach(setup_digital_output, Onsetup_digital_output);
  cmdMessenger.attach(set_analog_output, Onset_analog_output);
  cmdMessenger.attach(set_digital_output, Onset_digital_output);
  cmdMessenger.attach(save_configuration, Onsave_configuration);
  cmdMessenger.printLfCr(false); 
}


void cmdLog(unsigned level, const char *s)
{
  cmdMessenger.sendCmdStart(send_log);
  cmdMessenger.sendCmdArg (level);
  cmdMessenger.sendCmdEscArg (s);
  cmdMessenger.sendCmdEnd();
}

void pollGps()
{
  static int     max_gps_available = 0;     // Collect buffer fill statistics in case it's needed

  while (gps && (gps->available()))
  {
    if (gps->available() > max_gps_available) max_gps_available = gps->available();
    char c = gps->read();
    if (c == '\r') continue;
    gps_line[current_gps_line][gps_index++] = c;
    
    if ((c == '\n') || (gps_index >= (int)sizeof(gps_line[0])-1) || ((c == '$') && (gps_index > 1)))
    {
      int   temp = last_gps_line;
      if (c == '$')
      {
        gps_line[current_gps_line][gps_index-1] = 0;
        gps_line[last_gps_line][0] = '$';
        gps_index = 1;
      } else
      {
        gps_line[current_gps_line][gps_index] = 0;
        gps_index = 0;
      }
      last_gps_line = current_gps_line;
      current_gps_line = temp;
      cmdMessenger.sendCmdStart (sensor_reading);
      cmdMessenger.sendCmdArg (function2chan [gps_function]);
      cmdMessenger.sendCmdArg (gps_line[last_gps_line]);
      cmdMessenger.sendCmdArg (millis());
      cmdMessenger.sendCmdEnd ();
      max_gps_available = 0;
    }
  }
}

void filter_channel (pChannel pch, int ninputs, float input[])
{
    int     in, j, filter = 0;
    float   diff[3], adiff;
    bool    trigger_secondary = false;

    for (in = 0; in < ninputs; in++)
    {
        diff[in] = input[in] - pch->state[in];
        adiff = ABS(diff[in]);
        if (adiff > pch->rejection_band)
        {   // Reject all inputs from this sample. They are all suspect
            pch->reject_count++;
            if ((pch->reject_count > 9) && (pch->secondary_filter_count + pch->sample_count < pch->reject_count))
            {
              // Reset filter state
              for (j = 0; j < ninputs; j++) pch->state[j] = input[j];
              pch->sample_count = 0;
              pch->reject_count = 0;
              pch->secondary_filter_count = 0;
            }
            return;
        }
        if (adiff > pch->secondary_band)
        {   trigger_secondary = true; }
    }
    if (trigger_secondary)                  // If we get more large sensor movement,
    {   pch->secondary_filter_count = 1; }  // reset the filter count for more responsiveness
    if (pch->secondary_filter_count > 0)
    {
        filter = 1;
        pch->secondary_use_count++;
    }
    for (in = 0; in < ninputs; in++)
    {
        pch->state[in] += pch->filter_coefficient[filter] * diff[in];
    }
    if (pch->secondary_filter_count > 0)
    {
        pch->secondary_filter_count++;
        if (pch->secondary_filter_count >= pch->secondary_filter_duration)
        {   pch->secondary_filter_count = 0;    }
    }
    pch->sample_count++;
}

void setup()
{
  // put your setup code here, to run once:
  memset (channels, 0, sizeof (channels));
  memset (function2chan, 0xff, sizeof (function2chan));
  gps_line[current_gps_line][0] = 0;
  gps_line[last_gps_line][0] = 0;
  gps_index = 0;
  Serial.begin(115200);
  cmdLog (99, "Sensors Starting...");
  output_line[0] = 0;
  bmpSensorEnabled = bmpSensor.begin();
  if (output_line[0] != 0)
  {
    cmdLog (50, output_line);
    output_line[0] = 0;
  }
  if (bmpSensorEnabled) cmdLog (99, "BMP085 available");
  gyroSensorEnabled = gyroSensor.begin();
  if (output_line[0] != 0)
  {
    cmdLog (50, output_line);
    output_line[0] = 0;
  }
  if (gyroSensorEnabled) cmdLog (99, "gyro available");
  accSensorEnabled = accSensor.begin();
  if (output_line[0] != 0)
  {
    cmdLog (50, output_line);
    output_line[0] = 0;
  }
  if (accSensorEnabled) cmdLog (99, "acc available");
  magSensorEnabled = magSensor.begin();
  if (output_line[0] != 0)
  {
    cmdLog (50, output_line);
    output_line[0] = 0;
  }
  if (magSensorEnabled) cmdLog (99, "mag available");
  cmdLog (99, "MPU9250 begin...");
  mpu9250Enabled = MPU9250_begin();
  if (output_line[0] != 0)
  {
    cmdLog (50, output_line);
    output_line[0] = 0;
  }
  if (mpu9250Enabled) cmdLog (99, "MPU9250 available");
  else cmdLog (99, "No 9250 available");
  cmdLog (99, "Check BMP280...");
  bmp280SensorEnabled = bmp280Sensor.begin();
  if (output_line[0] != 0)
  {
    cmdLog (50, output_line);
    output_line[0] = 0;
  }
  if (bmp280SensorEnabled) cmdLog (99, "BMP280 available");
  else cmdLog(99, "BMP280 not available");
  attachCommandCallbacks();
  //check_eeprom();
}

void loop()
{
  float   pressure, temperature;
  sensors_event_t   event;
  pChannel          pch;
  bool              valid, mpu9250_valid = false, mpu9250_fail, mag_data_valid;
  float             ax, ay, az, gx, gy, gz, mx, my, mz;
  
  // put your main code here, to run repeatedly:
  cmdMessenger.feedinSerialData();
  pollGps();

  unsigned long ms = millis();
  long          timediff;
  int           channel;
  // Get 10DOF readings
  if (function2chan[pressure_function] != 0xff)
  {
    channel = function2chan[pressure_function];
    pch = &(channels[channel]);
    if (bmpSensorEnabled) valid = bmpSensor.getPressure(&pressure);
    else                  valid = bmp280Sensor.readPressure(&pressure);
    if (valid)
    {
        filter_channel (pch, 1, &pressure);
        timediff = (long)pch->next_time - (long)ms;
        if (timediff <= 0)
        {
            cmdMessenger.sendCmdStart (sensor_reading);
            cmdMessenger.sendCmdArg (channel);
            cmdMessenger.sendCmdArg (pch->state[0]);
            cmdMessenger.sendCmdArg (ms);
            cmdMessenger.sendCmdArg (pch->sample_count);
            cmdMessenger.sendCmdArg (pch->secondary_use_count);
            cmdMessenger.sendCmdArg (pch->reject_count);
            cmdMessenger.sendCmdEnd ();
            pch->next_time += pch->period;
            pch->sample_count = 0;
            pch->reject_count = 0;
            pch->secondary_use_count = 0;
            //wdt_reset();
        }
    } else
    {
        if (bmpSensorEnabled) bmpSensor.begin();
        else                  bmp280Sensor.begin();
        cmdLog (99, "BMP Sensor reset");
    }
  }

  pollGps();
  ms = millis();

  if (function2chan[temp_function] != 0xff)
  {
    channel = function2chan[temp_function];
    pch = &(channels[channel]);
    if (bmpSensorEnabled) valid = bmpSensor.getTemperature(&temperature);
    else                  valid = bmp280Sensor.readTemperature(&temperature);
    if (valid)
    {
        timediff = (long)pch->next_time - (long)ms;
        filter_channel (pch, 1, &temperature);
        if (timediff <= 0)
        {
            //cmdLog (99, "T");
            cmdMessenger.sendCmdStart (sensor_reading);
            cmdMessenger.sendCmdArg (channel);
            cmdMessenger.sendCmdArg (pch->state[0]);
            cmdMessenger.sendCmdArg (pch->sample_count);
            cmdMessenger.sendCmdArg (pch->secondary_use_count);
            cmdMessenger.sendCmdArg (pch->reject_count);
            cmdMessenger.sendCmdEnd ();
            pch->next_time += pch->period;
            pch->sample_count = 0;
            pch->reject_count = 0;
            pch->secondary_use_count = 0;
            //wdt_reset();
        }
    } else
    {
        if (bmpSensorEnabled) bmpSensor.begin();
        else                  bmp280Sensor.begin();
        cmdLog (99, "BMP Sensor reset");
    }
  }

  pollGps();
  ms = millis();

  if (mpu9250Enabled)
  {
      mpu9250_valid = MPU9250_read9dof (ax, ay, az, gx, gy, gz, mx, my, mz, mag_data_valid, mpu9250_fail);
  }

  if (function2chan[rotation_function] != 0xff)
  {
    channel = function2chan[rotation_function];
    pch = &(channels[channel]);
    if (gyroSensorEnabled) valid = gyroSensor.read();
    else                   valid = mpu9250_valid;
    if (valid)
    {
        if (mpu9250_valid)
        {
            gyroSensor.data.x = gx;
            gyroSensor.data.y = gy;
            gyroSensor.data.z = gz;
        }
        filter_channel (pch, 3, &(gyroSensor.data.x));
        timediff = (long)pch->next_time - (long)ms;
        if (timediff <= 0)
        {
            //cmdLog (99, "g");
            cmdMessenger.sendCmdStart (sensor_reading);
            cmdMessenger.sendCmdArg (channel);
            cmdMessenger.sendCmdArg (pch->state[0]);
            cmdMessenger.sendCmdArg (pch->state[1]);
            cmdMessenger.sendCmdArg (pch->state[2]);
            cmdMessenger.sendCmdArg (ms);
            cmdMessenger.sendCmdArg (pch->sample_count);
            cmdMessenger.sendCmdArg (pch->secondary_use_count);
            cmdMessenger.sendCmdArg (pch->reject_count);
            cmdMessenger.sendCmdEnd ();
            pch->next_time += pch->period;
            pch->sample_count = 0;
            pch->reject_count = 0;
            pch->secondary_use_count = 0;
            //wdt_reset();
        }
    }
  }

  pollGps();
  ms = millis();

  if (function2chan[accel_function] != 0xff)
  {
    channel = function2chan[accel_function];
    pch = &(channels[channel]);
    if (accSensorEnabled) valid = accSensor.getEvent(&event);
    else                   valid = mpu9250_valid;
    if (valid)
    {
        //cmdLog (99, "A");
        if (mpu9250_valid)
        {
            event.orientation.x = ax;
            event.orientation.y = ay;
            event.orientation.z = az;
        }
        filter_channel (pch, 3, event.orientation.v);
        timediff = (long)pch->next_time - (long)ms;
        if (timediff <= 0)
        {
            cmdMessenger.sendCmdStart (sensor_reading);
            cmdMessenger.sendCmdArg (channel);
            cmdMessenger.sendCmdArg (pch->state[0]);
            cmdMessenger.sendCmdArg (pch->state[1]);
            cmdMessenger.sendCmdArg (pch->state[2]);
            cmdMessenger.sendCmdArg (event.timestamp);
            cmdMessenger.sendCmdArg (pch->sample_count);
            cmdMessenger.sendCmdArg (pch->secondary_use_count);
            cmdMessenger.sendCmdArg (pch->reject_count);
            cmdMessenger.sendCmdEnd ();
            pch->next_time += pch->period;
            pch->sample_count = 0;
            pch->reject_count = 0;
            pch->secondary_use_count = 0;
            //wdt_reset();
        }
    } else
    {
        cmdLog (99, "Accel Sensor not valid read");
    }
  }

  pollGps();
  ms = millis();

  if (function2chan[magnetic_function] != 0xff)
  {
    channel = function2chan[magnetic_function];
    pch = &(channels[channel]);
    if (magSensorEnabled) valid = magSensor.getEvent(&event);
    else                  valid = (mpu9250_valid && mag_data_valid);
    if (valid)
    {
        if (mpu9250_valid)
        {
            event.magnetic.x = mx;
            event.magnetic.y = my;
            event.magnetic.z = mz;
        }
        filter_channel (pch, 3, event.magnetic.v);
        timediff = (long)pch->next_time - (long)ms;
        if (timediff <= 0)
        {
            //cmdLog (99, "M");
            cmdMessenger.sendCmdStart (sensor_reading);
            cmdMessenger.sendCmdArg (channel);
            cmdMessenger.sendCmdArg (pch->state[0]);
            cmdMessenger.sendCmdArg (pch->state[1]);
            cmdMessenger.sendCmdArg (pch->state[2]);
            cmdMessenger.sendCmdArg (event.timestamp);
            cmdMessenger.sendCmdArg (pch->sample_count);
            cmdMessenger.sendCmdArg (pch->secondary_use_count);
            cmdMessenger.sendCmdArg (pch->reject_count);
            cmdMessenger.sendCmdEnd ();
            pch->next_time += pch->period;
            pch->sample_count = 0;
            pch->reject_count = 0;
            pch->secondary_use_count = 0;
            //wdt_reset();
        }
    } else
    {
        if (magSensorEnabled) magSensor.begin();
        else if (mpu9250Enabled && mpu9250_fail)
        {
            cmdLog(99, "MPU9250 reset");
            MPU9250_begin(GFS_250DPS, AFS_4G);
        }
    }
  }

  pollGps();

  for (channel = 0; channel < (int)NELEMENTS(channels); channel++)
  {
    pch = &(channels[channel]);
    if ((pch->function == 'd') || (pch->function == 'n'))
    {
        ms = millis();
        if (pch->function == 'n')
        {
            float       a = analogRead(pch->pin);
            filter_channel (pch, 1, &a);
        }
        timediff = (long)pch->next_time - (long)ms;
        if (timediff <= 0)
        {
            cmdMessenger.sendCmdStart (sensor_reading);
            cmdMessenger.sendCmdArg (channel);
            if (pch->function == 'd')
            {
                cmdMessenger.sendCmdArg (digitalRead(pch->pin));
            } else
            {
                cmdMessenger.sendCmdArg (pch->state[0]);
            }
            cmdMessenger.sendCmdArg (ms);
            cmdMessenger.sendCmdArg (pch->sample_count);
            cmdMessenger.sendCmdArg (pch->secondary_use_count);
            cmdMessenger.sendCmdArg (pch->reject_count);
            cmdMessenger.sendCmdEnd ();
            
            pch->next_time += pch->period;
            pch->sample_count = 0;
            pch->reject_count = 0;
            pch->secondary_use_count = 0;
        }
    }
  }
  //if (sensors_configured >= 5) cmdLog (99, "L");
}

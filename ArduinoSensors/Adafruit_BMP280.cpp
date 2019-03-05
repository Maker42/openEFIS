/***************************************************************************
  This is a library for the BMP280 pressure sensor
  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651
  These sensors use I2C to communicate, 2 pins are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "Arduino.h"
#include <SPI.h>
#include "Adafruit_BMP280.h"

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

extern void cmdLog(unsigned level, const char *s);
extern char output_line[];

Adafruit_BMP280::Adafruit_BMP280()
  : _cs(-1), _mosi(-1), _miso(-1), _sck(-1)
{ }

Adafruit_BMP280::Adafruit_BMP280(int8_t cspin)
  : _cs(cspin), _mosi(-1), _miso(-1), _sck(-1)
{ }

Adafruit_BMP280::Adafruit_BMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
  : _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin)
{ }


bool Adafruit_BMP280::begin(uint8_t a, uint8_t chipid)
{
  uint8_t       val = 0;
  _i2caddr = a;

  if (_cs == -1) {
    // i2c
#ifdef CORE_TEENSY
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);
    Wire.setDefaultTimeout(1000);
#else
    Wire.begin();
#endif
  } else {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);

    if (_sck == -1) {
      // hardware SPI
      SPI.begin();
    } else {
      // software SPI
      pinMode(_sck, OUTPUT);
      pinMode(_mosi, OUTPUT);
      pinMode(_miso, INPUT);
    }
  }

  if ((!read8(BMP280_REGISTER_CHIPID, &val)) || (val != chipid))
  {
      if (val == 0)
      {
        sprintf (output_line, "BMP280: No response");
      } else
      {
        sprintf (output_line, "Invalid BMP280 ID: 0x%02x. Should be 0x%02x",
                val, chipid);
      }
    return false;
  }

  if (!readCoefficients())
  {
      sprintf (output_line, "BMP280: Failure to read coefficients");
      return false;
  }
  write8(BMP280_REGISTER_CONTROL, 0x3F);
  return true;
}

uint8_t Adafruit_BMP280::spixfer(uint8_t x) {
  if (_sck == -1)
    return SPI.transfer(x);

  // software spi
  //Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, x & (1<<i));
    digitalWrite(_sck, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
void Adafruit_BMP280::write8(byte reg, byte value)
{
  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg & ~0x80); // write, bit 7 low
    spixfer(value);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C/SPI
*/
/**************************************************************************/
bool Adafruit_BMP280::read8(byte reg, uint8_t *value)
{

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    if (Wire.requestFrom((uint8_t)_i2caddr, (byte)1)
            < 1) return false;
    *value = Wire.read();
  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    *value = spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C/SPI
*/
/**************************************************************************/
bool Adafruit_BMP280::read16(byte reg, uint16_t *value)
{

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    if (Wire.requestFrom((uint8_t)_i2caddr, (byte)2)
            < 2) return false;
    *value = (Wire.read() << 8) | Wire.read();

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    *value = (spixfer(0) << 8) | spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }

  return true;
}

bool Adafruit_BMP280::read16_LE(byte reg, uint16_t *value) {
  uint16_t temp;
  if (!read16(reg, &temp)) return false;
  *value = (temp >> 8) | (temp << 8);
  return true;
}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C/SPI
*/
/**************************************************************************/
bool Adafruit_BMP280::readS16(byte reg, int16_t *value)
{
  return read16(reg, (uint16_t*)value);

}

bool Adafruit_BMP280::readS16_LE(byte reg, int16_t *val)
{
  return read16_LE(reg, (uint16_t*)val);

}


/**************************************************************************/
/*!
    @brief  Reads a 24 bit value over I2C/SPI
*/
/**************************************************************************/
bool Adafruit_BMP280::read24(byte reg, uint32_t *value)
{

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    if (Wire.requestFrom((uint8_t)_i2caddr, (byte)3)
            < 3) return false;
    
    *value = Wire.read();
    *value <<= 8;
    *value |= Wire.read();
    *value <<= 8;
    *value |= Wire.read();

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    
    *value = spixfer(0);
    *value <<= 8;
    *value |= spixfer(0);
    *value <<= 8;
    *value |= spixfer(0);

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
bool Adafruit_BMP280::readCoefficients(void)
{
    if (!read16_LE(BMP280_REGISTER_DIG_T1, &_bmp280_calib.dig_T1)) return false;
    if (!readS16_LE(BMP280_REGISTER_DIG_T2, &_bmp280_calib.dig_T2)) return false;
    if (!readS16_LE(BMP280_REGISTER_DIG_T3, &_bmp280_calib.dig_T3)) return false;

    if (!read16_LE(BMP280_REGISTER_DIG_P1, &_bmp280_calib.dig_P1)) return false;
    if (!readS16_LE(BMP280_REGISTER_DIG_P2, &_bmp280_calib.dig_P2)) return false;
    if (!readS16_LE(BMP280_REGISTER_DIG_P3, &_bmp280_calib.dig_P3)) return false;
    if (!readS16_LE(BMP280_REGISTER_DIG_P4, &_bmp280_calib.dig_P4)) return false;
    if (!readS16_LE(BMP280_REGISTER_DIG_P5, &_bmp280_calib.dig_P5)) return false;
    if (!readS16_LE(BMP280_REGISTER_DIG_P6, &_bmp280_calib.dig_P6)) return false;
    if (!readS16_LE(BMP280_REGISTER_DIG_P7, &_bmp280_calib.dig_P7)) return false;
    if (!readS16_LE(BMP280_REGISTER_DIG_P8, &_bmp280_calib.dig_P8)) return false;
    if (!readS16_LE(BMP280_REGISTER_DIG_P9, &_bmp280_calib.dig_P9)) return false;
    return true;
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
bool Adafruit_BMP280::readTemperature(float *temperature)
{
    static unsigned print_count = 10;
  int32_t var1, var2;

  int32_t adc_T;
  if (!read24(BMP280_REGISTER_TEMPDATA, (uint32_t*)(&adc_T))) return false;
  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
	   ((int32_t)_bmp280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
	     ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
	   ((int32_t)_bmp280_calib.dig_T3)) >> 14;

  t_fine = var1 + var2;

  float T  = (t_fine * 5 + 128) >> 8;
  *temperature = T/100;
  if (print_count > 0)
  {
    if (print_count == 10)
    {
        sprintf (output_line, "dig_t[] = %u,%d,%d"
                                ,_bmp280_calib.dig_T1
                                ,_bmp280_calib.dig_T2
                                ,_bmp280_calib.dig_T3);
        cmdLog(99, output_line);
    }
    print_count--;
    sprintf (output_line, "temp: %ld,%ld,%ld,%ld ==> %g"
                            ,adc_T
                            ,var1
                            ,var2, t_fine, *temperature);
    cmdLog(99, output_line);
  }

  return true;
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
bool Adafruit_BMP280::readPressure(float *pressure)
{
  static unsigned print_count = 10;
  int64_t var1, var2, p;
  float t;

  // Must be done first to get the t_fine variable set up
  if (!readTemperature(&t)) return false;

  int32_t adc_P;
  if (!read24(BMP280_REGISTER_PRESSUREDATA, (uint32_t*)(&adc_P))) return false;
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

  if (print_count > 0)
  {
      if (print_count == 10)
      {
        sprintf (output_line, "dig_p[] = %u,%d,%d,%d,%d,%d,%d,%d,%d"
                                ,_bmp280_calib.dig_P1
                                ,_bmp280_calib.dig_P2
                                ,_bmp280_calib.dig_P3
                                ,_bmp280_calib.dig_P4
                                ,_bmp280_calib.dig_P5
                                ,_bmp280_calib.dig_P6
                                ,_bmp280_calib.dig_P7
                                ,_bmp280_calib.dig_P8
                                ,_bmp280_calib.dig_P9
                                );
        cmdLog(99, output_line);
      }
        sprintf (output_line, "pressure1: %ld,%lld,%lld,%ld"
                                ,adc_P
                                ,var1
                                ,var2, t_fine);
        cmdLog(99, output_line);
  }
  if (var1 == 0) {
    return false;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  if (print_count > 0)
  {
        sprintf (output_line, "pressure2: %lld,%lld,%lld"
                                ,p
                                ,var1
                                ,var2);
        cmdLog(99, output_line);
  }
  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
  *pressure = (float)p/256;
  if (print_count > 0)
  {
      print_count--;
        sprintf (output_line, "pressure3: %lld ==> %g"
                                ,p
                                ,*pressure);
        cmdLog(99, output_line);
  }
  return true;
}

bool Adafruit_BMP280::readAltitude(float *altitude, float seaLevelhPa) {

  float pressure;
  if (!readPressure(&pressure)) return false; // in Si units for Pascal
  pressure /= 100;

  *altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return true;
}

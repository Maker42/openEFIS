# Copyright (C) 2019  Garrett Herschleb
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


BMP280_ADDRESS = 0x76
BMP280_CHIPID = 0x58
BMP280_REGISTER_DIG_T1              = 0x88
BMP280_REGISTER_DIG_T2              = 0x8A
BMP280_REGISTER_DIG_T3              = 0x8C

BMP280_REGISTER_DIG_P1              = 0x8E
BMP280_REGISTER_DIG_P2              = 0x90
BMP280_REGISTER_DIG_P3              = 0x92
BMP280_REGISTER_DIG_P4              = 0x94
BMP280_REGISTER_DIG_P5              = 0x96
BMP280_REGISTER_DIG_P6              = 0x98
BMP280_REGISTER_DIG_P7              = 0x9A
BMP280_REGISTER_DIG_P8              = 0x9C
BMP280_REGISTER_DIG_P9              = 0x9E

BMP280_REGISTER_CHIPID             = 0xD0
BMP280_REGISTER_VERSION            = 0xD1
BMP280_REGISTER_SOFTRESET          = 0xE0

BMP280_REGISTER_CAL26              = 0xE1  # R calibration stored in 0xE1-0xF0

BMP280_REGISTER_CONTROL            = 0xF4
BMP280_REGISTER_CONFIG             = 0xF5
BMP280_REGISTER_PRESSUREDATA       = 0xF7
BMP280_REGISTER_TEMPDATA           = 0xFA

class BMP280:
    def __init__(self, ado=0):
        self.address = BMP280_ADDRESS if ado==0 else BMP280_ADDRESS+1
        self.print_count = 10

    def begin(self, bus):
        self.bus = bus
        _id = self.bus.read_byte_data(self.address, BMP280_REGISTER_CHIPID)
        if _id != BMP280_CHIPID:
            print ("BMP280 not found at address 0x%02x"%self.address)
            return False

        print ("BMP280 found")

        # Read coefficients
        self.dig_T = [0] * 4
        self.dig_T[1] = self.read16_LE(BMP280_REGISTER_DIG_T1)
        self.dig_T[2] = self.readS16_LE(BMP280_REGISTER_DIG_T2)
        self.dig_T[3] = self.readS16_LE(BMP280_REGISTER_DIG_T3)
        print ("dig_t = %s"%str(self.dig_T))

        self.dig_P = [0] * 10
        self.dig_P[1] = self.read16_LE(BMP280_REGISTER_DIG_P1)
        self.dig_P[2] = self.readS16_LE(BMP280_REGISTER_DIG_P2)
        self.dig_P[3] = self.readS16_LE(BMP280_REGISTER_DIG_P3)
        self.dig_P[4] = self.readS16_LE(BMP280_REGISTER_DIG_P4)
        self.dig_P[5] = self.readS16_LE(BMP280_REGISTER_DIG_P5)
        self.dig_P[6] = self.readS16_LE(BMP280_REGISTER_DIG_P6)
        self.dig_P[7] = self.readS16_LE(BMP280_REGISTER_DIG_P7)
        self.dig_P[8] = self.readS16_LE(BMP280_REGISTER_DIG_P8)
        self.dig_P[9] = self.readS16_LE(BMP280_REGISTER_DIG_P9)

        self.bus.write_byte_data(self.address, BMP280_REGISTER_CONTROL, 0x3f)
        return True

    def readS16_LE(self, reg):
        b = self.bus.read_i2c_block_data(self.address, reg, 2)
        ret = b[1] << 8 | b[0]
        # Sign extend the result from bit 15
        if ret & 0x8000:
            temp = -1
            temp &= (~0xffff)
            temp |= ret
            ret = temp
        return ret

    def read16_LE(self, reg):
        b = self.bus.read_i2c_block_data(self.address, reg, 2)
        ret = b[1] << 8 | b[0]
        return ret

    def read24(self, reg):
        b = self.bus.read_i2c_block_data(self.address, reg, 3)
        ret = b[0]
        ret <<= 8
        ret |= b[1]
        ret <<= 8
        ret |= b[2]
        return ret

    def _tfine(self):
        adc_T = self.read24(BMP280_REGISTER_TEMPDATA)
        adc_T >>= 4

        var1  = ((((adc_T>>3) - (self.dig_T[1] <<1))) *
                 (self.dig_T[2])) >> 11

        var2  = (((((adc_T>>4) - (self.dig_T[1])) *
                   ((adc_T>>4) - (self.dig_T[1]))) >> 12) *
                  (self.dig_T[3])) >> 14
        if self.print_count > 0:
            print ("temp: %d,%d,%d"%(adc_T, var1, var2))

        return var1 + var2

    def readTemperature(self):
        t_fine = self._tfine()
        T  = float((t_fine * 5 + 128) >> 8)
        if self.print_count > 0:
            print ("temperature(t_fine %d) = %g"%(t_fine, T/100))
            self.print_count -= 1
        return [T/100]

    def readPressure(self):
        t_fine = self._tfine()
        adc_P = self.read24(BMP280_REGISTER_PRESSUREDATA)
        adc_P >>= 4

        var1 = (t_fine) - 128000
        var2 = var1 * var1 * self.dig_P[6]
        var2 = var2 + ((var1*self.dig_P[5])<<17)
        var2 = var2 + ((self.dig_P[4])<<35)
        var1 = ((var1 * var1 * self.dig_P[3])>>8) + \
               ((var1 * self.dig_P[2])<<12)
        var1 = ((((1)<<47)+var1))*(self.dig_P[1])>>33

        if (var1 == 0):
            return None
        p = 1048576 - adc_P
        p = int((((p<<31) - var2)*3125) / var1)
        var1 = ((self.dig_P[9]) * (p>>13) * (p>>13)) >> 25
        var2 = ((self.dig_P[8]) * p) >> 19

        p = ((p + var1 + var2) >> 8) + ((self.dig_P[7])<<4)
        if self.print_count > 0:
            print ("pressure3: %d,%d,%d ==> %g"%(p, var1, var2,(float(p)/256)))
        return [float(p)/256]

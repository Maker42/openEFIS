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


import time

AK8963_WHO_AM_I =0x00 # should return 0x48
AK8963_INFO =    0x01
AK8963_ST1 =     0x02  # data ready status bit 0
AK8963_XOUT_L   =0x03  # data
AK8963_XOUT_H   =0x04
AK8963_YOUT_L   =0x05
AK8963_YOUT_H   =0x06
AK8963_ZOUT_L   =0x07
AK8963_ZOUT_H   =0x08
AK8963_ST2 =     0x09  # Data overflow bit 3 and data read error status bit 2
AK8963_CNTL =    0x0A  # Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
AK8963_ASTC =    0x0C  # Self test control
AK8963_I2CDIS =  0x0F  # I2C disable
AK8963_ASAX =    0x10  # Fuse ROM x-axis sensitivity adjustment value
AK8963_ASAY =    0x11  # Fuse ROM y-axis sensitivity adjustment value
AK8963_ASAZ =    0x12  # Fuse ROM z-axis sensitivity adjustment value

SELF_TEST_X_GYRO = 0x00
SELF_TEST_Y_GYRO = 0x01
SELF_TEST_Z_GYRO = 0x02

SELF_TEST_X_ACCEL = 0x0D
SELF_TEST_Y_ACCEL = 0x0E
SELF_TEST_Z_ACCEL = 0x0F

SELF_TEST_A =    0x10

XG_OFFSET_H =    0x13  # User-defined trim values for gyroscope
XG_OFFSET_L =    0x14
YG_OFFSET_H =    0x15
YG_OFFSET_L =    0x16
ZG_OFFSET_H =    0x17
ZG_OFFSET_L =    0x18
SMPLRT_DIV =     0x19
CONFIG =         0x1A
GYRO_CONFIG =    0x1B
ACCEL_CONFIG =   0x1C
ACCEL_CONFIG2 =  0x1D
LP_ACCEL_ODR =   0x1E
WOM_THR =        0x1F

MOT_DUR =        0x20  # Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
ZMOT_THR =       0x21  # Zero-motion detection threshold bits [7:0]
ZRMOT_DUR =      0x22  # Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

FIFO_EN =        0x23
I2C_MST_CTRL =   0x24
I2C_SLV0_ADDR =  0x25
I2C_SLV0_REG =   0x26
I2C_SLV0_CTRL =  0x27
I2C_SLV1_ADDR =  0x28
I2C_SLV1_REG =   0x29
I2C_SLV1_CTRL =  0x2A
I2C_SLV2_ADDR =  0x2B
I2C_SLV2_REG =   0x2C
I2C_SLV2_CTRL =  0x2D
I2C_SLV3_ADDR =  0x2E
I2C_SLV3_REG =   0x2F
I2C_SLV3_CTRL =  0x30
I2C_SLV4_ADDR =  0x31
I2C_SLV4_REG =   0x32
I2C_SLV4_DO =    0x33
I2C_SLV4_CTRL =  0x34
I2C_SLV4_DI =    0x35
I2C_MST_STATUS = 0x36
INT_PIN_CFG =    0x37
INT_ENABLE =     0x38
DMP_INT_STATUS = 0x39  # Check DMP interrupt
INT_STATUS =     0x3A
ACCEL_XOUT_H =   0x3B
ACCEL_XOUT_L =   0x3C
ACCEL_YOUT_H =   0x3D
ACCEL_YOUT_L =   0x3E
ACCEL_ZOUT_H =   0x3F
ACCEL_ZOUT_L =   0x40
TEMP_OUT_H =     0x41
TEMP_OUT_L =     0x42
GYRO_XOUT_H =    0x43
GYRO_XOUT_L =    0x44
GYRO_YOUT_H =    0x45
GYRO_YOUT_L =    0x46
GYRO_ZOUT_H =    0x47
GYRO_ZOUT_L =    0x48
EXT_SENS_DATA_00 = 0x49
EXT_SENS_DATA_01 = 0x4A
EXT_SENS_DATA_02 = 0x4B
EXT_SENS_DATA_03 = 0x4C
EXT_SENS_DATA_04 = 0x4D
EXT_SENS_DATA_05 = 0x4E
EXT_SENS_DATA_06 = 0x4F
EXT_SENS_DATA_07 = 0x50
EXT_SENS_DATA_08 = 0x51
EXT_SENS_DATA_09 = 0x52
EXT_SENS_DATA_10 = 0x53
EXT_SENS_DATA_11 = 0x54
EXT_SENS_DATA_12 = 0x55
EXT_SENS_DATA_13 = 0x56
EXT_SENS_DATA_14 = 0x57
EXT_SENS_DATA_15 = 0x58
EXT_SENS_DATA_16 = 0x59
EXT_SENS_DATA_17 = 0x5A
EXT_SENS_DATA_18 = 0x5B
EXT_SENS_DATA_19 = 0x5C
EXT_SENS_DATA_20 = 0x5D
EXT_SENS_DATA_21 = 0x5E
EXT_SENS_DATA_22 = 0x5F
EXT_SENS_DATA_23 = 0x60
MOT_DETECT_STATUS = 0x61
I2C_SLV0_DO =    0x63
I2C_SLV1_DO =    0x64
I2C_SLV2_DO =    0x65
I2C_SLV3_DO =    0x66
I2C_MST_DELAY_CTRL = 0x67
SIGNAL_PATH_RESET =0x68
MOT_DETECT_CTRL =0x69
USER_CTRL =      0x6A  # Bit 7 enable DMP, bit 3 reset DMP
PWR_MGMT_1 =     0x6B # Device defaults to the SLEEP mode
PWR_MGMT_2 =     0x6C
DMP_BANK =       0x6D  # Activates a specific bank in the DMP
DMP_RW_PNT =     0x6E  # Set read/write pointer to a specific start address in specified DMP bank
DMP_REG =        0x6F  # Register in DMP from which to read or to which to write
DMP_REG_1 =      0x70
DMP_REG_2 =      0x71
FIFO_COUNTH =    0x72
FIFO_COUNTL =    0x73
FIFO_R_W =       0x74
WHO_AM_I_MPU9250 = 0x75 # Should return 0x71
XA_OFFSET_H =    0x77
XA_OFFSET_L =    0x78
YA_OFFSET_H =    0x7A
YA_OFFSET_L =    0x7B
ZA_OFFSET_H =    0x7D
ZA_OFFSET_L =    0x7E

# Using the MSENSR-9250 breakout board, ADO is set to 0
# Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
MPU9250_ADDRESS =0x68  # Device address when ADO = 0
AK8963_ADDRESS =0x0C   #  Address of magnetometer

AFS_2G = 0
AFS_4G = 1
AFS_8G = 2
AFS_16G =3

GFS_250DPS = 0
GFS_500DPS = 1
GFS_1000DPS= 2
GFS_2000DPS= 3

MFS_14BITS = 0
MFS_16BITS = 1

class MPU9250:
    def __init__(self, gscale=GFS_500DPS, ascale=AFS_4G, mscale=MFS_16BITS, ado=0):
        self.gscale = gscale
        self.ascale = ascale
        self.mscale = mscale
        self.ado = ado
        self.bus = None
        if self.mscale == MFS_14BITS:
            self.mres = 10.*4912./8190. # Proper scale to return milliGauss
        elif self.mscale == MFS_16BITS:
            self.mres = 10.*4912./32760. # Proper scale to return milliGauss
        else:
            raise RuntimeError ("Unrecognized mscale")
        print ("mres=%g"%self.mres)

        if self.gscale == GFS_250DPS:
            self.gres = 250.0/32768.0;
        elif self.gscale == GFS_500DPS:
            self.gres = 500.0/32768.0;
        elif self.gscale == GFS_1000DPS:
            self.gres = 1000.0/32768.0;
        elif self.gscale == GFS_2000DPS:
            self.gres = 2000.0/32768.0;
        else:
            raise RuntimeError ("Unrecognized gscale")

        if self.ascale == AFS_2G:
            self.ares = 2.0/32768.0;
        elif self.ascale == AFS_4G:
            self.ares = 4.0/32768.0;
        elif self.ascale == AFS_8G:
            self.ares = 8.0/32768.0;
        elif self.ascale == AFS_16G:
            self.ares = 16.0/32768.0;
        else:
            raise RuntimeError ("Unrecognized ascale")
        self.accel_data = None
        self.gyro_data = None
        self.mag_data = None
        self.print_count = 10

    def begin(self, bus):
        self.bus = bus
        self.mpu_address = MPU9250_ADDRESS
        self.ak_address = AK8963_ADDRESS
        if self.ado:
            self.mpu_address += 1
            self.ak_address += 1
        _id = self.bus.read_byte_data(self.mpu_address, WHO_AM_I_MPU9250)
        if _id & (~0x2) != 0x71:
            print ("MPU9250 not found at address 0x%02x"%self.mpu_address)
            return False

        print ("MPU9250 found")
        # Initialize Gyro and Accelerometer
        self.bus.write_byte_data(self.mpu_address, PWR_MGMT_1, 0x00)
        time.sleep(.1)
        self.bus.write_byte_data(self.mpu_address, PWR_MGMT_1, 0x01)
        time.sleep(.2)

        self.bus.write_byte_data(self.mpu_address, CONFIG, 0x03)
        self.bus.write_byte_data(self.mpu_address, SMPLRT_DIV, 0x04)

        c = self.bus.read_byte_data(self.mpu_address, GYRO_CONFIG)
        c = c & (~0x03) # Clear Fchoice bits [1:0]
        c = c & (~0x18) # Clear GFS bits [4:3]
        c = c | (self.gscale << 3) # Set full scale range for the gyro
        self.bus.write_byte_data(self.mpu_address, GYRO_CONFIG, c)

        c = self.bus.read_byte_data(self.mpu_address, ACCEL_CONFIG)
        c = c & ~0x18       # Clear AFS bits [4:3]
        c = c | (self.ascale << 3)     # Set full scale range for the accelerometer
        self.bus.write_byte_data(self.mpu_address, ACCEL_CONFIG, c)

        c = self.bus.read_byte_data(self.mpu_address, ACCEL_CONFIG2)
        c = c & (~0x0f) # Clear Fchoice bits [1:0]
        c = c | 0x03    # Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        self.bus.write_byte_data(self.mpu_address, ACCEL_CONFIG2, c)

        self.bus.write_byte_data(self.mpu_address, INT_PIN_CFG, 0x22)
        self.bus.write_byte_data(self.mpu_address, INT_ENABLE, 0x01)

        # Init Magnetometer
        if self.ado:
            # Turn off bypass to isolate the second AK chip
            b = self.bus.read_byte_data(self.mpu_address, INT_PIN_CFG)
            b &= (~0x02)
            self.bus.write_byte_data(self.mpu_address, INT_PIN_CFG, b)
        else:
            d = self.bus.read_byte_data(self.ak_address, AK8963_WHO_AM_I)
            if d != 0x48:
                print ("AK8963 magnetometer not found at address 0x%02x"%self.ak_address)
                return False
            self.bus.write_byte_data(self.ak_address, AK8963_CNTL, 0)    # Power down
            time.sleep(.1)
            self.bus.write_byte_data(self.ak_address, AK8963_CNTL, 0x0f) # Fuse ROM access mode
            d = self.bus.read_i2c_block_data(self.ak_address, AK8963_ASAX, 3)
            self.mag_cal = [(float(x - 128)/256. + 1.) for x in d]
            print ("mag_cal = %s"%str(self.mag_cal))

            self.bus.write_byte_data(self.ak_address, AK8963_CNTL, 0)    # Power down
            time.sleep(.1)
            self.bus.write_byte_data(self.ak_address, AK8963_CNTL,
                            (self.mscale<<4) | 0x02)
            time.sleep(.1)

        return True

    def read9DOF(self):
        intstat = self.bus.read_byte_data(self.mpu_address, INT_STATUS)
        if intstat & 1:
            a = self.readAccelData()
            self.accel_data = [x*self.ares for x in a]
            g = self.readGyroData()
            self.gyro_data = [x*self.gres for x in g]
            if not self.ado:
                m = self.readMagData()
                if m is not None:
                    self.mag_data = [x*self.mres*c for x,c in zip(m, self.mag_cal)]

    def readAccel(self):
        if self.accel_data is None:
            self.read9DOF()
        ret = self.accel_data
        self.accel_data = None
        if self.print_count > 0:
            self.print_count -= 1
            print ("accel: %s"%str(ret))
        return ret

    def readGyro(self):
        if self.gyro_data is None:
            self.read9DOF()
        ret = self.gyro_data
        self.gyro_data = None
        if self.print_count > 0:
            print ("gyro: %s"%str(ret))
        return ret

    def readMagnetometer(self):
        if self.mag_data is None:
            self.read9DOF()
        ret = self.mag_data
        self.mag_data = None
        if self.print_count > 0:
            print ("mag: %s"%str(ret))
        return ret

    def readAccelData(self):
        d = self.bus.read_i2c_block_data (self.mpu_address, ACCEL_XOUT_H, 6)
        ret = list()
        ret.append ((d[0] << 8) | d[1])
        ret.append ((d[2] << 8) | d[3])
        ret.append ((d[4] << 8) | d[5])
        # Sign extend the result from bit 15
        for i in range(len(ret)):
            if ret[i] & 0x8000:
                temp = -1
                temp &= (~0xffff)
                temp |= ret[i]
                ret[i] = temp
        #print ("accel: %s"%str(ret))
        return ret

    def readGyroData(self):
        d = self.bus.read_i2c_block_data (self.mpu_address, GYRO_XOUT_H, 6)
        ret = list()
        ret.append ((d[0] << 8) | d[1])
        ret.append ((d[2] << 8) | d[3])
        ret.append ((d[4] << 8) | d[5])
        # Sign extend the result from bit 15
        for i in range(len(ret)):
            if ret[i] & 0x8000:
                temp = -1
                temp &= (~0xffff)
                temp |= ret[i]
                ret[i] = temp
        #print ("gyro: %s"%str(ret))
        return ret

    def readMagData(self):
        drdy = self.bus.read_byte_data(self.ak_address, AK8963_ST1)
        if (drdy & 1) != 0:
            d = self.bus.read_i2c_block_data (self.ak_address, AK8963_XOUT_L, 7)
            if (d[6] & 0x08) == 0:
                ret = list()
                ret.append ((d[1] << 8) | d[0])
                ret.append ((d[3] << 8) | d[2])
                ret.append ((d[5] << 8) | d[4])
                # Sign extend the result from bit 15
                for i in range(len(ret)):
                    if ret[i] & 0x8000:
                        temp = -1
                        temp &= (~0xffff)
                        temp |= ret[i]
                        ret[i] = temp
                #print ("mag: %s"%str(ret))
                return ret
        return None

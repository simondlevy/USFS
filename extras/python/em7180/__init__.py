'''
   Python classes for EM7180 SENtral Sensor

   Copyright (C) 2018 Simon D. Levy

   This file is part of EM7180.

   EM7180 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   EM7180 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EM7180.  If not, see <http:#www.gnu.org/licenses/>.
'''

import smbus
import time

class EM7180(object):

    def __init__(self):

        #EM7180 SENtral register map
        #see http:#www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf
        self.QX                 = 0x00  # this is a 32-bit normalized floating point number read from registers = 0x00-03
        self.QY                 = 0x04  # this is a 32-bit normalized floating point number read from registers = 0x04-07
        self.QZ                 = 0x08  # this is a 32-bit normalized floating point number read from registers = 0x08-0B
        self.QW                 = 0x0C  # this is a 32-bit normalized floating point number read from registers = 0x0C-0F
        self.QTIME              = 0x10  # this is a 16-bit unsigned integer read from registers = 0x10-11
        self.MX                 = 0x12  # int16_t from registers = 0x12-13
        self.MY                 = 0x14  # int16_t from registers = 0x14-15
        self.MZ                 = 0x16  # int16_t from registers = 0x16-17
        self.MTIME              = 0x18  # uint16_t from registers = 0x18-19
        self.AX                 = 0x1A  # int16_t from registers = 0x1A-1B
        self.AY                 = 0x1C  # int16_t from registers = 0x1C-1D
        self.AZ                 = 0x1E  # int16_t from registers = 0x1E-1F
        self.ATIME              = 0x20  # uint16_t from registers = 0x20-21
        self.GX                 = 0x22  # int16_t from registers = 0x22-23
        self.GY                 = 0x24  # int16_t from registers = 0x24-25
        self.GZ                 = 0x26  # int16_t from registers = 0x26-27
        self.GTIME              = 0x28  # uint16_t from registers = 0x28-29
        self.Baro               = 0x2A  # start of two-byte MS5637 pressure data, 16-bit signed interger
        self.BaroTIME           = 0x2C  # start of two-byte MS5637 pressure timestamp, 16-bit unsigned
        self.Temp               = 0x2E  # start of two-byte MS5637 temperature data, 16-bit signed interger
        self.TempTIME           = 0x30  # start of two-byte MS5637 temperature timestamp, 16-bit unsigned
        self.QRateDivisor       = 0x32  # uint8_t 
        self.EnableEvents       = 0x33
        self.HostControl        = 0x34
        self.EventStatus        = 0x35
        self.SensorStatus       = 0x36
        self.SentralStatus      = 0x37
        self.AlgorithmStatus    = 0x38
        self.FeatureFlags       = 0x39
        self.ParamAcknowledge   = 0x3A
        self.SavedParamByte0    = 0x3B
        self.SavedParamByte1    = 0x3C
        self.SavedParamByte2    = 0x3D
        self.SavedParamByte3    = 0x3E
        self.ActualMagRate      = 0x45
        self.ActualAccelRate    = 0x46
        self.ActualGyroRate     = 0x47
        self.ActualBaroRate     = 0x48
        self.ActualTempRate     = 0x49
        self.ErrorRegister      = 0x50
        self.AlgorithmControl   = 0x54
        self.MagRate            = 0x55
        self.AccelRate          = 0x56
        self.GyroRate           = 0x57
        self.BaroRate           = 0x58
        self.TempRate           = 0x59
        self.LoadParamByte0     = 0x60
        self.LoadParamByte1     = 0x61
        self.LoadParamByte2     = 0x62
        self.LoadParamByte3     = 0x63
        self.ParamRequest       = 0x64
        self.ROMVersion1        = 0x70
        self.ROMVersion2        = 0x71
        self.RAMVersion1        = 0x72
        self.RAMVersion2        = 0x73
        self.ProductID          = 0x90
        self.RevisionID         = 0x91
        self.RunStatus          = 0x92
        self.UploadAddress      = 0x94 # uint16_t registers = 0x94 (MSB)-5(LSB)
        self.UploadData         = 0x96  
        self.CRCHost            = 0x97 # uint32_t from registers = 0x97-9A
        self.ResetRequest       = 0x9B   
        self.PassThruStatus     = 0x9E   
        self.PassThruControl    = 0xA0
        self.ACC_LPF_BW         = 0x5B  #Register GP36
        self.GYRO_LPF_BW        = 0x5C  #Register GP37
        self.BARO_LPF_BW        = 0x5D  #Register GP38
        self.GP36               = 0x5B
        self.GP37               = 0x5C
        self.GP38               = 0x5D
        self.GP39               = 0x5E
        self.GP40               = 0x5F
        self.GP50               = 0x69
        self.GP51               = 0x6A
        self.GP52               = 0x6B
        self.GP53               = 0x6C
        self.GP54               = 0x6D
        self.GP55               = 0x6E
        self.GP56               = 0x6F

        self.ADDRESS            = 0x28   # Address of the EM7180 SENtral sensor hub

        self.bus = None
        self.errorStart = 0

    def begin(self, bus=1):

        self.bus = smbus.SMBus(bus)

        self.errorStatus = 0

        print(self.readRegister(self.SentralStatus))
        exit(0)


        # Check SENtral status, make sure EEPROM upload of firmware was accomplished
        for attempts in range(10):
            if (self.readRegister(self.SentralStatus) & 0x01):
                if (self.readRegister(self.SentralStatus) & 0x01): continue
                if (self.readRegister(self.SentralStatus) & 0x02): continue
                if (self.readRegister(self.SentralStatus) & 0x04):
                    self.errorStatus = 0xB0
                    return False
                
                if (self.readRegister(self.SentralStatus) & 0x08): continue
                if (self.readRegister(self.SentralStatus) & 0x10): 
                    self.errorStatus = 0xB0
                    return False
                break
            
            self.writeRegister(self.ResetRequest, 0x01)
            time.sleep(0.5)


        if (self.readRegister(self.SentralStatus) & 0x04):
            self.errorStatus = 0xB0
            return False

        return True

    def getErrorString(self):

        return ''

    def readRegister(self, subAddress):

        return self.readRegisters(subAddress, 1)[0]

    def readRegisters(self, subAddress, count):

        self.bus.write_byte(self.ADDRESS, subAddress)

        return [self.bus.read_byte(self.ADDRESS) for k in range(count)]

class EM7180_Master(object):

    def __init__(self, magRate, accelRate, gyroRate, baroRate, qRateDivisor):

        self.magRate = magRate
        self.accelRate = accelRate
        self.gyroRate = gyroRate 
        self.baroRate = baroRate
        self.qRateDivisor = qRateDivisor

        self.eventStatus = 0

        self.em7180 = EM7180()

    def begin(self, bus=1):

        # Fail immediately if unable to upload EEPROM
        if not self.em7180.begin(bus):
            return False

        return True

    def getErrorString(self):

        return self.em7180.getErrorString()

    def checkEventStatus(self):

        return

    def gotError(self):

        return False

    def gotQuaternion(self):

        return True

    def readQuaternion(self):

        return 0,0,0,0

    def gotAccelerometer(self):

        return True

    def readAccelerometer(self):

        return 0,0,0

    def gotGyrometer(self):

        return True

    def readGyrometer(self):

        return 0,0,0

    def gotMagnetometer(self):

        return True

    def readMagnetometer(self):

        return 0,0,0

    def gotBarometer(self):

        return True

    def readBarometer(self):

        return 0,0



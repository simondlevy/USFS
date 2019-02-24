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

import struct
import time

# Try MicroPython
try:

    from pyb import I2C

    def _initBus(bus): 
        return I2C(bus, I2C.MASTER)

    def _writeRegister(bus, address, subAddress, data): 
        bus.mem_write(data, address, subAddress)

    def _readRegisters(bus, address, subAddress, count):
        return bus.mem_read(count, address, subAddress)

# Default to Raspberry Pi
except:

    import smbus

    def _initBus(bus): 
        return smbus.SMBus(bus)

    def _writeRegister(bus, address, subAddress, data): 
        bus.write_byte_data(address, subAddress, data)

    def _readRegisters(bus, address, subAddress, count):
        bus.write_byte(address, subAddress)
        return [bus.read_byte(address) for k in range(count)]

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

        self.bus = _initBus(bus)

        self.errorStatus = 0

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

        if (self.errorStatus & 0x01): return 'Magnetometer error'
        if (self.errorStatus & 0x02): return 'Accelerometer error'
        if (self.errorStatus & 0x04): return 'Gyro error'
        if (self.errorStatus & 0x10): return 'Magnetometer ID not recognized'
        if (self.errorStatus & 0x20): return 'Accelerometer ID not recognized'
        if (self.errorStatus & 0x30): return 'Math error'
        if (self.errorStatus & 0x40): return 'Gyro ID not recognized'
        if (self.errorStatus & 0x80): return 'Invalid sample rate'

        # Ad-hoc
        if (self.errorStatus & 0x90): return 'Failed to put SENtral in pass-through mode'
        if (self.errorStatus & 0xA0): return 'Unable to read from SENtral EEPROM'
        if (self.errorStatus & 0xB0): return 'Unable to upload config to SENtral EEPROM'

        return 'Unknown error'

    def getProductId(self): 
    
        return self.readRegister(self.ProductID)
    

    def getRevisionId(self): 
    
        return self.readRegister(self.RevisionID)

    def getRamVersion(self):
    
        ram1 = self.readRegister(self.RAMVersion1)
        ram2 = self.readRegister(self.RAMVersion2)

        return ram1 << 8 | ram2

    def getRomVersion(self):
    
        rom1 = self.readRegister(self.ROMVersion1)
        rom2 = self.readRegister(self.ROMVersion2)

        return rom1 << 8 | rom2

    def getSentralStatus(self):
    
        return self.readRegister(self.SentralStatus) 

    def requestReset(self):
    
        self.writeRegister(self.ResetRequest, 0x01)

    def readThreeAxis(self, xreg):

        return struct.unpack('hhh', bytes(self.readRegisters(xreg, 6)))

    def setPassThroughMode(self):
    
        # First put SENtral in standby mode
        self.writeRegister(self.AlgorithmControl, 0x01)
        time.sleep(.005)

        # Place SENtral in pass-through mode
        self.writeRegister(self.PassThruControl, 0x01)
        while True:
            if (self.readRegister(self.PassThruStatus) & 0x01): break
            time.sleep(.005)
        
    def hasFeature(self, features):
    
        return features & self.readRegister(self.FeatureFlags)
    
    def setMasterMode(self):
    
        # Cancel pass-through mode
        self.writeRegister(self.PassThruControl, 0x00)
        while True:
            if (not (self.readRegister(self.PassThruStatus) & 0x01)): break
            time.sleep(.005)

        # Re-start algorithm
        self.writeRegister(self.AlgorithmControl, 0x00)
        while True:
            if (not (self.readRegister(self.AlgorithmStatus) & 0x01)): break
            time.sleep(.005)
        
    def setRunEnable(self):
    
        self.writeRegister(self.HostControl, 0x01) 
    
    def setRunDisable(self):
    
        self.writeRegister(self.HostControl, 0x00) 

    def setAccelLpfBandwidth(self, bw):
    
        self.writeRegister(self.ACC_LPF_BW, bw) 
    
    def setGyroLpfBandwidth(self, bw):
    
        self.writeRegister(self.GYRO_LPF_BW, bw) 

    def setQRateDivisor(self, divisor):
    
        self.writeRegister(self.QRateDivisor, divisor)

    def setMagRate(self, rate):
    
        self.writeRegister(self.MagRate, rate)

    def setAccelRate(self, rate):
    
        self.writeRegister(self.AccelRate, rate)

    def setGyroRate(self, rate):
    
        self.writeRegister(self.GyroRate, rate)

    def setBaroRate(self, rate):
    
        self.writeRegister(self.BaroRate, rate)

    def algorithmControlRequestParameterTransfer(self):
    
        self.writeRegister(self.AlgorithmControl, 0x80)

    def algorithmControlReset(self):
    
        self.writeRegister(self.AlgorithmControl, 0x00)
    
    def enableEvents(self, mask):
    
        self.writeRegister(self.EnableEvents, mask)

    def requestParamRead(self, param):
    
        self.writeRegister(self.ParamRequest, param) 
    
    def getParamAcknowledge(self):
    
        return self.readRegister(self.ParamAcknowledge)

    def readSavedParamByte0(self):
    
        return self.readRegister(self.SavedParamByte0)
    
    def readSavedParamByte1(self):
    
        return self.readRegister(self.SavedParamByte1)
    
    def readSavedParamByte2(self):
    
        return self.readRegister(self.SavedParamByte2)
    
    def readSavedParamByte3(self):
    
        return self.readRegister(self.SavedParamByte3)
    
    def getRunStatus(self):
    
        return self.readRegister(self.RunStatus)

    def getAlgorithmStatus(self):
    
        return self.readRegister(self.AlgorithmStatus)

    def getPassThruStatus(self):
    
        return self.readRegister(self.PassThruStatus)

    def getEventStatus(self):
    
        return self.readRegister(self.EventStatus)

    def getSensorStatus(self):
    
        return self.readRegister(self.SensorStatus)
    
    def getErrorStatus(self):
    
        return self.readRegister(self.ErrorRegister)
    
    def setGyroFs(self, gyro_fs):
    
        bites = [gyro_fs & (0xFF), (gyro_fs >> 8) & (0xFF), 0x00, 0x00]
        self.writeRegister(self.LoadParamByte0, bites[0]) #Gyro LSB
        self.writeRegister(self.LoadParamByte1, bites[1]) #Gyro MSB
        self.writeRegister(self.LoadParamByte2, bites[2]) #Unused
        self.writeRegister(self.LoadParamByte3, bites[3]) #Unused
        self.writeRegister(self.ParamRequest, 0xCB) #Parameter 75 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
        self.writeRegister(self.AlgorithmControl, 0x80) #Request parameter transfer procedure
        STAT = self.readRegister(self.ParamAcknowledge) #Check the parameter acknowledge register and loop until the result matches parameter request byte
        while(not (STAT==0xCB)):
            STAT = self.readRegister(self.ParamAcknowledge)
        
        self.writeRegister(self.ParamRequest, 0x00) #Parameter request = 0 to end parameter transfer process
        self.writeRegister(self.AlgorithmControl, 0x00) # Re-start algorithm
    
    def setMagAccFs(self, mag_fs, acc_fs):
    
        bites = [mag_fs & (0xFF), (mag_fs >> 8) & (0xFF),acc_fs & (0xFF), (acc_fs >> 8) & (0xFF)]
        self.writeRegister(self.LoadParamByte0, bites[0]) #Mag LSB
        self.writeRegister(self.LoadParamByte1, bites[1]) #Mag MSB
        self.writeRegister(self.LoadParamByte2, bites[2]) #Acc LSB
        self.writeRegister(self.LoadParamByte3, bites[3]) #Acc MSB
        self.writeRegister(self.ParamRequest, 0xCA) #Parameter 74 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
        self.writeRegister(self.AlgorithmControl, 0x80) #Request parameter transfer procedure
        STAT = self.readRegister(self.ParamAcknowledge) #Check the parameter acknowledge register and loop until the result matches parameter request byte
        while(not (STAT==0xCA)):
            STAT = self.readRegister(self.ParamAcknowledge)
        
        self.writeRegister(self.ParamRequest, 0x00) #Parameter request = 0 to end parameter transfer process
        self.writeRegister(self.AlgorithmControl, 0x00) # Re-start algorithm

    def loadParamByte0(self, value):
    
        self.writeRegister(self.LoadParamByte0, value)

    def loadParamByte1(self, value):
    
        self.writeRegister(self.LoadParamByte1, value)

    def loadParamByte2(self, value):
    
        self.writeRegister(self.LoadParamByte2, value)

    def loadParamByte3(self, value):
    
        self.writeRegister(self.LoadParamByte3, value)

    def writeGp36(self, value):
    
        self.writeRegister(self.GP36, value)

    def writeGp37(self, value):
    
        self.writeRegister(self.GP37, value)

    def writeGp38(self, value):
    
        self.writeRegister(self.GP38, value)

    def writeGp39(self, value):
    
        self.writeRegister(self.GP39, value)

    def writeGp40(self, value):
    
        self.writeRegister(self.GP40, value)

    def writeGp50(self, value):
    
        self.writeRegister(self.GP50, value)

    def writeGp51(self, value):
    
        self.writeRegister(self.GP51, value)

    def writeGp52(self, value):
    
        self.writeRegister(self.GP52, value)

    def writeGp53(self, value):
    
        self.writeRegister(self.GP53, value)
    
    def writeGp54(self, value):
    
        self.writeRegister(self.GP54, value)
    
    def writeGp55(self, value):
    
        self.writeRegister(self.GP55, value)
    
    def writeGp56(self, value):
    
        self.writeRegister(self.GP56, value)
    
    def readAccelerometer(self):
    
        return self.readThreeAxis(self.AX)

    def readGyrometer(self):
    
        return self.readThreeAxis(self.GX)
    
    def readBarometer(self):

        return self.readFloat(self.Baro, 1013.25), self.readFloat(self.Temp, 0)


    def readMagnetometer(self):
    
        return self.readThreeAxis(self.MX)
    
    def readQuaternion(self):
    
        rawData = self.readRegisters(self.QX, 16)       

        qx = self.uint32_reg_to_float(rawData[0:4])
        qy = self.uint32_reg_to_float(rawData[4:8])
        qz = self.uint32_reg_to_float(rawData[8:12])
        qw = self.uint32_reg_to_float(rawData[12:16]) 

        return qw, qx, qy, qz
    
    def setIntegerParam(self, param, param_val):
    
        bites = [param_val & (0xFF),(param_val >> 8) & (0xFF),(param_val >> 16) & (0xFF),(param_val >> 24) & (0xFF)]
        param = param | 0x80 #Parameter is the decimal value with the MSB set high to indicate a paramter write processs
        self.writeRegister(self.LoadParamByte0, bites[0]) #Param LSB
        self.writeRegister(self.LoadParamByte1, bites[1])
        self.writeRegister(self.LoadParamByte2, bites[2])
        self.writeRegister(self.LoadParamByte3, bites[3]) #Param MSB
        self.writeRegister(self.ParamRequest, param)
        self.writeRegister(self.AlgorithmControl, 0x80) #Request parameter transfer procedure
        STAT = self.readRegister(self.ParamAcknowledge) #Check the parameter acknowledge register and loop until the result matches parameter request byte
        while(not (STAT==param)):
            STAT = self.readRegister(self.ParamAcknowledge)
        
        self.writeRegister(self.ParamRequest, 0x00) #Parameter request = 0 to end parameter transfer process
        self.writeRegister(self.AlgorithmControl, 0x00) # Re-start algorithm
    

    def getFullScaleRanges(self):
    
        # Read sensor new FS values from parameter space
        self.writeRegister(self.ParamRequest, 0x4A) # Request to read  parameter 74
        self.writeRegister(self.AlgorithmControl, 0x80) # Request parameter transfer process
        param_xfer = self.readRegister(self.ParamAcknowledge)
        while(not (param_xfer==0x4A)):
            param_xfer = self.readRegister(self.ParamAcknowledge)
        
        params = [
                self.readRegister(self.SavedParamByte0),
                self.readRegister(self.SavedParamByte1),
                self.readRegister(self.SavedParamByte2),
                self.readRegister(self.SavedParamByte3)]
        magFs = (params[1]<<8) | params[0]
        accFs = (params[3]<<8) | params[2]
        self.writeRegister(self.ParamRequest, 0x4B) # Request to read  parameter 75
        param_xfer = self.readRegister(self.ParamAcknowledge)
        while (not (param_xfer==0x4B)):
            param_xfer = self.readRegister(self.ParamAcknowledge)
        
        params = [
                self.readRegister(self.SavedParamByte0), 
                self.readRegister(self.SavedParamByte1), 
                self.readRegister(self.SavedParamByte2), 
                self.readRegister(self.SavedParamByte3)]
        gyroFs = (params[1]<<8) | params[0]
        self.writeRegister(self.ParamRequest, 0x00) #End parameter transfer
        self.writeRegister(self.AlgorithmControl, 0x00) # re-enable algorithm

        return accFs, gyroFs, magFs
    
    def getActualMagRate(self):
    
        return self.readRegister(self.ActualMagRate)

    def getActualAccelRate(self):
    
        return self.readRegister(self.ActualAccelRate)
    
    def getActualGyroRate(self):
    
        return self.readRegister(self.ActualGyroRate)

    def getActualBaroRate(self):
    
        return self.readRegister(self.ActualBaroRate)

    def getActualTempRate(self):
    
        return self.readRegister(self.ActualTempRate)

    def writeRegister(self, subAddress, data):
    
        _writeRegister(self.bus, self.ADDRESS, subAddress, data)

    def readRegister(self, subAddress):

        return self.readRegisters(subAddress, 1)[0]

    def readRegisters(self, subAddress, count):

        return _readRegisters(self.bus, self.ADDRESS, subAddress, count)

    def uint32_reg_to_float(self, buf):

        return struct.unpack('f', bytes(buf))[0]

    def readFloat(self, reg, offset):
    
        return struct.unpack('h', bytes(self.readRegisters(reg, 2)))[0] * .01 + offset

# =======================================================================================

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

        time.sleep(.1)

        # Enter EM7180 initialized state
        self.em7180.setRunDisable()# set SENtral in initialized state to configure registers
        self.em7180.setMasterMode()
        self.em7180.setRunEnable()
        self.em7180.setRunDisable()# set SENtral in initialized state to configure registers

        # Setup LPF bandwidth (BEFORE setting ODR's)
        self.em7180.setAccelLpfBandwidth(0x03) # 41Hz
        self.em7180.setGyroLpfBandwidth(0x03)  # 41Hz

        # Set accel/gyro/mage desired ODR rates
        self.em7180.setQRateDivisor(self.qRateDivisor-1)
        self.em7180.setMagRate(self.magRate)
        self.em7180.setAccelRate(self.accelRate//10)
        self.em7180.setGyroRate(self.gyroRate//10)
        self.em7180.setBaroRate(0x80 | self.baroRate) # 0x80 = enable bit

        # Configure operating modeA
        self.em7180.algorithmControlReset()# read scale sensor data

        # Enable interrupt to host upon certain events:
        # quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
        self.em7180.enableEvents(0x07)

        # Enable EM7180 run mode
        self.em7180.setRunEnable()# set SENtral in normal run mode
        time.sleep(0.1)

        # Disable stillness mode
        self.em7180.setIntegerParam (0x49, 0x00)

        # Success
        return not self.em7180.getSensorStatus()

    def getErrorString(self):

        return self.em7180.getErrorString()

    def checkEventStatus(self):

        # Check event status register, way to check data ready by checkEventStatusing rather than interrupt
        self.eventStatus = self.em7180.getEventStatus() # reading clears the register

    def gotError(self):

        return self.eventStatus & 0x02

    def gotQuaternion(self):

        return self.eventStatus & 0x04

    def gotAccelerometer(self):

        return self.eventStatus & 0x10

    def gotMagnetometer(self):

        return self.eventStatus & 0x08

    def gotGyrometer(self):

        return self.eventStatus & 0x20

    def gotBarometer(self):

        return self.eventStatus & 0x40

    def readQuaternion(self):

        return self.em7180.readQuaternion()

    def readAccelerometer(self):

        return self.readThreeAxis(self.em7180.AX, 0.000488)

    def readGyrometer(self):

        return self.readThreeAxis(self.em7180.GX, 0.153)

    def readMagnetometer(self):

        return self.readThreeAxis(self.em7180.MX, 0.305176)

    def readBarometer(self):

        return self.em7180.readBarometer()

    def readThreeAxis(self, regx, scale):

        x,y,z = self.em7180.readThreeAxis(regx)

        return x*scale, y*scale, z*scale

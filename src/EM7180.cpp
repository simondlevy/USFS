/* 

   EM7180.cpp: Class implementation for EM7180 SENtral Sensor

   Copyright (C) 2018 Simon D. Levy

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/MPU9250_BMP280

   This file is part of EM7180.

   EM7180 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   EM7180 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "EM7180.h"
#include <CrossPlatformI2C_Core.h>

float EM7180::uint32_reg_to_float (uint8_t *buf)
{
    union {
        uint32_t ui32;
        float f;
    } u;

    u.ui32 = (((uint32_t)buf[0]) +
            (((uint32_t)buf[1]) <<  8) +
            (((uint32_t)buf[2]) << 16) +
            (((uint32_t)buf[3]) << 24));
    return u.f;
}

bool EM7180::begin(uint8_t bus)
{
    _i2c = cpi2c_open(ADDRESS, bus);

    errorStatus = 0;

    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    for (int attempts=0; attempts<10; ++attempts) {
        if (readRegister(SentralStatus) & 0x01) {
            if(readRegister(SentralStatus) & 0x01) { }
            if(readRegister(SentralStatus) & 0x02) { }
            if(readRegister(SentralStatus) & 0x04) {
                errorStatus = 0xB0;
                return false;
            }
            if(readRegister(SentralStatus) & 0x08) { }
            if(readRegister(SentralStatus) & 0x10) {
                errorStatus = 0xB0;
                return false;
            }
            break;
        }
        writeRegister(ResetRequest, 0x01);
        delay(500);  
    }


    if (readRegister(SentralStatus) & 0x04) {
        errorStatus = 0xB0;
        return false;
    }

    return true;
}

const char * EM7180::getErrorString(void)
{
    if (errorStatus & 0x01) return "Magnetometer error";
    if (errorStatus & 0x02) return "Accelerometer error";
    if (errorStatus & 0x04) return "Gyro error";
    if (errorStatus & 0x10) return "Magnetometer ID not recognized";
    if (errorStatus & 0x20) return "Accelerometer ID not recognized";
    if (errorStatus & 0x30) return "Math error";
    if (errorStatus & 0x40) return "Gyro ID not recognized";
    if (errorStatus & 0x80) return "Invalid sample rate";

    // Ad-hoc
    if (errorStatus & 0x90) return "Failed to put SENtral in pass-through mode";
    if (errorStatus & 0xA0) return "Unable to read from SENtral EEPROM";
    if (errorStatus & 0xB0) return "Unable to upload config to SENtral EEPROM";

    return "Unknown error";
}

uint8_t EM7180::getProductId(void) 
{
    return readRegister(ProductID);
}

uint8_t EM7180::getRevisionId(void) 
{
    return readRegister(RevisionID);
}

uint16_t EM7180::getRamVersion(void)
{
    uint16_t ram1 = readRegister(RAMVersion1);
    uint16_t ram2 = readRegister(RAMVersion2);

    return ram1 << 8 | ram2;
}

uint16_t EM7180::getRomVersion(void)
{
    uint16_t rom1 = readRegister(ROMVersion1);
    uint16_t rom2 = readRegister(ROMVersion2);

    return rom1 << 8 | rom2;
}

uint8_t EM7180::getSentralStatus(void)
{
    return readRegister(SentralStatus); 
}

void EM7180::requestReset(void)
{
    writeRegister(ResetRequest, 0x01);
}

void EM7180::readThreeAxis(uint8_t xreg, int16_t & x, int16_t & y, int16_t & z)
{
    uint8_t rawData[6];  // x/y/z register data stored here
    readRegisters(xreg, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void EM7180::setPassThroughMode()
{
    // First put SENtral in standby mode
    writeRegister(AlgorithmControl, 0x01);
    delay(5);

    // Place SENtral in pass-through mode
    writeRegister(PassThruControl, 0x01);
    while (true) {
        if (readRegister(PassThruStatus) & 0x01) break;
        delay(5);
    }
}

bool EM7180::hasFeature(uint8_t features)
{
    return features & readRegister(FeatureFlags);
}

void EM7180::setMasterMode()
{
    // Cancel pass-through mode
    writeRegister(PassThruControl, 0x00);
    while (true) {
        if (!(readRegister(PassThruStatus) & 0x01)) break;
        delay(5);
    }

    // Re-start algorithm
    writeRegister(AlgorithmControl, 0x00);
    while (true) {
        if (!(readRegister(AlgorithmStatus) & 0x01)) break;
        delay(5);
    }
}

void EM7180::setRunEnable(void)
{
    writeRegister(HostControl, 0x01); 
}

void EM7180::setRunDisable(void)
{
    writeRegister(HostControl, 0x00); 
}

void EM7180::setAccelLpfBandwidth(uint8_t bw)
{
    writeRegister(ACC_LPF_BW, bw); 
}

void EM7180::setGyroLpfBandwidth(uint8_t bw)
{
    writeRegister(GYRO_LPF_BW, bw); 
}

void EM7180::setQRateDivisor(uint8_t divisor)
{
    writeRegister(QRateDivisor, divisor);
}

void EM7180::setMagRate(uint8_t rate)
{
    writeRegister(MagRate, rate);
}

void EM7180::setAccelRate(uint8_t rate)
{
    writeRegister(AccelRate, rate);
}

void EM7180::setGyroRate(uint8_t rate)
{
    writeRegister(GyroRate, rate);
}

void EM7180::setBaroRate(uint8_t rate)
{
    writeRegister(BaroRate, rate);
}

void EM7180::algorithmControlRequestParameterTransfer(void)
{
    writeRegister(AlgorithmControl, 0x80);
}

void EM7180::algorithmControlReset(void)
{
    writeRegister(AlgorithmControl, 0x00);
}

void EM7180::enableEvents(uint8_t mask)
{
    writeRegister(EnableEvents, mask);
}

void EM7180::requestParamRead(uint8_t param)
{
    writeRegister(ParamRequest, param); 
}

uint8_t EM7180::getParamAcknowledge(void)
{
    return readRegister(ParamAcknowledge);
}

uint8_t EM7180::readSavedParamByte0(void)
{
    return readRegister(SavedParamByte0);
}

uint8_t EM7180::readSavedParamByte1(void)
{
    return readRegister(SavedParamByte1);
}

uint8_t EM7180::readSavedParamByte2(void)
{
    return readRegister(SavedParamByte2);
}

uint8_t EM7180::readSavedParamByte3(void)
{
    return readRegister(SavedParamByte3);
}

uint8_t EM7180::getRunStatus(void)
{
    return readRegister(RunStatus);
}

uint8_t EM7180::getAlgorithmStatus(void)
{
    return readRegister(AlgorithmStatus);
}

uint8_t EM7180::getPassThruStatus(void)
{
    return readRegister(PassThruStatus);
}

uint8_t EM7180::getEventStatus(void)
{
    return readRegister(EventStatus);
}

uint8_t EM7180::getSensorStatus(void)
{
    return readRegister(SensorStatus);
}

uint8_t EM7180::getErrorStatus(void)
{
    return readRegister(ErrorRegister);
}

void EM7180::setGyroFs(uint16_t gyro_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = gyro_fs & (0xFF);
    bytes[1] = (gyro_fs >> 8) & (0xFF);
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    writeRegister(LoadParamByte0, bytes[0]); //Gyro LSB
    writeRegister(LoadParamByte1, bytes[1]); //Gyro MSB
    writeRegister(LoadParamByte2, bytes[2]); //Unused
    writeRegister(LoadParamByte3, bytes[3]); //Unused
    writeRegister(ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
    writeRegister(AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readRegister(ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCB)) {
        STAT = readRegister(ParamAcknowledge);
    }
    writeRegister(ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeRegister(AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180::setMagAccFs(uint16_t mag_fs, uint16_t acc_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    writeRegister(LoadParamByte0, bytes[0]); //Mag LSB
    writeRegister(LoadParamByte1, bytes[1]); //Mag MSB
    writeRegister(LoadParamByte2, bytes[2]); //Acc LSB
    writeRegister(LoadParamByte3, bytes[3]); //Acc MSB
    writeRegister(ParamRequest, 0xCA); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
    writeRegister(AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readRegister(ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCA)) {
        STAT = readRegister(ParamAcknowledge);
    }
    writeRegister(ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeRegister(AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180::loadParamByte0(uint8_t value)
{
    writeRegister(LoadParamByte0, value);
}

void EM7180::loadParamByte1(uint8_t value)
{
    writeRegister(LoadParamByte1, value);
}

void EM7180::loadParamByte2(uint8_t value)
{
    writeRegister(LoadParamByte2, value);
}

void EM7180::loadParamByte3(uint8_t value)
{
    writeRegister(LoadParamByte3, value);
}

void EM7180::writeGp36(uint8_t value)
{
    writeRegister(GP36, value);
}

void EM7180::writeGp37(uint8_t value)
{
    writeRegister(GP37, value);
}

void EM7180::writeGp38(uint8_t value)
{
    writeRegister(GP38, value);
}

void EM7180::writeGp39(uint8_t value)
{
    writeRegister(GP39, value);
}

void EM7180::writeGp40(uint8_t value)
{
    writeRegister(GP40, value);
}

void EM7180::writeGp50(uint8_t value)
{
    writeRegister(GP50, value);
}

void EM7180::writeGp51(uint8_t value)
{
    writeRegister(GP51, value);
}

void EM7180::writeGp52(uint8_t value)
{
    writeRegister(GP52, value);
}

void EM7180::writeGp53(uint8_t value)
{
    writeRegister(GP53, value);
}

void EM7180::writeGp54(uint8_t value)
{
    writeRegister(GP54, value);
}

void EM7180::writeGp55(uint8_t value)
{
    writeRegister(GP55, value);
}

void EM7180::writeGp56(uint8_t value)
{
    writeRegister(GP56, value);
}

void EM7180::readAccelerometer(int16_t & ax, int16_t & ay, int16_t & az)
{
    readThreeAxis(AX, ax, ay, az);
}

void EM7180::readGyrometer(int16_t & gx, int16_t & gy, int16_t & gz)
{
    readThreeAxis(GX, gx, gy, gz);
}

void EM7180::readBarometer(float & pressure, float & temperature)
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here

    readRegisters(Baro, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    int16_t rawPressure =  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    pressure = (float)rawPressure *.01f + 1013.25f; // pressure in millibars

    // get BMP280 temperature
    readRegisters(Temp, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    int16_t rawTemperature =  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value

    temperature = (float) rawTemperature*0.01;  // temperature in degrees C
}


void EM7180::readMagnetometer(int16_t & mx, int16_t & my, int16_t & mz)
{
    readThreeAxis(MX, mx, my, mz);
}

void EM7180::readQuaternion(float & qw, float & qx, float & qy, float &qz)
{
    uint8_t rawData[16];  // x/y/z/w quaternion register data stored here (note unusual order!)

    readRegisters(QX, 16, &rawData[0]);       

    qx = uint32_reg_to_float (&rawData[0]);
    qy = uint32_reg_to_float (&rawData[4]);
    qz = uint32_reg_to_float (&rawData[8]);
    qw = uint32_reg_to_float (&rawData[12]); 
}

void EM7180::setIntegerParam(uint8_t param, uint32_t param_val) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = param_val & (0xFF);
    bytes[1] = (param_val >> 8) & (0xFF);
    bytes[2] = (param_val >> 16) & (0xFF);
    bytes[3] = (param_val >> 24) & (0xFF);
    param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    writeRegister(LoadParamByte0, bytes[0]); //Param LSB
    writeRegister(LoadParamByte1, bytes[1]);
    writeRegister(LoadParamByte2, bytes[2]);
    writeRegister(LoadParamByte3, bytes[3]); //Param MSB
    writeRegister(ParamRequest, param);
    writeRegister(AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readRegister(ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==param)) {
        STAT = readRegister(ParamAcknowledge);
    }
    writeRegister(ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeRegister(AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180::getFullScaleRanges(uint8_t& accFs, uint16_t& gyroFs, uint16_t& magFs)
{
    uint8_t param[4];

    // Read sensor new FS values from parameter space
    writeRegister(ParamRequest, 0x4A); // Request to read  parameter 74
    writeRegister(AlgorithmControl, 0x80); // Request parameter transfer process
    uint8_t param_xfer = readRegister(ParamAcknowledge);
    while(!(param_xfer==0x4A)) {
        param_xfer = readRegister(ParamAcknowledge);
    }
    param[0] = readRegister(SavedParamByte0);
    param[1] = readRegister(SavedParamByte1);
    param[2] = readRegister(SavedParamByte2);
    param[3] = readRegister(SavedParamByte3);
    magFs = ((uint16_t)(param[1]<<8) | param[0]);
    accFs = ((uint8_t)(param[3]<<8) | param[2]);
    writeRegister(ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readRegister(ParamAcknowledge);
    while(!(param_xfer==0x4B)) {
        param_xfer = readRegister(ParamAcknowledge);
    }
    param[0] = readRegister(SavedParamByte0);
    param[1] = readRegister(SavedParamByte1);
    param[2] = readRegister(SavedParamByte2);
    param[3] = readRegister(SavedParamByte3);
    gyroFs = ((uint16_t)(param[1]<<8) | param[0]);
    writeRegister(ParamRequest, 0x00); //End parameter transfer
    writeRegister(AlgorithmControl, 0x00); // re-enable algorithm
}

uint8_t EM7180::getActualMagRate()
{
    return readRegister(ActualMagRate);
}

uint16_t EM7180::getActualAccelRate()
{
    return readRegister(ActualAccelRate);
}

uint16_t EM7180::getActualGyroRate()
{
    return readRegister(ActualGyroRate);
}

uint8_t EM7180::getActualBaroRate()
{
    return readRegister(ActualBaroRate);
}

uint8_t EM7180::getActualTempRate()
{
    return readRegister(ActualTempRate);
}

uint8_t EM7180::readRegister(uint8_t subAddress)
{
    uint8_t data;
    readRegisters(subAddress, 1, &data);
    return data;                       
}

void EM7180::writeRegister(uint8_t subAddress, uint8_t data)
{
    cpi2c_writeRegister(_i2c, subAddress, data);
}

void EM7180::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    cpi2c_readRegisters(_i2c, subAddress, count, dest);
}

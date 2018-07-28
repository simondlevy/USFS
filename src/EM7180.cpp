/* 

   EM7180.cpp: Class implementation for EM7180 SENtral Sensor

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/EM7180_MPU9250_BMP280

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
#include "CrossPlatformI2C.h"

#if defined(ARDUINO)
#include <Arduino.h>
#elif defined(__arm__)
extern "C" {void delay(uint32_t msec);}
#else
void delay(uint32_t msec);
#endif

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

bool EM7180::begin(void)
{
    _i2c = cpi2c_open(EM7180_ADDRESS);

    errorStatus = 0;

    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    for (int attempts=0; attempts<10; ++attempts) {
        if (readRegister(EM7180_SentralStatus) & 0x01) {
            if(readRegister(EM7180_SentralStatus) & 0x01) { }
            if(readRegister(EM7180_SentralStatus) & 0x02) { }
            if(readRegister(EM7180_SentralStatus) & 0x04) {
                errorStatus = 0xB0;
                return false;
            }
            if(readRegister(EM7180_SentralStatus) & 0x08) { }
            if(readRegister(EM7180_SentralStatus) & 0x10) {
                errorStatus = 0xB0;
                return false;
            }
            break;
        }
        writeRegister(EM7180_ResetRequest, 0x01);
        delay(500);  
    }


    if (readRegister(EM7180_SentralStatus) & 0x04) {
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
    return readRegister(EM7180_ProductID);
}

uint8_t EM7180::getRevisionId(void) 
{
    return readRegister(EM7180_RevisionID);
}

uint16_t EM7180::getRamVersion(void)
{
    uint16_t ram1 = readRegister(EM7180_RAMVersion1);
    uint16_t ram2 = readRegister(EM7180_RAMVersion2);

    return ram1 << 8 | ram2;
}

uint16_t EM7180::getRomVersion(void)
{
    uint16_t rom1 = readRegister(EM7180_ROMVersion1);
    uint16_t rom2 = readRegister(EM7180_ROMVersion2);

    return rom1 << 8 | rom2;
}

uint8_t EM7180::getSentralStatus(void)
{
    return readRegister(EM7180_SentralStatus); 
}

void EM7180::requestReset(void)
{
    writeRegister(EM7180_ResetRequest, 0x01);
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
    writeRegister(EM7180_AlgorithmControl, 0x01);
    delay(5);

    // Place SENtral in pass-through mode
    writeRegister(EM7180_PassThruControl, 0x01);
    while (true) {
        if (readRegister(EM7180_PassThruStatus) & 0x01) break;
        delay(5);
    }
}

bool EM7180::hasFeature(uint8_t features)
{
    return features & readRegister(EM7180_FeatureFlags);
}

void EM7180::setMasterMode()
{
    // Cancel pass-through mode
    writeRegister(EM7180_PassThruControl, 0x00);
    while (true) {
        if (!(readRegister(EM7180_PassThruStatus) & 0x01)) break;
        delay(5);
    }

    // Re-start algorithm
    writeRegister(EM7180_AlgorithmControl, 0x00);
    while (true) {
        if (!(readRegister(EM7180_AlgorithmStatus) & 0x01)) break;
        delay(5);
    }
}

void EM7180::setRunEnable(void)
{
    writeRegister(EM7180_HostControl, 0x01); 
}

void EM7180::setRunDisable(void)
{
    writeRegister(EM7180_HostControl, 0x00); 
}

void EM7180::setAccelLpfBandwidth(uint8_t bw)
{
    writeRegister(EM7180_ACC_LPF_BW, bw); 
}

void EM7180::setGyroLpfBandwidth(uint8_t bw)
{
    writeRegister(EM7180_GYRO_LPF_BW, bw); 
}

void EM7180::setQRateDivisor(uint8_t divisor)
{
    writeRegister(EM7180_QRateDivisor, divisor);
}

void EM7180::setMagRate(uint8_t rate)
{
    writeRegister(EM7180_MagRate, rate);
}

void EM7180::setAccelRate(uint8_t rate)
{
    writeRegister(EM7180_AccelRate, rate);
}

void EM7180::setGyroRate(uint8_t rate)
{
    writeRegister(EM7180_GyroRate, rate);
}

void EM7180::setBaroRate(uint8_t rate)
{
    writeRegister(EM7180_BaroRate, rate);
}

void EM7180::algorithmControlRequestParameterTransfer(void)
{
    writeRegister(EM7180_AlgorithmControl, 0x80);
}

void EM7180::algorithmControlReset(void)
{
    writeRegister(EM7180_AlgorithmControl, 0x00);
}

void EM7180::enableEvents(uint8_t mask)
{
    writeRegister(EM7180_EnableEvents, mask);
}

void EM7180::requestParamRead(uint8_t param)
{
    writeRegister(EM7180_ParamRequest, param); 
}

uint8_t EM7180::getParamAcknowledge(void)
{
    return readRegister(EM7180_ParamAcknowledge);
}

uint8_t EM7180::readSavedParamByte0(void)
{
    return readRegister(EM7180_SavedParamByte0);
}

uint8_t EM7180::readSavedParamByte1(void)
{
    return readRegister(EM7180_SavedParamByte1);
}

uint8_t EM7180::readSavedParamByte2(void)
{
    return readRegister(EM7180_SavedParamByte2);
}

uint8_t EM7180::readSavedParamByte3(void)
{
    return readRegister(EM7180_SavedParamByte3);
}

uint8_t EM7180::getRunStatus(void)
{
    return readRegister(EM7180_RunStatus);
}

uint8_t EM7180::getAlgorithmStatus(void)
{
    return readRegister(EM7180_AlgorithmStatus);
}

uint8_t EM7180::getPassThruStatus(void)
{
    return readRegister(EM7180_PassThruStatus);
}

uint8_t EM7180::getEventStatus(void)
{
    return readRegister(EM7180_EventStatus);
}

uint8_t EM7180::getSensorStatus(void)
{
    return readRegister(EM7180_SensorStatus);
}

uint8_t EM7180::getErrorStatus(void)
{
    return readRegister(EM7180_ErrorRegister);
}

void EM7180::setGyroFs(uint16_t gyro_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = gyro_fs & (0xFF);
    bytes[1] = (gyro_fs >> 8) & (0xFF);
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    writeRegister(EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
    writeRegister(EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
    writeRegister(EM7180_LoadParamByte2, bytes[2]); //Unused
    writeRegister(EM7180_LoadParamByte3, bytes[3]); //Unused
    writeRegister(EM7180_ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
    writeRegister(EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readRegister(EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCB)) {
        STAT = readRegister(EM7180_ParamAcknowledge);
    }
    writeRegister(EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeRegister(EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180::setMagAccFs(uint16_t mag_fs, uint16_t acc_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    writeRegister(EM7180_LoadParamByte0, bytes[0]); //Mag LSB
    writeRegister(EM7180_LoadParamByte1, bytes[1]); //Mag MSB
    writeRegister(EM7180_LoadParamByte2, bytes[2]); //Acc LSB
    writeRegister(EM7180_LoadParamByte3, bytes[3]); //Acc MSB
    writeRegister(EM7180_ParamRequest, 0xCA); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
    writeRegister(EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readRegister(EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCA)) {
        STAT = readRegister(EM7180_ParamAcknowledge);
    }
    writeRegister(EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeRegister(EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180::loadParamByte0(uint8_t value)
{
    writeRegister(EM7180_LoadParamByte0, value);
}

void EM7180::loadParamByte1(uint8_t value)
{
    writeRegister(EM7180_LoadParamByte1, value);
}

void EM7180::loadParamByte2(uint8_t value)
{
    writeRegister(EM7180_LoadParamByte2, value);
}

void EM7180::loadParamByte3(uint8_t value)
{
    writeRegister(EM7180_LoadParamByte3, value);
}

void EM7180::writeGp36(uint8_t value)
{
    writeRegister(EM7180_GP36, value);
}

void EM7180::writeGp37(uint8_t value)
{
    writeRegister(EM7180_GP37, value);
}

void EM7180::writeGp38(uint8_t value)
{
    writeRegister(EM7180_GP38, value);
}

void EM7180::writeGp39(uint8_t value)
{
    writeRegister(EM7180_GP39, value);
}

void EM7180::writeGp40(uint8_t value)
{
    writeRegister(EM7180_GP40, value);
}

void EM7180::writeGp50(uint8_t value)
{
    writeRegister(EM7180_GP50, value);
}

void EM7180::writeGp51(uint8_t value)
{
    writeRegister(EM7180_GP51, value);
}

void EM7180::writeGp52(uint8_t value)
{
    writeRegister(EM7180_GP52, value);
}

void EM7180::writeGp53(uint8_t value)
{
    writeRegister(EM7180_GP53, value);
}

void EM7180::writeGp54(uint8_t value)
{
    writeRegister(EM7180_GP54, value);
}

void EM7180::writeGp55(uint8_t value)
{
    writeRegister(EM7180_GP55, value);
}

void EM7180::writeGp56(uint8_t value)
{
    writeRegister(EM7180_GP56, value);
}

void EM7180::readAccelerometer(int16_t & ax, int16_t & ay, int16_t & az)
{
    readThreeAxis(EM7180_AX, ax, ay, az);
}

void EM7180::readGyrometer(int16_t & gx, int16_t & gy, int16_t & gz)
{
    readThreeAxis(EM7180_GX, gx, gy, gz);
}

void EM7180::readBarometer(float & pressure, float & temperature)
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here

    readRegisters(EM7180_Baro, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    int16_t rawPressure =  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    pressure = (float)rawPressure *.01f + 1013.25f; // pressure in millibars

    // get BMP280 temperature
    readRegisters(EM7180_Temp, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    int16_t rawTemperature =  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value

    temperature = (float) rawTemperature*0.01;  // temperature in degrees C
}


void EM7180::readMagnetometer(int16_t & mx, int16_t & my, int16_t & mz)
{
    readThreeAxis(EM7180_MX, mx, my, mz);
}

void EM7180::readQuaternion(float & qw, float & qx, float & qy, float &qz)
{
    uint8_t rawData[16];  // x/y/z/w quaternion register data stored here (note unusual order!)

    readRegisters(EM7180_QX, 16, &rawData[0]);       

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
    writeRegister(EM7180_LoadParamByte0, bytes[0]); //Param LSB
    writeRegister(EM7180_LoadParamByte1, bytes[1]);
    writeRegister(EM7180_LoadParamByte2, bytes[2]);
    writeRegister(EM7180_LoadParamByte3, bytes[3]); //Param MSB
    writeRegister(EM7180_ParamRequest, param);
    writeRegister(EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readRegister(EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==param)) {
        STAT = readRegister(EM7180_ParamAcknowledge);
    }
    writeRegister(EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeRegister(EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180::getFullScaleRanges(uint8_t& accFs, uint16_t& gyroFs, uint16_t& magFs)
{
    uint8_t param[4];

    // Read sensor new FS values from parameter space
    writeRegister(EM7180_ParamRequest, 0x4A); // Request to read  parameter 74
    writeRegister(EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    uint8_t param_xfer = readRegister(EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4A)) {
        param_xfer = readRegister(EM7180_ParamAcknowledge);
    }
    param[0] = readRegister(EM7180_SavedParamByte0);
    param[1] = readRegister(EM7180_SavedParamByte1);
    param[2] = readRegister(EM7180_SavedParamByte2);
    param[3] = readRegister(EM7180_SavedParamByte3);
    magFs = ((uint16_t)(param[1]<<8) | param[0]);
    accFs = ((uint8_t)(param[3]<<8) | param[2]);
    writeRegister(EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readRegister(EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4B)) {
        param_xfer = readRegister(EM7180_ParamAcknowledge);
    }
    param[0] = readRegister(EM7180_SavedParamByte0);
    param[1] = readRegister(EM7180_SavedParamByte1);
    param[2] = readRegister(EM7180_SavedParamByte2);
    param[3] = readRegister(EM7180_SavedParamByte3);
    gyroFs = ((uint16_t)(param[1]<<8) | param[0]);
    writeRegister(EM7180_ParamRequest, 0x00); //End parameter transfer
    writeRegister(EM7180_AlgorithmControl, 0x00); // re-enable algorithm
}

uint8_t EM7180::getActualMagRate()
{
    return readRegister(EM7180_ActualMagRate);
}

uint16_t EM7180::getActualAccelRate()
{
    return readRegister(EM7180_ActualAccelRate);
}

uint16_t EM7180::getActualGyroRate()
{
    return readRegister(EM7180_ActualGyroRate);
}

uint8_t EM7180::getActualBaroRate()
{
    return readRegister(EM7180_ActualBaroRate);
}

uint8_t EM7180::getActualTempRate()
{
    return readRegister(EM7180_ActualTempRate);
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
// EM7180_Master ==========================================================================================================

EM7180_Master::EM7180_Master(
        uint8_t aRes, 
        uint16_t gRes, 
        uint16_t mRes, 
        uint8_t magRate, 
        uint16_t accelRate, 
        uint16_t gyroRate, 
        uint8_t baroRate, 
        uint8_t qRateDivisor) 
{
    _aRes = aRes;
    _gRes = gRes;
    _mRes = mRes;
    _magRate = magRate;
    _accelRate = accelRate;
    _gyroRate = gyroRate; 
    _baroRate = baroRate;
    _qRateDivisor = qRateDivisor;
}

uint8_t EM7180_Master::getProductId(void) 
{
    return _em7180.getProductId();
}

uint8_t EM7180_Master::getRevisionId(void) 
{
    return _em7180.getRevisionId();
}

uint16_t EM7180_Master::getRamVersion(void)
{
    return _em7180.getRamVersion();
}

uint16_t EM7180_Master::getRomVersion(void)
{
    return _em7180.getRomVersion();
}

bool EM7180_Master::hasBaro(void)
{
    return _em7180.hasFeature(0x01);
}

bool EM7180_Master::hasHumidity(void)
{
    return _em7180.hasFeature(0x02);
}

bool EM7180_Master::hasTemperature(void)
{
    return _em7180.hasFeature(0x04);
}

bool EM7180_Master::hasCustom1(void)
{
    return _em7180.hasFeature(0x08);
}

bool EM7180_Master::hasCustom2(void)
{
    return _em7180.hasFeature(0x10);
}

bool EM7180_Master::hasCustom3(void)
{
    return _em7180.hasFeature(0x20);
}

void EM7180_Master::setIntegerParam(uint8_t param, uint32_t param_val) 
{
    _em7180.setIntegerParam(param, param_val);
}

void EM7180_Master::setMagAccFs(uint16_t mag_fs, uint16_t acc_fs) 
{
    _em7180.setMagAccFs(mag_fs, acc_fs);
}

void EM7180_Master::setGyroFs(uint16_t gyro_fs) 
{
    _em7180.setGyroFs(gyro_fs);
}

bool EM7180_Master::algorithmStatus(uint8_t status)
{
    return _em7180.getAlgorithmStatus() & status;
}

const char * EM7180_Master::getErrorString(void)
{
    return _em7180.getErrorString();
}

bool EM7180_Master::begin(void)
{
    // Fail immediately if unable to upload EEPROM
    if (!_em7180.begin()) return false;

    // Enter EM7180 initialized state
    _em7180.setRunDisable();// set SENtral in initialized state to configure registers
    _em7180.setMasterMode();
    _em7180.setRunEnable();
    _em7180.setRunDisable();// set SENtral in initialized state to configure registers

    // Setup LPF bandwidth (BEFORE setting ODR's)
    _em7180.setAccelLpfBandwidth(0x03); // 41Hz
    _em7180.setGyroLpfBandwidth(0x03);  // 41Hz

    // Set accel/gyro/mage desired ODR rates
    _em7180.setQRateDivisor(_qRateDivisor-1);
    _em7180.setMagRate(_magRate);
    _em7180.setAccelRate(_accelRate/10);
    _em7180.setGyroRate(_gyroRate/10);
    _em7180.setBaroRate(0x80 | _baroRate); // 0x80 = enable bit

    // Configure operating modeA
    _em7180.algorithmControlReset();// read scale sensor data

    // Enable interrupt to host upon certain events:
    // quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    _em7180.enableEvents(0x07);

    // Enable EM7180 run mode
    _em7180.setRunEnable();// set SENtral in normal run mode
    delay(100);

    // Disable stillness mode
    setIntegerParam (0x49, 0x00);

    // Write desired sensor full scale ranges to the EM7180
    setMagAccFs(_mRes, _aRes);
    setGyroFs(_gRes); 

    // Success
    return _em7180.getSensorStatus() ? false : true;
}

void EM7180_Master::getFullScaleRanges(uint8_t& accFs, uint16_t& gyroFs, uint16_t& magFs)
{
    _em7180.getFullScaleRanges(accFs, gyroFs, magFs);
}

bool EM7180_Master::algorithmStatusStandby(void)
{
    return algorithmStatus(0x01);
}

bool EM7180_Master::algorithmStatusSlow(void)
{
    return algorithmStatus(0x02);
}

bool EM7180_Master::algorithmStatusStillness(void)
{
    return algorithmStatus(0x04);
}

bool EM7180_Master::algorithmStatusMagCalibrationCompleted(void)
{
    return algorithmStatus(0x08);
}

bool EM7180_Master::algorithmStatusMagneticAnomalyDetected(void)
{
    return algorithmStatus(0x10);
}

bool EM7180_Master::algorithmStatusUnreliableData(void)
{
    return algorithmStatus(0x20);
}

bool EM7180_Master::runStatusNormal(void)
{
    return _em7180.getRunStatus() & 0x01;
}

void EM7180_Master::checkEventStatus(void)
{
    // Check event status register, way to check data ready by checkEventStatusing rather than interrupt
    _eventStatus = _em7180.getEventStatus(); // reading clears the register

}

bool EM7180_Master::gotError(void)
{
    if (_eventStatus & 0x02) {

        return true;
    }

    return false;
}

bool EM7180_Master::gotQuaternion(void)
{
    return _eventStatus & 0x04;
}

bool EM7180_Master::gotMagnetometer(void)
{
    return _eventStatus & 0x08;
}

bool EM7180_Master::gotAccelerometer(void)
{
    return _eventStatus & 0x10;
}

bool EM7180_Master::gotGyrometer(void)
{
    return _eventStatus & 0x20;
}

bool EM7180_Master::gotBarometer(void)
{
    return _eventStatus & 0x40;
}

void EM7180_Master::readQuaternion(float & qw, float & qx, float & qy, float &qz)
{
    _em7180.readQuaternion(qw, qx, qy, qz);
}

void EM7180_Master::readAccelerometer(int16_t & ax, int16_t & ay, int16_t & az)
{
    _em7180.readAccelerometer(ax, ay, az);
}

void EM7180_Master::readGyrometer(int16_t & gx, int16_t & gy, int16_t & gz)
{
    _em7180.readGyrometer(gx, gy, gz);
}

void EM7180_Master::readMagnetometer(int16_t & mx, int16_t & my, int16_t & mz)
{
    _em7180.readMagnetometer(mx, my, mz);
}


void EM7180_Master::readBarometer(float & pressure, float & temperature)
{
    _em7180.readBarometer(pressure, temperature);
}

uint8_t EM7180_Master::getActualMagRate()
{
    return _em7180.getActualMagRate();
}

uint16_t EM7180_Master::getActualAccelRate()
{
    return 10*_em7180.getActualAccelRate();
}

uint16_t EM7180_Master::getActualGyroRate()
{
    return 10*_em7180.getActualGyroRate();
}

uint8_t EM7180_Master::getActualBaroRate()
{
    return _em7180.getActualBaroRate();
}

uint8_t EM7180_Master::getActualTempRate()
{
    return _em7180.getActualTempRate();
}

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

#include <Arduino.h>

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

#include "EM7180.h"

#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42

// EM7180 SENtral register map
// see http://www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf
//
#define EM7180_QX                 0x00  // this is a 32-bit normalized floating point number read from registers 0x00-03
#define EM7180_QY                 0x04  // this is a 32-bit normalized floating point number read from registers 0x04-07
#define EM7180_QZ                 0x08  // this is a 32-bit normalized floating point number read from registers 0x08-0B
#define EM7180_QW                 0x0C  // this is a 32-bit normalized floating point number read from registers 0x0C-0F
#define EM7180_QTIME              0x10  // this is a 16-bit unsigned integer read from registers 0x10-11
#define EM7180_MX                 0x12  // int16_t from registers 0x12-13
#define EM7180_MY                 0x14  // int16_t from registers 0x14-15
#define EM7180_MZ                 0x16  // int16_t from registers 0x16-17
#define EM7180_MTIME              0x18  // uint16_t from registers 0x18-19
#define EM7180_AX                 0x1A  // int16_t from registers 0x1A-1B
#define EM7180_AY                 0x1C  // int16_t from registers 0x1C-1D
#define EM7180_AZ                 0x1E  // int16_t from registers 0x1E-1F
#define EM7180_ATIME              0x20  // uint16_t from registers 0x20-21
#define EM7180_GX                 0x22  // int16_t from registers 0x22-23
#define EM7180_GY                 0x24  // int16_t from registers 0x24-25
#define EM7180_GZ                 0x26  // int16_t from registers 0x26-27
#define EM7180_GTIME              0x28  // uint16_t from registers 0x28-29
#define EM7180_Baro               0x2A  // start of two-byte MS5637 pressure data, 16-bit signed interger
#define EM7180_BaroTIME           0x2C  // start of two-byte MS5637 pressure timestamp, 16-bit unsigned
#define EM7180_Temp               0x2E  // start of two-byte MS5637 temperature data, 16-bit signed interger
#define EM7180_TempTIME           0x30  // start of two-byte MS5637 temperature timestamp, 16-bit unsigned
#define EM7180_QRateDivisor       0x32  // uint8_t 
#define EM7180_EnableEvents       0x33
#define EM7180_HostControl        0x34
#define EM7180_EventStatus        0x35
#define EM7180_SensorStatus       0x36
#define EM7180_SentralStatus      0x37
#define EM7180_AlgorithmStatus    0x38
#define EM7180_FeatureFlags       0x39
#define EM7180_ParamAcknowledge   0x3A
#define EM7180_SavedParamByte0    0x3B
#define EM7180_SavedParamByte1    0x3C
#define EM7180_SavedParamByte2    0x3D
#define EM7180_SavedParamByte3    0x3E
#define EM7180_ActualMagRate      0x45
#define EM7180_ActualAccelRate    0x46
#define EM7180_ActualGyroRate     0x47
#define EM7180_ActualBaroRate     0x48
#define EM7180_ActualTempRate     0x49
#define EM7180_ErrorRegister      0x50
#define EM7180_AlgorithmControl   0x54
#define EM7180_MagRate            0x55
#define EM7180_AccelRate          0x56
#define EM7180_GyroRate           0x57
#define EM7180_BaroRate           0x58
#define EM7180_TempRate           0x59
#define EM7180_LoadParamByte0     0x60
#define EM7180_LoadParamByte1     0x61
#define EM7180_LoadParamByte2     0x62
#define EM7180_LoadParamByte3     0x63
#define EM7180_ParamRequest       0x64
#define EM7180_ROMVersion1        0x70
#define EM7180_ROMVersion2        0x71
#define EM7180_RAMVersion1        0x72
#define EM7180_RAMVersion2        0x73
#define EM7180_ProductID          0x90
#define EM7180_RevisionID         0x91
#define EM7180_RunStatus          0x92
#define EM7180_UploadAddress      0x94 // uint16_t registers 0x94 (MSB)-5(LSB)
#define EM7180_UploadData         0x96  
#define EM7180_CRCHost            0x97  // uint32_t from registers 0x97-9A
#define EM7180_ResetRequest       0x9B   
#define EM7180_PassThruStatus     0x9E   
#define EM7180_PassThruControl    0xA0
#define EM7180_ACC_LPF_BW         0x5B  //Register GP36
#define EM7180_GYRO_LPF_BW        0x5C  //Register GP37
#define EM7180_BARO_LPF_BW        0x5D  //Register GP38

#define EM7180_ADDRESS           0x28   // Address of the EM7180 SENtral sensor hub
#define M24512DFM_DATA_ADDRESS   0x50   // Address of the 500 page M24512DRC EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
#define M24512DFM_IDPAGE_ADDRESS 0x58   // Address of the single M24512DRC lockable EEPROM ID page
#define AK8963_ADDRESS           0x0C   // Address of magnetometer
#define BMP280_ADDRESS           0x76   // Address of BMP280 altimeter when ADO = 0

void _EM7180::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t _EM7180::readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data	 
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);	                 // Put slave register address in Tx buffer
    Wire.endTransmission(NOSTOP);            // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void _EM7180::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(NOSTOP);      // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

float EM7180::uint32_reg_to_float (uint8_t *buf)
{
    union {
        uint32_t ui32;
        float f;
    } u;

    u.ui32 =     (((uint32_t)buf[0]) +
            (((uint32_t)buf[1]) <<  8) +
            (((uint32_t)buf[2]) << 16) +
            (((uint32_t)buf[3]) << 24));
    return u.f;
}

void EM7180::setIntegerParam(uint8_t param, uint32_t param_val) {
    uint8_t bytes[4], STAT;
    bytes[0] = param_val & (0xFF);
    bytes[1] = (param_val >> 8) & (0xFF);
    bytes[2] = (param_val >> 16) & (0xFF);
    bytes[3] = (param_val >> 24) & (0xFF);
    param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==param)) {
        STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180::setMagAccFs(uint16_t mag_fs, uint16_t acc_fs) {
    uint8_t bytes[4], STAT;
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Mag LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Mag MSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Acc LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Acc MSB
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCA)) {
        STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180::setGyroFs(uint16_t gyro_fs) {
    uint8_t bytes[4], STAT;
    bytes[0] = gyro_fs & (0xFF);
    bytes[1] = (gyro_fs >> 8) & (0xFF);
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCB)) {
        STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

void _EM7180::M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(device_address);   // Initialize the Tx buffer
    Wire.write(data_address1);                     // Put slave register address in Tx buffer
    Wire.write(data_address2);                     // Put slave register address in Tx buffer
    Wire.endTransmission(NOSTOP);             // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(device_address, (size_t) count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); }                // Put read results in the Rx buffer
}

bool _EM7180::hasFeature(uint8_t features)
{
    return features & readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
}

bool EM7180::algorithmStatus(uint8_t status)
{
    return readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus) & status;
}

static volatile bool newData;

void interruptHandler()
{
    newData = true;
}

// public methods ========================================================================================================

bool _EM7180::begin(void)
{
    errorStatus = 0;

    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    for (int attempts=0; attempts<10; ++attempts) {
        if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01) {
            if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01) { }
            if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02) { }
            if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04) {
                errorStatus = 0xB0;
                return false;
            }
            if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08) { }
            if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10) {
                errorStatus = 0xB0;
                return false;
            }
            break;
        }
        writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
        delay(500);  
    }


    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04) {
        errorStatus = 0xB0;
        return false;
    }

    return true;
}

const char * _EM7180::getErrorString(void)
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


bool _EM7180::hasBaro(void)
{
    return hasFeature(0x01);
}

bool _EM7180::hasHumidity(void)
{
    return hasFeature(0x02);
}

bool _EM7180::hasTemperature(void)
{
    return hasFeature(0x04);
}

bool _EM7180::hasCustom1(void)
{
    return hasFeature(0x08);
}

bool _EM7180::hasCustom2(void)
{
    return hasFeature(0x10);
}

bool _EM7180::hasCustom3(void)
{
    return hasFeature(0x20);
}

uint8_t _EM7180::getProductId(void) 
{
    return readByte(EM7180_ADDRESS, EM7180_ProductID);
}

uint8_t _EM7180::getRevisionId(void) 
{
    return readByte(EM7180_ADDRESS, EM7180_RevisionID);
}

uint16_t _EM7180::getRamVersion(void)
{
    uint16_t ram1 = readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
    uint16_t ram2 = readByte(EM7180_ADDRESS, EM7180_RAMVersion2);

    return ram1 << 8 | ram2;
}

uint16_t _EM7180::getRomVersion(void)
{
    uint16_t rom1 = readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
    uint16_t rom2 = readByte(EM7180_ADDRESS, EM7180_ROMVersion2);

    return rom1 << 8 | rom2;
}

bool EM7180_Passthru::begin(void)
{
    // Do generic intialization
    if (!_EM7180::begin()) return false;

    // First put SENtral in standby mode
    uint8_t c = readByte(EM7180_ADDRESS, EM7180_AlgorithmControl);
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, c | 0x01);
    // Verify standby status
    // if(readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus) & 0x01) {
    // Place SENtral in pass-through mode
    writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x01); 
    if(readByte(EM7180_ADDRESS, EM7180_PassThruStatus) & 0x01) {
    }
    else {
        errorStatus = 0x90;
        return false;
    }

    uint8_t data[128];
    M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x00, 0x00, 128, data);
    if (data[0] != 0x2A || data[1] != 0x65) {
        errorStatus = 0xA0;
        return false;
    }

    // Success
    return true;
}

bool EM7180::begin(uint8_t ares, uint16_t gres, uint16_t mres, int8_t interruptPin)
{
    // Fail immediately if unable to upload EEPROM
    if (!_EM7180::begin()) return false;

    // Enter EM7180 initialized state
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // Force initialize
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers

    // Setup LPF bandwidth (BEFORE setting ODR's)
    writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, 0x03);  // 41Hz
    writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, 0x03); // 41Hz

    // Set accel/gyro/mage desired ODR rates
    writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02);    // One half of gyro rate
    writeByte(EM7180_ADDRESS, EM7180_MagRate, 0x64);         // 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_AccelRate, 0x14);       // 200 Hz (because units are 10Hz)
    writeByte(EM7180_ADDRESS, EM7180_GyroRate, 0x14);        // 200 Hz (because units are 10 Hz)
    writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | 0x32); // 0x80 = enable bit; 0x32 = 50 Hz, yielding 25 Hz response

    // Configure operating mode
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data

    // Enable interrupt to host upon certain events
    // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);

    // Enable EM7180 run mode
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
    delay(100);

    // Disable stillness mode
    setIntegerParam (0x49, 0x00);

    // Write desired sensor full scale ranges to the EM7180
    setMagAccFs(mres, ares);
    setGyroFs(gres); 

    if (interruptPin >= 0) {

        // Set up the interrupt pin: active high, push-pull
        pinMode(interruptPin, INPUT);
        attachInterrupt(interruptPin, interruptHandler, RISING);  // define interrupt for INT pin output of EM7180

        // Check event status register to clear the EM7180 interrupt before the main loop
        readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register and interrupt

        newData = false;
    }

    // Success
    return readByte(EM7180_ADDRESS, EM7180_SensorStatus) ? false : true;
}

void EM7180::getFullScaleRanges(uint8_t& accFs, uint16_t& gyroFs, uint16_t& magFs)
{
    uint8_t param[4];

    // Read sensor new FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read  parameter 74
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    byte param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4A)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    magFs = ((uint16_t)(param[1]<<8) | param[0]);
    accFs = ((uint8_t)(param[3]<<8) | param[2]);
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4B)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    gyroFs = ((uint16_t)(param[1]<<8) | param[0]);
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm
}

bool EM7180::algorithmStatusStandby(void)
{
    return algorithmStatus(0x01);
}

bool EM7180::algorithmStatusSlow(void)
{
    return algorithmStatus(0x02);
}

bool EM7180::algorithmStatusStillness(void)
{
    return algorithmStatus(0x04);
}

bool EM7180::algorithmStatusMagCalibrationCompleted(void)
{
    return algorithmStatus(0x08);
}

bool EM7180::algorithmStatusMagneticAnomalyDetected(void)
{
    return algorithmStatus(0x10);
}

bool EM7180::algorithmStatusUnreliableData(void)
{
    return algorithmStatus(0x20);
}

bool EM7180::runStatusNormal()
{
    return (readByte(EM7180_ADDRESS, EM7180_RunStatus) & 0x01);
}

void EM7180::checkEventStatus(void)
{
    // Check event status register, way to check data ready by checkEventStatusing rather than interrupt
    eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

}

bool EM7180::gotError(void)
{
    if (eventStatus & 0x02) {

        return true;
    }

    return false;
}

bool EM7180::gotQuaternions(void)
{
    return eventStatus & 0x04;
}

bool EM7180::gotMagnetometer(void)
{
    return eventStatus & 0x08;
}

bool EM7180::gotAccelerometer(void)
{
    return eventStatus & 0x10;
}

bool EM7180::gotGyrometer(void)
{
    return eventStatus & 0x20;
}

bool EM7180::gotBarometer(void)
{
    return eventStatus & 0x40;
}

void EM7180::readQuaternions(float q[4])
{
    uint8_t rawData[16];  // x/y/z quaternion register data stored here
    readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]);       // Read the sixteen raw data registers into data array
    q[0] = uint32_reg_to_float (&rawData[0]);
    q[1] = uint32_reg_to_float (&rawData[4]);
    q[2] = uint32_reg_to_float (&rawData[8]);
    q[3] = uint32_reg_to_float (&rawData[12]);  // SENtral stores quats as qx, qy, qz, q0!

}

void EM7180::readMagnetometer(int16_t mag[3])
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    mag[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    mag[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    mag[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void EM7180::readAccelerometer(int16_t accel[3])
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]);       // Read the six raw data registers into data array
    accel[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
    accel[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    accel[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void EM7180::readGyrometer(int16_t gyro[3])
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gyro[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    gyro[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    gyro[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void EM7180::readBarometer(float & pressure, float & temperature)
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here

    readBytes(EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    int16_t rawPressure =  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    pressure = (float)rawPressure *.01f + 1013.25f; // pressure in millibars

    // get BMP280 temperature
    readBytes(EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    int16_t rawTemperature =  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value

    temperature = (float) rawTemperature*0.01;  // temperature in degrees C
}

uint8_t EM7180::getActualMagRate()
{
    return readByte(EM7180_ADDRESS, EM7180_ActualMagRate);
}

uint16_t EM7180::getActualAccelRate()
{
    return 10*readByte(EM7180_ADDRESS, EM7180_ActualAccelRate);
}

uint16_t EM7180::getActualGyroRate()
{
    return 10*readByte(EM7180_ADDRESS, EM7180_ActualGyroRate);
}

uint8_t EM7180::getActualBaroRate()
{
    return readByte(EM7180_ADDRESS, EM7180_ActualBaroRate);
}

uint8_t EM7180::getActualTempRate()
{
    return readByte(EM7180_ADDRESS, EM7180_ActualTempRate);
}

bool EM7180::gotInterrupt(void)
{
    return newData;
}


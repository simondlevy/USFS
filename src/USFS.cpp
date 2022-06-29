/* 
   USFS source

   Copyright (C) 2022 TleraCorp && Simon D. Levy

   Adapted from

https://github.com/kriswiner/USFS_SENtral_sensor_hub/tree/master/WarmStartandAccelCal

This file is part of USFS.

USFS is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

USFS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with USFS.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <Arduino.h>
#include <Wire.h>

#include "USFS.h"

static const uint8_t EM7180_ADDRESS= 0x28;

// EM7180 SENtral register map
// see http:;//www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf

// this is a 32-bit normalized floating point number read from registers = 0x00-03
static const uint8_t EM7180_QX = 0x00;

// this is a 32-bit normalized floating point number read from registers = 0x04-07
static const uint8_t EM7180_QY = 0x04;

// this is a 32-bit normalized floating point number read from registers = 0x08-0B
static const uint8_t EM7180_QZ = 0x08;

// this is a 32-bit normalized floating point number read from registers = 0x0C-0F
static const uint8_t EM7180_QW = 0x0C;

// this is a 16-bit unsigned integer read from registers = 0x10-11
static const uint8_t EM7180_QTIME= 0x10;

static const uint8_t EM7180_MX = 0x12;// int16_t from registers = 0x12-13
static const uint8_t EM7180_MY = 0x14;// int16_t from registers = 0x14-15
static const uint8_t EM7180_MZ = 0x16;// int16_t from registers = 0x16-17
static const uint8_t EM7180_MTIME= 0x18;// uint16_t from registers = 0x18-19
static const uint8_t EM7180_AX = 0x1A;// int16_t from registers = 0x1A-1B
static const uint8_t EM7180_AY = 0x1C;// int16_t from registers = 0x1C-1D
static const uint8_t EM7180_AZ = 0x1E;// int16_t from registers = 0x1E-1F
static const uint8_t EM7180_ATIME= 0x20;// uint16_t from registers = 0x20-21
static const uint8_t EM7180_GX = 0x22;// int16_t from registers = 0x22-23
static const uint8_t EM7180_GY = 0x24;// int16_t from registers = 0x24-25
static const uint8_t EM7180_GZ = 0x26;// int16_t from registers = 0x26-27
static const uint8_t EM7180_GTIME= 0x28;// uint16_t from registers = 0x28-29

// start of two-byte MS5637 pressure data, 16-bit signed interger
static const uint8_t EM7180_Baro = 0x2A;

// start of two-byte MS5637 pressure timestamp, 16-bit unsigned
static const uint8_t EM7180_BaroTIME = 0x2C;

// start of two-byte MS5637 temperature data, 16-bit signed interger
static const uint8_t EM7180_Temp = 0x2E;

// start of two-byte MS5637 temperature timestamp, 16-bit unsigned
static const uint8_t EM7180_TempTIME = 0x30;

static const uint8_t EM7180_QRateDivisor = 0x32;
static const uint8_t EM7180_EnableEvents = 0x33;
static const uint8_t EM7180_HostControl= 0x34;
static const uint8_t EM7180_EventStatus= 0x35;
static const uint8_t EM7180_SensorStatus = 0x36;
static const uint8_t EM7180_SentralStatus= 0x37;
static const uint8_t EM7180_AlgorithmStatus= 0x38;
static const uint8_t EM7180_FeatureFlags = 0x39;
static const uint8_t EM7180_ParamAcknowledge = 0x3A;
static const uint8_t EM7180_SavedParamByte0= 0x3B;
static const uint8_t EM7180_SavedParamByte1= 0x3C;
static const uint8_t EM7180_SavedParamByte2= 0x3D;
static const uint8_t EM7180_SavedParamByte3= 0x3E;
static const uint8_t EM7180_ActualMagRate= 0x45;
static const uint8_t EM7180_ActualAccelRate= 0x46;
static const uint8_t EM7180_ActualGyroRate = 0x47;
static const uint8_t EM7180_ActualBaroRate = 0x48;
static const uint8_t EM7180_ActualTempRate = 0x49;
static const uint8_t EM7180_ErrorRegister= 0x50;
static const uint8_t EM7180_AlgorithmControl = 0x54;
static const uint8_t EM7180_MagRate= 0x55;
static const uint8_t EM7180_AccelRate= 0x56;
static const uint8_t EM7180_GyroRate = 0x57;
static const uint8_t EM7180_BaroRate = 0x58;
static const uint8_t EM7180_TempRate = 0x59;
static const uint8_t EM7180_LoadParamByte0 = 0x60;
static const uint8_t EM7180_LoadParamByte1 = 0x61;
static const uint8_t EM7180_LoadParamByte2 = 0x62;
static const uint8_t EM7180_LoadParamByte3 = 0x63;
static const uint8_t EM7180_ParamRequest = 0x64;
static const uint8_t EM7180_ROMVersion1= 0x70;
static const uint8_t EM7180_ROMVersion2= 0x71;
static const uint8_t EM7180_RAMVersion1= 0x72;
static const uint8_t EM7180_RAMVersion2= 0x73;
static const uint8_t EM7180_ProductID= 0x90;
static const uint8_t EM7180_RevisionID = 0x91;
static const uint8_t EM7180_RunStatus= 0x92;
static const uint8_t EM7180_UploadAddress= 0x94 ;// uint16_t registers = 0x94 (MSB)-5(LSB);
static const uint8_t EM7180_UploadData = 0x96;
static const uint8_t EM7180_CRCHost= 0x97;// uint32_t from registers = 0x97-9A
static const uint8_t EM7180_ResetRequest = 0x9B ;
static const uint8_t EM7180_PassThruStatus = 0x9E ;
static const uint8_t EM7180_PassThruControl= 0xA0;
static const uint8_t EM7180_ACC_LPF_BW = 0x5B;//Register GP36;
static const uint8_t EM7180_GYRO_LPF_BW= 0x5C;//Register GP37
static const uint8_t EM7180_BARO_LPF_BW= 0x5D;//Register GP38


static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) 
{
    uint8_t temp[2];
    temp[0] = subAddress;
    temp[1] = data;
    Wire.transfer(address, &temp[0], 2, NULL, 0); 
}

static uint8_t readByte(uint8_t address, uint8_t subAddress) 
{
    uint8_t temp[1];
    Wire.transfer(address, &subAddress, 1, &temp[0], 1);
    return temp[0];
}

static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) 
{
    Wire.transfer(address, &subAddress, 1, dest, count); 
}

static float uint32_reg_to_float (uint8_t *buf)
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

static void EM7180_set_integer_param (uint8_t param, uint32_t param_val) 
{
    uint8_t bytes[4] = {};
    bytes[0] = param_val & (0xFF);
    bytes[1] = (param_val >> 8) & (0xFF);
    bytes[2] = (param_val >> 16) & (0xFF);
    bytes[3] = (param_val >> 24) & (0xFF);

    // Parameter is the decimal value with the MSB set high to indicate a
    // paramter write processs
    param = param | 0x80; 

    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); // Param LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); // Param MSB
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

    // Request parameter transfer procedure
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); 

    // Check the parameter acknowledge register and loop until the result
    // matches parameter request byte
    uint8_t status = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); 

    while(!(status==param)) {
        status = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }

    // Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); 

    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

static void EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs) 
{
    uint8_t bytes[4] = {};
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); // Mag LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); // Mag MSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); // Acc LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); // Acc MSB

    // Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a
    // paramter write processs
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA); 

    // Request parameter transfer procedure
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); 

    // Check the parameter acknowledge register and loop until the result
    // matches parameter request byte
    uint8_t status = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); 
    while(!(status==0xCA)) {
        status = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }

    // Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); 

    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

static void EM7180_set_gyro_FS (uint16_t gyro_fs) 
{
    uint8_t bytes[4] = {};
    bytes[0] = gyro_fs & (0xFF);
    bytes[1] = (gyro_fs >> 8) & (0xFF);
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused

    // Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a
    // paramter write processs
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB); 

    //Request parameter transfer procedure
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); 

    // Check the parameter acknowledge register and loop until the result
    // matches parameter request byte
    uint8_t status = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); 
    while(!(status==0xCB)) {
        status = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }

    // Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); 

    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

// ============================================================================

uint8_t usfsCheckErrors(){
    uint8_t c = readByte(EM7180_ADDRESS, EM7180_ErrorRegister); // check error register
    return c;
}

uint8_t usfsCheckStatus(){
    // Check event status register, way to check data ready by polling rather than interrupt
    uint8_t c = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register and interrupt
    return c;
}

void usfsReportChipId()
{
    // Read SENtral device information
    uint16_t ROM1 = readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
    uint16_t ROM2 = readByte(EM7180_ADDRESS, EM7180_ROMVersion2);
    Serial.print("EM7180 ROM Version: 0x");
    Serial.print(ROM1, HEX);
    Serial.println(ROM2, HEX);
    Serial.println("Should be: 0xE609");
    uint16_t RAM1 = readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
    uint16_t RAM2 = readByte(EM7180_ADDRESS, EM7180_RAMVersion2);
    Serial.print("EM7180 RAM Version: 0x");
    Serial.print(RAM1);
    Serial.println(RAM2);
    uint8_t PID = readByte(EM7180_ADDRESS, EM7180_ProductID);
    Serial.print("EM7180 ProductID: 0x");
    Serial.print(PID, HEX);
    Serial.println(" Should be: 0x80");
    uint8_t RID = readByte(EM7180_ADDRESS, EM7180_RevisionID);
    Serial.print("EM7180 RevisionID: 0x");
    Serial.print(RID, HEX);
    Serial.println(" Should be: 0x02");
}


void usfsBegin(
        uint8_t accBW,
        uint8_t gyroBW,
        uint16_t accFS,
        uint16_t gyroFS,
        uint16_t magFS,
        uint8_t QRtDiv,
        uint8_t magRt,
        uint8_t accRt,
        uint8_t gyroRt,
        uint8_t baroRt,
        bool verbose)
{
    uint16_t EM7180_mag_fs,
             EM7180_acc_fs,
             EM7180_gyro_fs; // EM7180 sensor full scale ranges

    uint8_t param[4];      

    // Enter EM7180 initialized state
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); 

    // Make sure pass through mode is off
    writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); 
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // Force initialize

    // Set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); 

    //Setup LPF bandwidth (BEFORE setting ODR's)
    writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, accBW);   // accBW = 3 = 41Hz
    writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, gyroBW); // gyroBW = 3 = 41Hz

    // Set accel/gyro/mag desired ODR rates
    writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, QRtDiv); // quat rate = gyroRt/(1 QRTDiv)
    writeByte(EM7180_ADDRESS, EM7180_MagRate, magRt); // 0x64 = 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_AccelRate, accRt); // 200/10 Hz, 0x14 = 200 Hz
    writeByte(EM7180_ADDRESS, EM7180_GyroRate, gyroRt); // 200/10 Hz, 0x14 = 200 Hz

    // Set enable bit and set Baro rate to 25 Hz, rate = baroRt/2, 0x32 = 25 Hz
    writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | baroRt);  

    // Configure operating mode
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data

    // Enable interrupt to host upon certain events choose host interrupts when
    // any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02),
    // or the SENtral needs to be reset(0x01)
    writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);

    // Enable EM7180 run mode
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
    delay(100);

    if (verbose) {
        Serial.println("Beginning Parameter Adjustments");
    }

    // Read sensor default FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read parameter 74

    // Request parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); 
    byte param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4A)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
    EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);

    if (verbose) {
        Serial.print("Magnetometer Default Full Scale Range: +/-");
        Serial.print(EM7180_mag_fs);
        Serial.println("uT");
        Serial.print("Accelerometer Default Full Scale Range: +/-");
        Serial.print(EM7180_acc_fs);
        Serial.println("g");
    }

    // Request to read  parameter 75
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); 

    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4B)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
    Serial.print("Gyroscope Default Full Scale Range: +/-");
    Serial.print(EM7180_gyro_fs);
    Serial.println("dps");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

    //Disable stillness mode for balancing robot application
    EM7180_set_integer_param (0x49, 0x00);

    //Write desired sensor full scale ranges to the EM7180
    EM7180_set_mag_acc_FS (magFS, accFS); // 1000 uT == 0x3E8, 8 g == 0x08
    EM7180_set_gyro_FS (gyroFS); // 2000 dps == 0x7D0

    // Read sensor new FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read  parameter 74

    // Request parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); 
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4A)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
    EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);

    if (verbose) {
        Serial.print("Magnetometer New Full Scale Range: +/-");
        Serial.print(EM7180_mag_fs);
        Serial.println("uT");
        Serial.print("Accelerometer New Full Scale Range: +/-");
        Serial.print(EM7180_acc_fs);
        Serial.println("g");
    }

    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4B)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);

    if (verbose) {
        Serial.print("Gyroscope New Full Scale Range: +/-");
        Serial.print(EM7180_gyro_fs);
        Serial.println("dps");
    }

    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

    // Read EM7180 status
    uint8_t runStatus = readByte(EM7180_ADDRESS, EM7180_RunStatus);
    if (runStatus & 0x01) Serial.println(" EM7180 run status = normal mode");

    uint8_t algoStatus = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);

    if (verbose) {
        if (algoStatus & 0x01) Serial.println(" EM7180 standby status");
        if (algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
        if (algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
        if (algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
        if (algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
        if (algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");
    }

    uint8_t passthruStatus = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);

    if (passthruStatus & 0x01) Serial.print(" EM7180 in passthru mode!");

    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus);

    if (verbose) {
        if (eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
        if (eventStatus & 0x02) Serial.println(" EM7180 Error");
        if (eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
        if (eventStatus & 0x08) Serial.println(" EM7180 new mag result");
        if (eventStatus & 0x10) Serial.println(" EM7180 new accel result");
        if (eventStatus & 0x20) Serial.println(" EM7180 new gyro result"); 
    }

    delay(1000); // give some time to read the screen

    // Check sensor status
    uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);
    Serial.print(" EM7180 sensor status = ");
    Serial.println(sensorStatus);
    if (sensorStatus & 0x01) Serial.print("Magnetometer not acknowledging!");
    if (sensorStatus & 0x02) Serial.print("Accelerometer not acknowledging!");
    if (sensorStatus & 0x04) Serial.print("Gyro not acknowledging!");
    if (sensorStatus & 0x10) Serial.print("Magnetometer ID not recognized!");
    if (sensorStatus & 0x20) Serial.print("Accelerometer ID not recognized!");
    if (sensorStatus & 0x40) Serial.print("Gyro ID not recognized!");

    if (verbose) {
        Serial.print("Actual MagRate = ");
        Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualMagRate));
        Serial.println(" Hz"); 
        Serial.print("Actual AccelRate = ");
        Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualAccelRate));
        Serial.println(" Hz"); 
        Serial.print("Actual GyroRate = ");
        Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualGyroRate));
        Serial.println(" Hz"); 
        Serial.print("Actual BaroRate = ");
        Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualBaroRate));
        Serial.println(" Hz"); 
    }
}


void usfsLoadFirmware()
{
    // Check which sensors can be detected by the EM7180
    uint8_t featureflag = readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
    if (featureflag & 0x01)  {
        Serial.println("A barometer is installed");
    }
    if (featureflag & 0x02)  {
        Serial.println("A humidity sensor is installed");
    }
    if (featureflag & 0x04)  {
        Serial.println("A temperature sensor is installed");
    }
    if (featureflag & 0x08)  {
        Serial.println("A custom sensor is installed");
    }
    if (featureflag & 0x10)  {
        Serial.println("A second custom sensor is installed");
    }
    if (featureflag & 0x20)  {
        Serial.println("A third custom sensor is installed");
    }

    delay(1000); // give some time to read the screen

    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    byte status = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  {
        Serial.println("EEPROM detected on the sensor bus!");
    }
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  {
        Serial.println("EEPROM uploaded config file!");
    }
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  {
        Serial.println("EEPROM CRC incorrect!");
    }
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  {
        Serial.println("EM7180 in initialized state!");
    }
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  {
        Serial.println("No EEPROM detected!");
    }

    int count = 0;
    while(!status) {
        writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
        delay(500);  
        count++;  
        status = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
        if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  {
            Serial.println("EEPROM detected on the sensor bus!");
        }
        if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  {
            Serial.println("EEPROM uploaded config file!");
        }
        if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  {
            Serial.println("EEPROM CRC incorrect!");
        }
        if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  {
            Serial.println("EM7180 in initialized state!");
        }
        if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  {
            Serial.println("No EEPROM detected!");
        }
        if (count > 10) break;
    }

    if (!(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)) {
        Serial.println("EEPROM upload successful!");
    }
}

void usfsReadAccelerometer(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here

    // Read the six raw data registers into data array
    readBytes(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]);       

    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

int16_t usfsReadBarometer()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here

    // Read the two raw data registers sequentially into data array
    readBytes(EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0]);  

    // Turn the MSB and LSB into a signed 16-bit value
    return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   
}

void usfsReadGyrometer(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here

    // Read the six raw data registers sequentially into data array
    readBytes(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]);  

    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   

    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void usfsreadMagnetometer(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here

    // Read the six raw data registers sequentially into data array
    readBytes(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]);  

    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   

    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void usfsReadQuaternion(float * destination)
{
    uint8_t rawData[16];  // x/y/z quaternion register data stored here

    // Read the sixteen raw data registers into data array
    readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]); 

    // SENtral stores quats as qx, qy, qz, q0!
    destination[1] = uint32_reg_to_float (&rawData[0]);
    destination[2] = uint32_reg_to_float (&rawData[4]);
    destination[3] = uint32_reg_to_float (&rawData[8]);
    destination[0] = uint32_reg_to_float (&rawData[12]);   
}

int16_t usfsReadTemperature()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here

    // Read the two raw data registers sequentially into data array
    readBytes(EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0]);  

    // Turn the MSB and LSB into a signed 16-bit value
    return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   
}

bool usfsEventStatusIsError(uint8_t status)
{
    return status & 0x02;
}

bool usfsEventStatusIsAccelerometer(uint8_t status)
{
    return status & 0x10;
}

bool usfsEventStatusIsGyrometer(uint8_t status)
{
    return status & 0x20;
}

bool usfsEventStatusIsMagnetometer(uint8_t status)
{
    return status & 0x08;
}

bool usfsEventStatusIsQuaternion(uint8_t status)
{
    return status & 0x04;
}

bool usfsEventStatusIsBarometer(uint8_t status)
{
    return status & 0x40;
}

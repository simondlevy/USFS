/* 
   USFS source

   Copyright (C) 2022 TleraCorp && Simon D. Levy

   Adapted from

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


#include "USFS.h"

static const uint8_t ADDRESS= 0x28;

// EM7180 SENtral register map
// see http:;//www.emdeveloper.com/downloads/7180/EMSentral_Register_Map_v1_3.pdf

// this is a 32-bit normalized floating point number read from registers = 0x00-03
static const uint8_t QX = 0x00;

// this is a 32-bit normalized floating point number read from registers = 0x04-07
static const uint8_t QY = 0x04;

// this is a 32-bit normalized floating point number read from registers = 0x08-0B
static const uint8_t QZ = 0x08;

// this is a 32-bit normalized floating point number read from registers = 0x0C-0F
static const uint8_t QW = 0x0C;

// this is a 16-bit unsigned integer read from registers = 0x10-11
static const uint8_t QTIME= 0x10;

static const uint8_t MX = 0x12;// int16_t from registers = 0x12-13
static const uint8_t MY = 0x14;// int16_t from registers = 0x14-15
static const uint8_t MZ = 0x16;// int16_t from registers = 0x16-17
static const uint8_t MTIME = 0x18;// uint16_t from registers = 0x18-19
static const uint8_t AX = 0x1A;// int16_t from registers = 0x1A-1B
static const uint8_t AY = 0x1C;// int16_t from registers = 0x1C-1D
static const uint8_t AZ = 0x1E;// int16_t from registers = 0x1E-1F
static const uint8_t ATIME= 0x20;// uint16_t from registers = 0x20-21
static const uint8_t GX = 0x22;// int16_t from registers = 0x22-23
static const uint8_t GY = 0x24;// int16_t from registers = 0x24-25
static const uint8_t GZ = 0x26;// int16_t from registers = 0x26-27
static const uint8_t GTIME= 0x28;// uint16_t from registers = 0x28-29

// start of two-byte MS5637 pressure data, 16-bit signed interger
static const uint8_t Baro = 0x2A;

// start of two-byte MS5637 pressure timestamp, 16-bit unsigned
static const uint8_t BaroTIME = 0x2C;

// start of two-byte MS5637 temperature data, 16-bit signed interger
static const uint8_t Temp = 0x2E;

// start of two-byte MS5637 temperature timestamp, 16-bit unsigned
static const uint8_t TempTIME = 0x30;

static const uint8_t QRateDivisor = 0x32;
static const uint8_t EnableEvents = 0x33;
static const uint8_t HostControl= 0x34;
static const uint8_t EventStatus= 0x35;
static const uint8_t SensorStatus = 0x36;
static const uint8_t SentralStatus= 0x37;
static const uint8_t AlgorithmStatus= 0x38;
static const uint8_t FeatureFlags = 0x39;
static const uint8_t ParamAcknowledge = 0x3A;
static const uint8_t SavedParamByte0= 0x3B;
static const uint8_t SavedParamByte1= 0x3C;
static const uint8_t SavedParamByte2= 0x3D;
static const uint8_t SavedParamByte3= 0x3E;
static const uint8_t ActualMagRate= 0x45;
static const uint8_t ActualAccelRate= 0x46;
static const uint8_t ActualGyroRate = 0x47;
static const uint8_t ActualBaroRate = 0x48;
static const uint8_t ActualTempRate = 0x49;
static const uint8_t ErrorRegister= 0x50;
static const uint8_t AlgorithmControl = 0x54;
static const uint8_t MagRate= 0x55;
static const uint8_t AccelRate= 0x56;
static const uint8_t GyroRate = 0x57;
static const uint8_t BaroRate = 0x58;
static const uint8_t TempRate = 0x59;
static const uint8_t LoadParamByte0 = 0x60;
static const uint8_t LoadParamByte1 = 0x61;
static const uint8_t LoadParamByte2 = 0x62;
static const uint8_t LoadParamByte3 = 0x63;
static const uint8_t ParamRequest = 0x64;
static const uint8_t ROMVersion1= 0x70;
static const uint8_t ROMVersion2= 0x71;
static const uint8_t RAMVersion1= 0x72;
static const uint8_t RAMVersion2= 0x73;
static const uint8_t ProductID= 0x90;
static const uint8_t RevisionID = 0x91;
static const uint8_t RunStatus= 0x92;
static const uint8_t UploadAddress= 0x94 ;// uint16_t registers = 0x94 (MSB)-5(LSB);
static const uint8_t UploadData = 0x96;
static const uint8_t CRCHost= 0x97;// uint32_t from registers = 0x97-9A
static const uint8_t ResetRequest = 0x9B ;
static const uint8_t PassThruStatus = 0x9E ;
static const uint8_t PassThruControl= 0xA0;

static const uint8_t ACC_LPF_BW = 0x5B;//Register GP36;
static const uint8_t GYRO_LPF_BW= 0x5C;//Register GP37
static const uint8_t BARO_LPF_BW= 0x5D;//Register GP38

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

static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) 
{
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.write(data);
    Wire.endTransmission();
}

static uint8_t readByte(uint8_t address, uint8_t subAddress) 
{
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission();
    Wire.requestFrom((int)address, (int)1);
    return Wire.read();
}

static void readBytes(
        uint8_t address,
        uint8_t subAddress,
        uint8_t count,
        uint8_t * dst)
{  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);
    Wire.endTransmission(false);      // Send Tx buffer; keep connection alive
    uint32_t i = 0;
    Wire.requestFrom((int)address, (int)count);  // Read bytes from slave reg address 
    while (Wire.available()) {
        dst[i++] = Wire.read(); 
    } 
}


static uint8_t readUsfsByte(uint8_t subAddress) 
{
    return readByte(ADDRESS, subAddress);
}

static void readUsfsBytes( uint8_t subAddress, uint8_t count, uint8_t * dest) 
{
    readBytes(ADDRESS, subAddress, count, dest);
}


static void set_integer_param (uint8_t param, uint32_t param_val) 
{
    (void)param;
    (void)param_val;
}

static void readThreeAxisRaw(uint8_t subAddress, int16_t counts[3])
{
    uint8_t rawData[6] = {};

    // Read the six raw data registers into data array
    readUsfsBytes(subAddress, 6, &rawData[0]);       

    // Turn the MSB and LSB into a signed 16-bit value
    counts[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  
    counts[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    counts[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}


static void readThreeAxisScaled(uint8_t subAddress, float scale,
        float & x, float & y, float & z)
{
    int16_t counts[3] = {};

    // Read the six raw data registers into data array
    readThreeAxisRaw(subAddress, counts);       

    // Turn the MSB and LSB into a signed 16-bit value
    x = scale * counts[0];
    y = scale * counts[1];
    z = scale * counts[2];
}

static int16_t read16BitValue(uint8_t subAddress)
{
    uint8_t counts[2] = {};

    // Read the two raw data registers sequentially into data array
    readUsfsBytes(subAddress, 2, &counts[0]);  

    // Turn the MSB and LSB into a signed 16-bit value
    return  (int16_t) (((int16_t)counts[1] << 8) | counts[0]);   
}

static void setFullScale(
        uint8_t byte0,
        uint8_t byte1,
        uint8_t byte2,
        uint8_t byte3,
        uint8_t request)
{
    uint8_t bytes[4] = {byte0, byte1, byte2, byte3};
    writeByte(ADDRESS, LoadParamByte0, bytes[0]); 
    writeByte(ADDRESS, LoadParamByte1, bytes[1]); 
    writeByte(ADDRESS, LoadParamByte2, bytes[2]); 
    writeByte(ADDRESS, LoadParamByte3, bytes[3]); 
    writeByte(ADDRESS, ParamRequest, request); 
    writeByte(ADDRESS, AlgorithmControl, 0x80); 
    uint8_t status = readByte(ADDRESS, ParamAcknowledge); 
    while (status!=request) {
        status = readByte(ADDRESS, ParamAcknowledge);
    }
    writeByte(ADDRESS, ParamRequest, 0x00); 
    writeByte(ADDRESS, AlgorithmControl, 0x00); 
}

static void setGyroFullScale(uint16_t gyro_fs)
{
    uint8_t byte0 = gyro_fs & (0xFF);
    uint8_t byte1 = (gyro_fs >> 8) & (0xFF);
    uint8_t byte2 = 0x00;
    uint8_t byte3 = 0x00;

    setFullScale(byte0, byte1, byte2, byte3, 0xCB);
}

static void setMagAccFullScale (uint16_t mag_fs, uint16_t acc_fs)
{
    uint8_t byte0 = mag_fs & (0xFF);
    uint8_t byte1 = (mag_fs >> 8) & (0xFF);
    uint8_t byte2 = acc_fs & (0xFF);
    uint8_t byte3 = (acc_fs >> 8) & (0xFF);

    setFullScale(byte0, byte1, byte2, byte3, 0xCA);
}


// ============================================================================

uint8_t usfsCheckErrors()
{
    uint8_t c = readUsfsByte(ErrorRegister);
    return c;
}


uint8_t usfsCheckStatus()
{
    // Check event status register, way to check data ready by polling rather
    // than interrupt.  Reading clears the register and interrupt.
    return readUsfsByte(EventStatus); 
}

void usfsReadAccelerometerRaw(int16_t counts[3])
{
    readThreeAxisRaw(AX, counts);
}

void usfsReadGyrometerRaw(int16_t counts[3])
{
    readThreeAxisRaw(GX, counts);
}

// Returns scaled values (G)
void usfsReadAccelerometerScaled(float & x, float & y, float & z)
{
    readThreeAxisScaled(AX, USFS_ACCEL_SCALE, x, y, z);
}

// Returns scaled values (degrees per second)
void usfsReadGyrometerScaled(float & x, float & y, float & z)
{
    readThreeAxisScaled(GX, USFS_GYRO_SCALE, x, y, z);
}

// Returns scaled values (mGauss)
void usfsreadMagnetometerScaled(float & x, float & y, float & z)
{
    readThreeAxisScaled(MX, 0.305176f, x, y, z);
}

void usfsReadQuaternion(float & qw, float & qx, float & qy, float & qz)
{
    uint8_t counts[16];  // x/y/z quaternion register data stored here

    // Read the sixteen raw data registers into data array
    readUsfsBytes(QX, 16, &counts[0]); 

    // SENtral stores quats as qx, qy, qz, qw!
    qx = uint32_reg_to_float (&counts[0]);
    qy = uint32_reg_to_float (&counts[4]);
    qz =  uint32_reg_to_float (&counts[8]);
    qw = uint32_reg_to_float (&counts[12]);   
}

int16_t usfsReadBarometerRaw()
{
    return read16BitValue(Baro);
}

int16_t usfsReadTemperatureRaw()
{
    return read16BitValue(Temp);
}

bool usfsEventStatusIsReset(uint8_t status)
{
    return status & 0x02;
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

void usfsReportError(uint8_t errorStatus)
{

    switch (errorStatus) {

        case 0x11:
            Serial.println("Magnetometer failure!");
            break;

        case 0x12:
            Serial.println("Accelerometer failure!");
            break;

        case 0x14:
            Serial.println("Gyro failure!");
            break;

        case 0x21:
            Serial.println("Magnetometer initialization failure!");
            break;

        case 0x22:
            Serial.println("Accelerometer initialization failure!");
            break;

        case 0x24:
            Serial.println("Gyro initialization failure!");
            break;

        case 0x30:
            Serial.println("Math error!");
            break;

        case 0x80:
            Serial.println("Invalid sample rate!");
            break;
    }
}
bool usfsAlgorithmStatusIsStandby(uint8_t status)
{
    return (bool)(status & 0x01);
}

bool usfsAlgorithmStatusIsAlgorithmSlow(uint8_t status)
{
    return (bool)(status & 0x02);
}

bool usfsAlgorithmStatusIsStillnessMode(uint8_t status)
{
    return (bool)(status & 0x04);
}

bool usfsAlgorithmStatusIsCalibrationCompleted(uint8_t status)
{
    return (bool)(status & 0x08);
}

bool usfsAlgorithmStatusIsMagneticAnomaly(uint8_t status)
{
    return (bool)(status & 0x10);
}

bool usfsAlgorithmStatusIsSensorUnreliable(uint8_t status)
{
    return (bool)(status & 0x20);
}

bool usfsIsInPassThroughMode(void)
{
    return (bool)readUsfsByte(PassThruStatus);
}

bool usfsRunStatusIsNormal(uint8_t status)
{
    return (bool)(status & 0x01);
}

void usfsCheckSensorStatus(uint8_t status) 
{
    if (status & 0x01) Serial.print("Magnetometer not acknowledging!");
    if (status & 0x02) Serial.print("Accelerometer not acknowledging!");
    if (status & 0x04) Serial.print("Gyro not acknowledging!");
    if (status & 0x10) Serial.print("Magnetometer ID not recognized!");
    if (status & 0x20) Serial.print("Accelerometer ID not recognized!");
    if (status & 0x40) Serial.print("Gyro ID not recognized!");
}

/*
void usfsLoadParamBytes(uint8_t byte[4])
{
    usfsWriteByte(LoadParamByte0, byte[0]);
    usfsWriteByte(LoadParamByte1, byte[1]);
    usfsWriteByte(LoadParamByte2, byte[2]);
    usfsWriteByte(LoadParamByte3, byte[3]);
}*/

void usfsWriteByte(uint8_t subAddress, uint8_t data) 
{
    writeByte(ADDRESS, subAddress, data);
}

void usfsReadSavedParamBytes(uint8_t bytes[4])
{
    bytes[0] = readUsfsByte(SavedParamByte0);
    bytes[1] = readUsfsByte(SavedParamByte1);
    bytes[2] = readUsfsByte(SavedParamByte2);
    bytes[3] = readUsfsByte(SavedParamByte3);
}

uint8_t usfsGetEventStatus(void)
{
    return readUsfsByte(EventStatus);
}

uint8_t usfsGetAlgorithmStatus(void)
{
    return readUsfsByte(AlgorithmStatus);
}

uint8_t usfsGetRunStatus(void)
{
    return readUsfsByte(RunStatus);
}

uint8_t usfsGetPassThruStatus(void)
{
    return readUsfsByte(PassThruStatus);
}

uint8_t usfsGetSensorStatus(void)
{
    return readUsfsByte(SensorStatus);
}

uint8_t usfsGetSentralStatus(void)
{
    return readUsfsByte(SentralStatus); 
}


void usfsAlgorithmControlReset(void)
{
    usfsWriteByte(AlgorithmControl, 0x00);
}

void usfsSetPassThroughMode()
{
    
    usfsWriteByte(AlgorithmControl, 0x01);
    delay(5);

    usfsWriteByte(PassThruControl, 0x01);
    while (true) {
        if (readUsfsByte(PassThruStatus) & 0x01) break;
        delay(5);
    }
}

void usfsSetMasterMode()
{
    
    usfsWriteByte(PassThruControl, 0x00);
    while (true) {
        if (!(readUsfsByte(PassThruStatus) & 0x01)) break;
        delay(5);
    }

    
    usfsWriteByte(AlgorithmControl, 0x00);
    while (true) {
        if (!(readUsfsByte(AlgorithmStatus) & 0x01)) break;
        delay(5);
    }
}

void usfsSetRunEnable(void)
{
    usfsWriteByte(HostControl, 0x01); 
}

void usfsSetRunDisable(void)
{
    usfsWriteByte(HostControl, 0x00); 
}

uint8_t usfsGetParamAcknowledge(void)
{
    return readUsfsByte(ParamAcknowledge);
}

void usfsSetAccelLpfBandwidth(uint8_t bw)
{
    usfsWriteByte(ACC_LPF_BW, bw); 
}

void usfsSetGyroLpfBandwidth(uint8_t bw)
{
    usfsWriteByte(GYRO_LPF_BW, bw); 
}

void usfsSetQRateDivisor(uint8_t divisor)
{
    usfsWriteByte(QRateDivisor, divisor);
}

void usfsSetMagRate(uint8_t rate)
{
    usfsWriteByte(MagRate, rate);
}

void usfsSetAccelRate(uint8_t rate)
{
    usfsWriteByte(AccelRate, rate);
}

void usfsSetGyroRate(uint8_t rate)
{
    usfsWriteByte(GyroRate, rate);
}

void usfsSetBaroRate(uint8_t rate)
{
    usfsWriteByte(BaroRate, rate);
}

void usfsSetRatesAndBandwidths(
        uint8_t accelLpfBandwidth,
        uint8_t accelRateTenth,
        uint8_t baroRate,
        uint8_t gyroLpfBandwidth,
        uint8_t gyroRateTenth,
        uint8_t magRate,
        uint8_t quatDivisor)
{
    usfsSetRunDisable();

    usfsWriteByte(ACC_LPF_BW, accelLpfBandwidth);   
    usfsWriteByte(AccelRate, accelRateTenth); 
    usfsWriteByte(BaroRate, 0x80 | baroRate);  
    usfsWriteByte(GYRO_LPF_BW, gyroLpfBandwidth); 
    usfsWriteByte(GyroRate, gyroRateTenth); 
    usfsWriteByte(MagRate, magRate); 
    usfsWriteByte(QRateDivisor, quatDivisor); 
}

void usfsEnableEvents(uint8_t mask)
{
    usfsWriteByte(EnableEvents, mask);
}

/* 

   Class implementation for USFS 

   Copyright (C) 2018 Simon D. Levy

   Adapted from

     https:

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
   along with USFS.  If not, see <http:
*/

#include <Arduino.h>
#include <Wire.h>

#include "USFS_old.h"
#include "USFS.h"

static const uint8_t ADDRESS = 0x28;

static const uint8_t QX                 = 0x00;  
static const uint8_t QY                 = 0x04;  
static const uint8_t QZ                 = 0x08;  
static const uint8_t QW                 = 0x0C;  
static const uint8_t QTIME              = 0x10;  
static const uint8_t MX                 = 0x12;  
static const uint8_t MY                 = 0x14;  
static const uint8_t MZ                 = 0x16;  
static const uint8_t MTIME              = 0x18;  
static const uint8_t AX                 = 0x1A;  
static const uint8_t AY                 = 0x1C;  
static const uint8_t AZ                 = 0x1E;  
static const uint8_t ATIME              = 0x20;  
static const uint8_t GX                 = 0x22;  
static const uint8_t GY                 = 0x24;  
static const uint8_t GZ                 = 0x26;  
static const uint8_t GTIME              = 0x28;  
static const uint8_t Baro               = 0x2A;  
static const uint8_t BaroTIME           = 0x2C;  
static const uint8_t Temp               = 0x2E;  
static const uint8_t TempTIME           = 0x30;  
static const uint8_t QRateDivisor       = 0x32;  
static const uint8_t EnableEvents       = 0x33;
static const uint8_t HostControl        = 0x34;
static const uint8_t EventStatus        = 0x35;
static const uint8_t SensorStatus       = 0x36;
static const uint8_t SentralStatus      = 0x37;
static const uint8_t AlgorithmStatus    = 0x38;
static const uint8_t FeatureFlags       = 0x39;
static const uint8_t ParamAcknowledge   = 0x3A;
static const uint8_t SavedParamByte0    = 0x3B;
static const uint8_t SavedParamByte1    = 0x3C;
static const uint8_t SavedParamByte2    = 0x3D;
static const uint8_t SavedParamByte3    = 0x3E;
static const uint8_t ActualMagRate      = 0x45;
static const uint8_t ActualAccelRate    = 0x46;
static const uint8_t ActualGyroRate     = 0x47;
static const uint8_t ActualBaroRate     = 0x48;
static const uint8_t ActualTempRate     = 0x49;
static const uint8_t ErrorRegister      = 0x50;
static const uint8_t AlgorithmControl   = 0x54;
static const uint8_t MagRate            = 0x55;
static const uint8_t AccelRate          = 0x56;
static const uint8_t GyroRate           = 0x57;
static const uint8_t BaroRate           = 0x58;
static const uint8_t TempRate           = 0x59;
static const uint8_t LoadParamByte0     = 0x60;
static const uint8_t LoadParamByte1     = 0x61;
static const uint8_t LoadParamByte2     = 0x62;
static const uint8_t LoadParamByte3     = 0x63;
static const uint8_t ParamRequest       = 0x64;
static const uint8_t ROMVersion1        = 0x70;
static const uint8_t ROMVersion2        = 0x71;
static const uint8_t RAMVersion1        = 0x72;
static const uint8_t RAMVersion2        = 0x73;
static const uint8_t ProductID          = 0x90;
static const uint8_t RevisionID         = 0x91;
static const uint8_t RunStatus          = 0x92;
static const uint8_t UploadAddress      = 0x94; 
static const uint8_t UploadData         = 0x96;  
static const uint8_t CRCHost            = 0x97; 
static const uint8_t ResetRequest       = 0x9B;   
static const uint8_t PassThruStatus     = 0x9E;   
static const uint8_t PassThruControl    = 0xA0;
static const uint8_t ACC_LPF_BW         = 0x5B;  
static const uint8_t GYRO_LPF_BW        = 0x5C;  
static const uint8_t BARO_LPF_BW        = 0x5D;  

static const uint8_t TEMP_OUT_H       = 0x41;
static const uint8_t TEMP_OUT_L       = 0x42;


static float uint32_reg_to_float (uint8_t *buf)
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


static void readBytes(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(ADDRESS);   
    Wire.write(subAddress);
    Wire.endTransmission(false);      
    uint32_t i = 0;
    Wire.requestFrom((int)ADDRESS, (int)count);  
    while (Wire.available()) {
        dest[i++] = Wire.read(); 
    } 
}

static uint8_t readByte(uint8_t subAddress)
{
    uint8_t data;
    readBytes(subAddress, 1, &data);
    return data;                       
}

static void readThreeAxis(uint8_t xreg, int16_t & x, int16_t & y, int16_t & z)
{
    uint8_t rawData[6]; 
    readBytes(xreg, 6, &rawData[0]);  
    x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
    y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}


// ============================================================================

bool usfs2_begin(void)
{

    
    for (int attempts=0; attempts<10; ++attempts) {
        if (readByte(SentralStatus) & 0x01) {
            if(readByte(SentralStatus) & 0x01) { }
            if(readByte(SentralStatus) & 0x02) { }
            if(readByte(SentralStatus) & 0x04) {
                return false;
            }
            if(readByte(SentralStatus) & 0x08) { }
            if(readByte(SentralStatus) & 0x10) {
                return false;
            }
            break;
        }
        usfsWriteByte(ResetRequest, 0x01);
        delay(500);  
    }


    if (readByte(SentralStatus) & 0x04) {
        return false;
    }

    return true;
}


uint8_t usfs2_getProductId(void) 
{
    return readByte(ProductID);
}

uint8_t usfs2_getRevisionId(void) 
{
    return readByte(RevisionID);
}

uint16_t usfs2_getRamVersion(void)
{
    uint16_t ram1 = readByte(RAMVersion1);
    uint16_t ram2 = readByte(RAMVersion2);

    return ram1 << 8 | ram2;
}

uint16_t usfs2_getRomVersion(void)
{
    uint16_t rom1 = readByte(ROMVersion1);
    uint16_t rom2 = readByte(ROMVersion2);

    return rom1 << 8 | rom2;
}

uint8_t usfs2_getSentralStatus(void)
{
    return readByte(SentralStatus); 
}

void usfs2_requestReset(void)
{
    usfsWriteByte(ResetRequest, 0x01);
}

void usfs2_setPassThroughMode()
{
    
    usfsWriteByte(AlgorithmControl, 0x01);
    delay(5);

    
    usfsWriteByte(PassThruControl, 0x01);
    while (true) {
        if (readByte(PassThruStatus) & 0x01) break;
        delay(5);
    }
}

void usfs2_setMasterMode()
{
    
    usfsWriteByte(PassThruControl, 0x00);
    while (true) {
        if (!(readByte(PassThruStatus) & 0x01)) break;
        delay(5);
    }

    
    usfsWriteByte(AlgorithmControl, 0x00);
    while (true) {
        if (!(readByte(AlgorithmStatus) & 0x01)) break;
        delay(5);
    }
}

void usfs2_setRunEnable(void)
{
    usfsWriteByte(HostControl, 0x01); 
}

void usfs2_setRunDisable(void)
{
    usfsWriteByte(HostControl, 0x00); 
}

void usfs2_setAccelLpfBandwidth(uint8_t bw)
{
    usfsWriteByte(ACC_LPF_BW, bw); 
}

void usfs2_setGyroLpfBandwidth(uint8_t bw)
{
    usfsWriteByte(GYRO_LPF_BW, bw); 
}

void usfs2_setQRateDivisor(uint8_t divisor)
{
    usfsWriteByte(QRateDivisor, divisor);
}

void usfs2_setMagRate(uint8_t rate)
{
    usfsWriteByte(MagRate, rate);
}

void usfs2_setAccelRate(uint8_t rate)
{
    usfsWriteByte(AccelRate, rate);
}

void usfs2_setGyroRate(uint8_t rate)
{
    usfsWriteByte(GyroRate, rate);
}

void usfs2_setBaroRate(uint8_t rate)
{
    usfsWriteByte(BaroRate, rate);
}

void usfs2_algorithmControlRequestParameterTransfer(void)
{
    usfsWriteByte(AlgorithmControl, 0x80);
}

void usfs2_algorithmControlReset(void)
{
    usfsWriteByte(AlgorithmControl, 0x00);
}

void usfs2_enableEvents(uint8_t mask)
{
    usfsWriteByte(EnableEvents, mask);
}

void usfs2_requestParamRead(uint8_t param)
{
    usfsWriteByte(ParamRequest, param); 
}

uint8_t usfs2_getParamAcknowledge(void)
{
    return readByte(ParamAcknowledge);
}

uint8_t usfs2_getRunStatus(void)
{
    return readByte(RunStatus);
}

uint8_t usfs2_getPassThruStatus(void)
{
    return readByte(PassThruStatus);
}

uint8_t usfs2_getSensorStatus(void)
{
    return readByte(SensorStatus);
}

uint8_t usfs2_getErrorStatus(void)
{
    return readByte(ErrorRegister);
}

void usfs2_setGyroFs(uint16_t gyro_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = gyro_fs & (0xFF);
    bytes[1] = (gyro_fs >> 8) & (0xFF);
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    usfsWriteByte(LoadParamByte0, bytes[0]); 
    usfsWriteByte(LoadParamByte1, bytes[1]); 
    usfsWriteByte(LoadParamByte2, bytes[2]); 
    usfsWriteByte(LoadParamByte3, bytes[3]); 
    usfsWriteByte(ParamRequest, 0xCB); 
    usfsWriteByte(AlgorithmControl, 0x80); 
    STAT = readByte(ParamAcknowledge); 
    while(!(STAT==0xCB)) {
        STAT = readByte(ParamAcknowledge);
    }
    usfsWriteByte(ParamRequest, 0x00); 
    usfsWriteByte(AlgorithmControl, 0x00); 
}

void usfs2_setMagAccFs(uint16_t mag_fs, uint16_t acc_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    usfsWriteByte(LoadParamByte0, bytes[0]); 
    usfsWriteByte(LoadParamByte1, bytes[1]); 
    usfsWriteByte(LoadParamByte2, bytes[2]); 
    usfsWriteByte(LoadParamByte3, bytes[3]); 
    usfsWriteByte(ParamRequest, 0xCA); 
    usfsWriteByte(AlgorithmControl, 0x80); 
    STAT = readByte(ParamAcknowledge); 
    while(!(STAT==0xCA)) {
        STAT = readByte(ParamAcknowledge);
    }
    usfsWriteByte(ParamRequest, 0x00); 
    usfsWriteByte(AlgorithmControl, 0x00); 
}

void usfs2_readAccelerometer(int16_t & ax, int16_t & ay, int16_t & az)
{
    readThreeAxis(AX, ax, ay, az);
}

void usfs2_readQuaternion(float & qw, float & qx, float & qy, float &qz)
{
    uint8_t rawData[16];  

    readBytes(QX, 16, &rawData[0]);       

    qx = uint32_reg_to_float (&rawData[0]);
    qy = uint32_reg_to_float (&rawData[4]);
    qz = uint32_reg_to_float (&rawData[8]);
    qw = uint32_reg_to_float (&rawData[12]);
}

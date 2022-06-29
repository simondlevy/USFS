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
static const uint8_t GP36               = 0x5B;
static const uint8_t GP37               = 0x5C;
static const uint8_t GP38               = 0x5D;
static const uint8_t GP39               = 0x5E;
static const uint8_t GP40               = 0x5F;
static const uint8_t GP50               = 0x69;
static const uint8_t GP51               = 0x6A;
static const uint8_t GP52               = 0x6B;
static const uint8_t GP53               = 0x6C;
static const uint8_t GP54               = 0x6D;
static const uint8_t GP55               = 0x6E;
static const uint8_t GP56               = 0x6F;


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

static void writeByte(uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(ADDRESS);
    Wire.write(subAddress);
    Wire.write(data);
    Wire.endTransmission();
}

static void readThreeAxis(uint8_t xreg, int16_t & x, int16_t & y, int16_t & z)
{
    uint8_t rawData[6]; 
    readBytes(xreg, 6, &rawData[0]);  
    x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
    y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}




bool USFS::begin(void)
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
        writeByte(ResetRequest, 0x01);
        delay(500);  
    }


    if (readByte(SentralStatus) & 0x04) {
        return false;
    }

    return true;
}


uint8_t USFS::getProductId(void) 
{
    return readByte(ProductID);
}

uint8_t USFS::getRevisionId(void) 
{
    return readByte(RevisionID);
}

uint16_t USFS::getRamVersion(void)
{
    uint16_t ram1 = readByte(RAMVersion1);
    uint16_t ram2 = readByte(RAMVersion2);

    return ram1 << 8 | ram2;
}

uint16_t USFS::getRomVersion(void)
{
    uint16_t rom1 = readByte(ROMVersion1);
    uint16_t rom2 = readByte(ROMVersion2);

    return rom1 << 8 | rom2;
}

uint8_t USFS::getSentralStatus(void)
{
    return readByte(SentralStatus); 
}

void USFS::requestReset(void)
{
    writeByte(ResetRequest, 0x01);
}

void USFS::setPassThroughMode()
{
    
    writeByte(AlgorithmControl, 0x01);
    delay(5);

    
    writeByte(PassThruControl, 0x01);
    while (true) {
        if (readByte(PassThruStatus) & 0x01) break;
        delay(5);
    }
}

void USFS::setMasterMode()
{
    
    writeByte(PassThruControl, 0x00);
    while (true) {
        if (!(readByte(PassThruStatus) & 0x01)) break;
        delay(5);
    }

    
    writeByte(AlgorithmControl, 0x00);
    while (true) {
        if (!(readByte(AlgorithmStatus) & 0x01)) break;
        delay(5);
    }
}

void USFS::setRunEnable(void)
{
    writeByte(HostControl, 0x01); 
}

void USFS::setRunDisable(void)
{
    writeByte(HostControl, 0x00); 
}

void USFS::setAccelLpfBandwidth(uint8_t bw)
{
    writeByte(ACC_LPF_BW, bw); 
}

void USFS::setGyroLpfBandwidth(uint8_t bw)
{
    writeByte(GYRO_LPF_BW, bw); 
}

void USFS::setQRateDivisor(uint8_t divisor)
{
    writeByte(QRateDivisor, divisor);
}

void USFS::setMagRate(uint8_t rate)
{
    writeByte(MagRate, rate);
}

void USFS::setAccelRate(uint8_t rate)
{
    writeByte(AccelRate, rate);
}

void USFS::setGyroRate(uint8_t rate)
{
    writeByte(GyroRate, rate);
}

void USFS::setBaroRate(uint8_t rate)
{
    writeByte(BaroRate, rate);
}

void USFS::algorithmControlRequestParameterTransfer(void)
{
    writeByte(AlgorithmControl, 0x80);
}

void USFS::algorithmControlReset(void)
{
    writeByte(AlgorithmControl, 0x00);
}

void USFS::enableEvents(uint8_t mask)
{
    writeByte(EnableEvents, mask);
}

void USFS::requestParamRead(uint8_t param)
{
    writeByte(ParamRequest, param); 
}

uint8_t USFS::getParamAcknowledge(void)
{
    return readByte(ParamAcknowledge);
}

uint8_t USFS::readSavedParamByte0(void)
{
    return readByte(SavedParamByte0);
}

uint8_t USFS::readSavedParamByte1(void)
{
    return readByte(SavedParamByte1);
}

uint8_t USFS::readSavedParamByte2(void)
{
    return readByte(SavedParamByte2);
}

uint8_t USFS::readSavedParamByte3(void)
{
    return readByte(SavedParamByte3);
}

uint8_t USFS::getRunStatus(void)
{
    return readByte(RunStatus);
}

uint8_t USFS::getAlgorithmStatus(void)
{
    return readByte(AlgorithmStatus);
}

uint8_t USFS::getPassThruStatus(void)
{
    return readByte(PassThruStatus);
}

uint8_t USFS::getEventStatus(void)
{
    return readByte(EventStatus);
}

uint8_t USFS::getSensorStatus(void)
{
    return readByte(SensorStatus);
}

uint8_t USFS::getErrorStatus(void)
{
    return readByte(ErrorRegister);
}

void USFS::setGyroFs(uint16_t gyro_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = gyro_fs & (0xFF);
    bytes[1] = (gyro_fs >> 8) & (0xFF);
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    writeByte(LoadParamByte0, bytes[0]); 
    writeByte(LoadParamByte1, bytes[1]); 
    writeByte(LoadParamByte2, bytes[2]); 
    writeByte(LoadParamByte3, bytes[3]); 
    writeByte(ParamRequest, 0xCB); 
    writeByte(AlgorithmControl, 0x80); 
    STAT = readByte(ParamAcknowledge); 
    while(!(STAT==0xCB)) {
        STAT = readByte(ParamAcknowledge);
    }
    writeByte(ParamRequest, 0x00); 
    writeByte(AlgorithmControl, 0x00); 
}

void USFS::setMagAccFs(uint16_t mag_fs, uint16_t acc_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    writeByte(LoadParamByte0, bytes[0]); 
    writeByte(LoadParamByte1, bytes[1]); 
    writeByte(LoadParamByte2, bytes[2]); 
    writeByte(LoadParamByte3, bytes[3]); 
    writeByte(ParamRequest, 0xCA); 
    writeByte(AlgorithmControl, 0x80); 
    STAT = readByte(ParamAcknowledge); 
    while(!(STAT==0xCA)) {
        STAT = readByte(ParamAcknowledge);
    }
    writeByte(ParamRequest, 0x00); 
    writeByte(AlgorithmControl, 0x00); 
}

void USFS::loadParamByte0(uint8_t value)
{
    writeByte(LoadParamByte0, value);
}

void USFS::loadParamByte1(uint8_t value)
{
    writeByte(LoadParamByte1, value);
}

void USFS::loadParamByte2(uint8_t value)
{
    writeByte(LoadParamByte2, value);
}

void USFS::loadParamByte3(uint8_t value)
{
    writeByte(LoadParamByte3, value);
}

void USFS::writeGp36(uint8_t value)
{
    writeByte(GP36, value);
}

void USFS::writeGp37(uint8_t value)
{
    writeByte(GP37, value);
}

void USFS::writeGp38(uint8_t value)
{
    writeByte(GP38, value);
}

void USFS::writeGp39(uint8_t value)
{
    writeByte(GP39, value);
}

void USFS::writeGp40(uint8_t value)
{
    writeByte(GP40, value);
}

void USFS::writeGp50(uint8_t value)
{
    writeByte(GP50, value);
}

void USFS::writeGp51(uint8_t value)
{
    writeByte(GP51, value);
}

void USFS::writeGp52(uint8_t value)
{
    writeByte(GP52, value);
}

void USFS::writeGp53(uint8_t value)
{
    writeByte(GP53, value);
}

void USFS::writeGp54(uint8_t value)
{
    writeByte(GP54, value);
}

void USFS::writeGp55(uint8_t value)
{
    writeByte(GP55, value);
}

void USFS::writeGp56(uint8_t value)
{
    writeByte(GP56, value);
}

void USFS::readAccelerometer(int16_t & ax, int16_t & ay, int16_t & az)
{
    readThreeAxis(AX, ax, ay, az);
}

void USFS::readQuaternion(float & qw, float & qx, float & qy, float &qz)
{
    uint8_t rawData[16];  

    readBytes(QX, 16, &rawData[0]);       

    qx = uint32_reg_to_float (&rawData[0]);
    qy = uint32_reg_to_float (&rawData[4]);
    qz = uint32_reg_to_float (&rawData[8]);
    qw = uint32_reg_to_float (&rawData[12]);
}


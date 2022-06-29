/* 

   Class implementation for USFS 

   Copyright (C) 2018 Simon D. Levy

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/MPU9250_BMP280

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

#include "USFS_old.h"

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
    Wire.beginTransmission(USFS::ADDRESS);   // Initialize the Tx buffer
    Wire.write(subAddress);
    Wire.endTransmission(false);      // Send Tx buffer; keep connection alive
    uint32_t i = 0;
    Wire.requestFrom((int)USFS::ADDRESS, (int)count);  // Read bytes from slave reg address 
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
    Wire.beginTransmission(USFS::ADDRESS);
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


// ============================================================================

bool USFS::begin(void)
{

    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
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
    // First put SENtral in standby mode
    writeByte(AlgorithmControl, 0x01);
    delay(5);

    // Place SENtral in pass-through mode
    writeByte(PassThruControl, 0x01);
    while (true) {
        if (readByte(PassThruStatus) & 0x01) break;
        delay(5);
    }
}

void USFS::setMasterMode()
{
    // Cancel pass-through mode
    writeByte(PassThruControl, 0x00);
    while (true) {
        if (!(readByte(PassThruStatus) & 0x01)) break;
        delay(5);
    }

    // Re-start algorithm
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
    writeByte(LoadParamByte0, bytes[0]); //Gyro LSB
    writeByte(LoadParamByte1, bytes[1]); //Gyro MSB
    writeByte(LoadParamByte2, bytes[2]); //Unused
    writeByte(LoadParamByte3, bytes[3]); //Unused
    writeByte(ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
    writeByte(AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCB)) {
        STAT = readByte(ParamAcknowledge);
    }
    writeByte(ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(AlgorithmControl, 0x00); // Re-start algorithm
}

void USFS::setMagAccFs(uint16_t mag_fs, uint16_t acc_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    writeByte(LoadParamByte0, bytes[0]); //Mag LSB
    writeByte(LoadParamByte1, bytes[1]); //Mag MSB
    writeByte(LoadParamByte2, bytes[2]); //Acc LSB
    writeByte(LoadParamByte3, bytes[3]); //Acc MSB
    writeByte(ParamRequest, 0xCA); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
    writeByte(AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCA)) {
        STAT = readByte(ParamAcknowledge);
    }
    writeByte(ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(AlgorithmControl, 0x00); // Re-start algorithm
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
    uint8_t rawData[16];  // x/y/z/w quaternion register data stored here (note unusual order!)

    readBytes(QX, 16, &rawData[0]);       

    qx = uint32_reg_to_float (&rawData[0]);
    qy = uint32_reg_to_float (&rawData[4]);
    qz = uint32_reg_to_float (&rawData[8]);
    qw = uint32_reg_to_float (&rawData[12]);
}


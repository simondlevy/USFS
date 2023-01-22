/* 
   USFS source header

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

#pragma once

#include <stdint.h>

#include <Arduino.h>
#include <Wire.h>

enum {

    USFS_INTERRUPT_RESET_REQUIRED = 0x01,
    USFS_INTERRUPT_ERROR = 0x02,
    USFS_INTERRUPT_QUAT = 0x04,
    USFS_INTERRUPT_MAG = 0x08,
    USFS_INTERRUPT_ACCEL = 0x10,
    USFS_INTERRUPT_GYRO = 0x20,
    USFS_INTERRUPT_ANY = 0x40
};

typedef enum {

    AFS_2G,
    AFS_4G,
    AFS_8G,
    AFS_16G

} ascale_t;

typedef enum {

    GFS_250DPS,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS

} gscale_t;

typedef enum {

    MFS_14BITS, // 0.6 mG per LSB
    MFS_16BITS  // 0.15 mG per LSB

} mscale_t;

// Full-scale ranges are fixed in master mode
static const float USFS_GYRO_SCALE  = 1.53e-1;
static const float USFS_ACCEL_SCALE = 4.88e-4;

void usfsAlgorithmControlRequestParameterTransfer(void);

void usfsAlgorithmControlReset(void); 

bool usfsAlgorithmStatusIsStandby(uint8_t status);
bool usfsAlgorithmStatusIsAlgorithmSlow(uint8_t status);
bool usfsAlgorithmStatusIsStillnessMode(uint8_t status);
bool usfsAlgorithmStatusIsCalibrationCompleted(uint8_t status);
bool usfsAlgorithmStatusIsMagneticAnomaly(uint8_t status);
bool usfsAlgorithmStatusIsSensorUnreliable(uint8_t status);

void usfsBegin(
        uint8_t accelBandwidth,
        uint8_t gyroBandwidth,
        uint8_t quatDivisor,
        uint8_t magRate,
        uint8_t accelRateTenth,
        uint8_t gyroRateTenth,
        uint8_t baroRate, 
        uint8_t interruptEnable=USFS_INTERRUPT_GYRO,
        bool verbose=false);

uint8_t usfsCheckErrors();

void usfsCheckSensorStatus(uint8_t status);

uint8_t usfsCheckStatus();

void usfsEnableEvents(uint8_t mask);

bool usfsEventStatusIsAccelerometer(uint8_t status);
bool usfsEventStatusIsBarometer(uint8_t status);
bool usfsEventStatusIsError(uint8_t status);
bool usfsEventStatusIsGyrometer(uint8_t status);
bool usfsEventStatusIsMagnetometer(uint8_t status);
bool usfsEventStatusIsQuaternion(uint8_t status);
bool usfsEventStatusIsReset(uint8_t status);

uint8_t usfsGetAlgorithmStatus(void);
uint8_t usfsGetEventStatus(void);
uint8_t usfsGetParamAcknowledge(void);
uint8_t usfsGetPassThruStatus(void);
uint8_t usfsGetRunStatus(void);
uint8_t usfsGetSensorStatus(void);
uint8_t usfsGetSentralStatus(void);


bool usfsIsInPassThroughMode(void);

void usfsLoadFirmware(bool verbose = false);

void usfsLoadParamBytes(uint8_t byte[4]);

int16_t usfsReadBarometerRaw();

void usfsReadAccelerometerRaw(int16_t counts[3]);

void usfsReadGyrometerRaw(int16_t counts[3]);

// Returns Gs
void usfsReadAccelerometerScaled(float & x, float & y, float & z);

// Returns degrees per second
void usfsReadGyrometerScaled(float & x, float & y, float & z);

void usfsreadMagnetometerScaled(float & x, float & y, float & z);

void usfsReadQuaternion(float & qw, float & qx, float & qy, float & qz);

int16_t usfsReadTemperatureRaw();

void  usfsReportChipId();

void usfsReadSavedParamBytes(uint8_t bytes[4]);

void usfsReportError(uint8_t errorStatus);

void usfsRequestParamRead(uint8_t param);

bool usfsRunStatusIsNormal(uint8_t status);

void usfsSetMasterMode(void);
void usfsSetPassThroughMode(void);

void usfsSetRatesAndBandwidths(
        uint8_t accelLpfBandwidth,
        uint8_t accelRateTenth,
        uint8_t baroRate,
        uint8_t gyroLpfBandwidth,
        uint8_t gyroRateTenth,
        uint8_t magRate,
        uint8_t quatDivisor);

void usfsSetRunEnable(void);
void usfsSetRunDisable(void);

void usfsWriteByte(uint8_t subAddress, uint8_t value);

class Usfs {

    private:

        static const uint8_t ADDRESS= 0x28;

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

        static uint8_t readByte(uint8_t address, uint8_t subAddress) 
        {
            Wire.beginTransmission(address);
            Wire.write(subAddress);
            Wire.endTransmission();
            Wire.requestFrom((int)address, (int)1);
            return Wire.read();
        }

        static uint8_t readUsfsByte(uint8_t subAddress) 
        {
            return readByte(ADDRESS, subAddress);
        }

    public:

        void reportChipId(void)
        {
            // Read SENtral device information
            uint16_t ROM1 = readUsfsByte(ROMVersion1);
            uint16_t ROM2 = readUsfsByte(ROMVersion2);
            Serial.print("EM7180 ROM Version: 0x");
            Serial.print(ROM1, HEX);
            Serial.println(ROM2, HEX);
            Serial.println("Should be: 0xE609");
            uint16_t RAM1 = readUsfsByte(RAMVersion1);
            uint16_t RAM2 = readUsfsByte(RAMVersion2);
            Serial.print("EM7180 RAM Version: 0x");
            Serial.print(RAM1);
            Serial.println(RAM2);
            uint8_t PID = readUsfsByte(ProductID);
            Serial.print("EM7180 ProductID: 0x");
            Serial.print(PID, HEX);
            Serial.println(" Should be: 0x80");
            uint8_t RID = readUsfsByte(RevisionID);
            Serial.print("EM7180 RevisionID: 0x");
            Serial.print(RID, HEX);
            Serial.println(" Should be: 0x02");
        }    

}; // class Usfs


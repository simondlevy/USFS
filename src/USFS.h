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

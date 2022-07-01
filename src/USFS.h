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

uint8_t usfsCheckErrors();

uint8_t usfsCheckStatus();

void  usfsReportChipId();

void usfsBegin(
        uint8_t accelBandwidth,
        uint8_t gyroBandwidth,
        uint16_t accelScale,
        uint16_t gyroScale,
        uint16_t magScale,
        uint8_t quatDivisor,
        uint8_t magRate,
        uint8_t accelRateTenth,
        uint8_t gyroRateTenth,
        uint8_t baroRate, 
        uint8_t interruptEnable=USFS_INTERRUPT_GYRO,
        bool verbose=false);

void usfsLoadFirmware();

void    usfsReadAccelerometer(int16_t * destination);
int16_t usfsReadBarometer();
void    usfsReadGyrometer(int16_t * destination);
void    usfsreadMagnetometer(int16_t * destination);
void    usfsReadQuaternion(float * destination);
int16_t usfsReadTemperature();

bool usfsEventStatusIsError(uint8_t status);

bool usfsEventStatusIsAccelerometer(uint8_t status);
bool usfsEventStatusIsGyrometer(uint8_t status);
bool usfsEventStatusIsMagnetometer(uint8_t status);
bool usfsEventStatusIsQuaternion(uint8_t status);
bool usfsEventStatusIsBarometer(uint8_t status);

void usfsReportError(uint8_t errorStatus);

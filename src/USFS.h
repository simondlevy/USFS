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

    USFS_ERROR_MAGNETOMETER       = 0x11,
    USFS_ERROR_ACCELEROMETER      = 0x12,
    USFS_ERROR_GYROMETER          = 0x14,
    USFS_ERROR_MAGNETOMETER_INIT  = 0x21,
    USFS_ERROR_ACCELEROMETER_INIT = 0x22,
    USFS_ERROR_GYROMETER_INIT     = 0x24,
    USFS_ERROR_MATH               = 0x30,
    USFS_ERROR_SAMPLE_RATE        = 0x80
};

uint8_t usfsCheckErrors();

uint8_t usfsCheckStatus();

void  usfsReportChipId();

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
        uint8_t baroRt);

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

/* 
   Class header for USFS in master mode

   Copyright (C) 2018 Simon D. Levy

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

#include "USFS.h"

class USFS_Master {

    private:

        USFS _usfs;

        uint8_t _eventStatus;

        uint8_t  _magRate;      // Hz
        uint16_t _accelRate;    // Hz
        uint16_t _gyroRate;     // Hz
        uint8_t  _baroRate;     // Hz
        uint8_t  _qRateDivisor; // w.r.t. gyro rate

        void readThreeAxis(uint8_t regx, float & x, float & y, float & z, float scale);

    public:

        USFS_Master(uint8_t  magRate, uint16_t accelRate, uint16_t gyroRate, uint8_t  baroRate, uint8_t qRateDivisor); 

        const char * getErrorString(void);

        bool begin(uint8_t bus=1);

        void checkEventStatus(void);

        bool gotError(void);

        bool gotQuaternion(void);

        bool gotMagnetometer(void);

        bool gotAccelerometer(void);

        bool gotGyrometer(void);

        bool gotBarometer(void);

        void readMagnetometer(float & mx, float & my, float & mz);

        void readAccelerometer(float & ax, float & ay, float & az);

        void readGyrometer(float & gx, float & gy, float & gz);

        void readQuaternion(float & qw, float & qx, float & qy, float & qz);

        void readBarometer(float & pressure, float & temperature);
};

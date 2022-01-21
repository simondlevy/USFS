/* 

    Class implementation for USFS in master mode

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

#include "USFS_Master.h"

#include <CrossPlatformI2C_Core.h>

USFS_Master::USFS_Master(uint8_t magRate, uint16_t accelRate, uint16_t gyroRate, uint8_t baroRate, uint8_t qRateDivisor) 
{
    _magRate = magRate;
    _accelRate = accelRate;
    _gyroRate = gyroRate; 
    _baroRate = baroRate;
    _qRateDivisor = qRateDivisor;
}

const char * USFS_Master::getErrorString(void)
{
    return _usfs.getErrorString();
}

bool USFS_Master::begin(uint8_t bus)
{
    // Fail immediately if unable to upload EEPROM
    if (!_usfs.begin(bus)) return false;

    // Enter USFS initialized state
    _usfs.setRunDisable();// set SENtral in initialized state to configure registers
    _usfs.setMasterMode();
    _usfs.setRunEnable();
    _usfs.setRunDisable();// set SENtral in initialized state to configure registers

    // Setup LPF bandwidth (BEFORE setting ODR's)
    _usfs.setAccelLpfBandwidth(0x03); // 41Hz
    _usfs.setGyroLpfBandwidth(0x03);  // 41Hz

    // Set accel/gyro/mage desired ODR rates
    _usfs.setQRateDivisor(_qRateDivisor-1);
    _usfs.setMagRate(_magRate);
    _usfs.setAccelRate(_accelRate/10);
    _usfs.setGyroRate(_gyroRate/10);
    _usfs.setBaroRate(0x80 | _baroRate); // 0x80 = enable bit

    // Configure operating modeA
    _usfs.algorithmControlReset();// read scale sensor data

    // Enable interrupt to host upon certain events:
    // quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    _usfs.enableEvents(0x07);

    // Enable USFS run mode
    _usfs.setRunEnable();// set SENtral in normal run mode
    delay(100);

    // Disable stillness mode
    _usfs.setIntegerParam (0x49, 0x00);

    // Success
    return _usfs.getSensorStatus() ? false : true;
}

void USFS_Master::checkEventStatus(void)
{
    // Check event status register, way to check data ready by checkEventStatusing rather than interrupt
    _eventStatus = _usfs.getEventStatus(); // reading clears the register

}

bool USFS_Master::gotError(void)
{
    if (_eventStatus & 0x02) {

        return true;
    }

    return false;
}

bool USFS_Master::gotQuaternion(void)
{
    return _eventStatus & 0x04;
}

bool USFS_Master::gotMagnetometer(void)
{
    return _eventStatus & 0x08;
}

bool USFS_Master::gotAccelerometer(void)
{
    return _eventStatus & 0x10;
}

bool USFS_Master::gotGyrometer(void)
{
    return _eventStatus & 0x20;
}

bool USFS_Master::gotBarometer(void)
{
    return _eventStatus & 0x40;
}

void USFS_Master::readQuaternion(float & qw, float & qx, float & qy, float &qz)
{
    _usfs.readQuaternion(qw, qx, qy, qz);
}

void USFS_Master::readThreeAxis(uint8_t regx, float & x, float & y, float & z, float scale)
{
    int16_t xx=0, yy=0, zz=0;

    _usfs.readThreeAxis(regx, xx, yy, zz);

    x = xx * scale;
    y = yy * scale;
    z = zz * scale;
}

void USFS_Master::readAccelerometer(float & ax, float & ay, float & az)
{
    readThreeAxis(USFS::AX, ax, ay, az, 0.000488);
}

void USFS_Master::readGyrometer(float & gx, float & gy, float & gz)
{
    readThreeAxis(USFS::GX, gx, gy, gz, 0.153);
}

void USFS_Master::readGyrometer(int16_t & gx, int16_t & gy, int16_t & gz)
{
    _usfs.readGyrometer(gx, gy, gz);
}

void USFS_Master::readMagnetometer(float & mx, float & my, float & mz)
{
    readThreeAxis(USFS::MX, mx, my, mz, 0.305176);
}

void USFS_Master::readBarometer(float & pressure, float & temperature)
{
    _usfs.readBarometer(pressure, temperature);
}

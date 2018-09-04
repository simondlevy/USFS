/* 

   EM7180_Master.cpp: Class implementation for EM7180 SENtral Sensor in master mode

   Copyright (C) 2018 Simon D. Levy

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/MPU9250_BMP280

   This file is part of EM7180.

   EM7180 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   EM7180 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "EM7180_Master.h"

#include <CrossPlatformI2C_Core.h>

EM7180_Master::EM7180_Master(uint8_t magRate, uint16_t accelRate, uint16_t gyroRate, uint8_t baroRate, uint8_t qRateDivisor) 
{
    _magRate = magRate;
    _accelRate = accelRate;
    _gyroRate = gyroRate; 
    _baroRate = baroRate;
    _qRateDivisor = qRateDivisor;
}

const char * EM7180_Master::getErrorString(void)
{
    return _em7180.getErrorString();
}

bool EM7180_Master::begin(uint8_t bus)
{
    // Fail immediately if unable to upload EEPROM
    if (!_em7180.begin(bus)) return false;

    // Enter EM7180 initialized state
    _em7180.setRunDisable();// set SENtral in initialized state to configure registers
    _em7180.setMasterMode();
    _em7180.setRunEnable();
    _em7180.setRunDisable();// set SENtral in initialized state to configure registers

    // Setup LPF bandwidth (BEFORE setting ODR's)
    _em7180.setAccelLpfBandwidth(0x03); // 41Hz
    _em7180.setGyroLpfBandwidth(0x03);  // 41Hz

    // Set accel/gyro/mage desired ODR rates
    _em7180.setQRateDivisor(_qRateDivisor-1);
    _em7180.setMagRate(_magRate);
    _em7180.setAccelRate(_accelRate/10);
    _em7180.setGyroRate(_gyroRate/10);
    _em7180.setBaroRate(0x80 | _baroRate); // 0x80 = enable bit

    // Configure operating modeA
    _em7180.algorithmControlReset();// read scale sensor data

    // Enable interrupt to host upon certain events:
    // quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    _em7180.enableEvents(0x07);

    // Enable EM7180 run mode
    _em7180.setRunEnable();// set SENtral in normal run mode
    delay(100);

    // Disable stillness mode
    _em7180.setIntegerParam (0x49, 0x00);

    // Success
    return _em7180.getSensorStatus() ? false : true;
}

void EM7180_Master::checkEventStatus(void)
{
    // Check event status register, way to check data ready by checkEventStatusing rather than interrupt
    _eventStatus = _em7180.getEventStatus(); // reading clears the register

}

bool EM7180_Master::gotError(void)
{
    if (_eventStatus & 0x02) {

        return true;
    }

    return false;
}

bool EM7180_Master::gotQuaternion(void)
{
    return _eventStatus & 0x04;
}

bool EM7180_Master::gotMagnetometer(void)
{
    return _eventStatus & 0x08;
}

bool EM7180_Master::gotAccelerometer(void)
{
    return _eventStatus & 0x10;
}

bool EM7180_Master::gotGyrometer(void)
{
    return _eventStatus & 0x20;
}

bool EM7180_Master::gotBarometer(void)
{
    return _eventStatus & 0x40;
}

void EM7180_Master::readQuaternion(float & qw, float & qx, float & qy, float &qz)
{
    _em7180.readQuaternion(qw, qx, qy, qz);
}

void EM7180_Master::readThreeAxis(uint8_t regx, float & x, float & y, float & z, float scale)
{
    int16_t xx=0, yy=0, zz=0;

    _em7180.readThreeAxis(regx, xx, yy, zz);

    x = xx * scale;
    y = yy * scale;
    z = zz * scale;
}

void EM7180_Master::readAccelerometer(float & ax, float & ay, float & az)
{
    readThreeAxis(EM7180::AX, ax, ay, az, 0.000488);
}

void EM7180_Master::readGyrometer(float & gx, float & gy, float & gz)
{
    readThreeAxis(EM7180::GX, gx, gy, gz, 0.153);
}

void EM7180_Master::readMagnetometer(float & mx, float & my, float & mz)
{
    readThreeAxis(EM7180::MX, mx, my, mz, 0.305176);
}

void EM7180_Master::readBarometer(float & pressure, float & temperature)
{
    _em7180.readBarometer(pressure, temperature);
}

/* 
   BaroImuFusion.ino: altitude estimation via barometer / IMU fusion

   Copyright (C) 2018 Simon D. Levy and Greg Tomasch

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

#include "EM7180.h"

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

EM7180 em7180;

// 1G = 2048 accelerometer reading
static float G_PER_COUNT = 1/2048.f;

// Accelerometer raw (ADC) and quaternions will be set in loop as readings become available
static int16_t accelX, accelY, accelZ;
static float qw, qx, qy, qz;

static void computeIMU ()
{
    // Pass-thru for future filter experimentation
    int16_t accelSmoothX = accelX;
    int16_t accelSmoothY = accelY;
    int16_t accelSmoothZ = accelZ;

    // Rotation matrix components to calculate Euler anfles and linear acceleration
    float a11 = qw*qw+qx*qx -qy*qy-qz*qz;
    float a12 = 2.0f*(qx*qy-qw*qz);
    float a13 = 2.0f*(qx*qz+qw*qy);
    float a21 = 2.0f*(qw*qz+qx*qy);
    float a22 = qw*qw-qx*qx +qy*qy-qz*qz;
    float a23 = 2.0f*(qy*qz-qw*qx);
    float a31 = 2.0f*(qx*qz-qw*qy);
    float a32 = 2.0f*(qw*qx+qy*qz);
    float a33 = qw*qw-qx*qx -qy*qy+qz*qz;

    float acc_X_minus_grav = (float)accelSmoothX;
    float acc_Y_minus_grav = (float)accelSmoothY;
    float acc_Z_minus_grav = (float)accelSmoothZ;

    acc_X_minus_grav *= G_PER_COUNT;
    acc_Y_minus_grav *= G_PER_COUNT;
    acc_Z_minus_grav *= G_PER_COUNT;

    Serial.print(acc_Z_minus_grav);

    acc_X_minus_grav -= a31;
    acc_Y_minus_grav -= a32;
    acc_Z_minus_grav -= a33;

    Serial.print(" => ");
    Serial.println(acc_Z_minus_grav);

    float LINaccData[3];
    LINaccData[0] = -(a11*acc_X_minus_grav + a12*acc_Y_minus_grav + a13*acc_Z_minus_grav); // 0=LAT 
    LINaccData[1] = -(a21*acc_X_minus_grav + a22*acc_Y_minus_grav + a23*acc_Z_minus_grav); // 1=LON
    LINaccData[2] =  (a31*acc_X_minus_grav + a32*acc_Y_minus_grav + a33*acc_Z_minus_grav); // 2=Z
}

void setup()
{
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    Serial.begin(38400);

    // Goose up the EM7180 ODRs
    em7180.accelRate = 330;
    em7180.gyroRate = 330;
    em7180.baroRate = 50;
    em7180.qRateDivisor = 5;

    // Start the EM7180 in master mode, polling instead of interrupt
    if (!em7180.begin()) {

        while (true) {
            Serial.println(em7180.getErrorString());
        }
    }
}

void loop()
{  
    em7180.checkEventStatus();

    if (em7180.gotError()) {
        Serial.print("ERROR: ");
        Serial.println(em7180.getErrorString());
        return;
    }

    if (em7180.gotQuaternions()) {
        em7180.readQuaternions(qw, qx, qy, qz);
    }

    if (em7180.gotAccelerometer()) {
        em7180.readAccelerometer(accelX, accelY, accelZ);
        computeIMU();
    }

    if (em7180.gotBarometer()) {
        float temperature, pressure;
        em7180.readBarometer(pressure, temperature);
    }
}



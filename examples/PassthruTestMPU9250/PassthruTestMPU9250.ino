/* 
   PassthruTestMPU9250.ino: Example sketch for running EM7180 SENtral 
   sensor hub in pass-through mode to talk directly to MPU9250

   Copyright (C) 2018 Simon D. Levy

   Additional dependencies:

       https://github.com/simondlevy/MPU

       https://github.com/simondlevy/CrossPlatformDataBus

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

#include <MPU9250_Master_I2C.h>

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

EM7180 em7180;

static const MPUIMU::Ascale_t  ASCALE              = MPUIMU::AFS_2G;
static const MPUIMU::Gscale_t  GSCALE              = MPUIMU::GFS_250DPS;
static const MPU9250::Mscale_t MSCALE              = MPU9250::MFS_16BITS;
static const MPU9250::Mmode_t  MMODE               = MPU9250::M_100Hz;
static const uint8_t           SAMPLE_RATE_DIVISOR = 4;         

// Instantiate MPU9250 class in master mode
static MPU9250_Master_I2C mpu9250(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);

static void mpu9250_error(const char * errmsg) 
{
    Serial.print("MPU9250 Error: ");
    Serial.println(errmsg);
    while (true) ;
}

static void reportAcceleration(const char * dim, float val)
{
    Serial.print(dim);
    Serial.print("-acceleration: ");
    Serial.print(1000*val);
    Serial.print(" milliG  "); 
}

static void reportGyroRate(const char * dim, float val)
{
    Serial.print(dim);
    Serial.print("-gyro rate: ");
    Serial.print(val, 1);
    Serial.print(" degrees/sec  "); 
}

static void reportMagnetometer(const char * dim, float val)
{
    Serial.print(dim);
    Serial.print("-magnetometer: ");
    Serial.print(val);
    Serial.print(" milligauss  "); 
}

void setup()
{
    Serial.begin(115200);

#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    // Start the EM7180
    if (!em7180.begin()) {

        while (true) {
            Serial.println(em7180.getErrorString());
        }
    }    

    // Put the EM7180 into pass-through mode
    em7180.setPassThroughMode();

    // Start the MPU9250 in master mode
    switch (mpu9250.begin()) {

        case MPUIMU::ERROR_IMU_ID:
            mpu9250_error("Bad IMU device ID");
        case MPUIMU::ERROR_MAG_ID:
            mpu9250_error("Bad magnetometer device ID");
        case MPUIMU::ERROR_SELFTEST:
            mpu9250_error("Failed self-test");
        default:
            Serial.println("MPU6050 online!\n");
    }
}


void loop()
{  
    static float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

    if (mpu9250.checkNewData())  { // data ready interrupt is detected

        mpu9250.readAccelerometer(ax, ay, az);
        mpu9250.readGyrometer(gx, gy, gz);
        mpu9250.readMagnetometer(mx, my, mz);
        temperature = mpu9250.readTemperature();
    }

    // Report at 4 Hz
    static uint32_t msec_prev;
    uint32_t msec_curr = millis();

    if (msec_curr-msec_prev > 250) {

        msec_prev = msec_curr;

        reportAcceleration("X", ax);
        reportAcceleration("Y", ay);
        reportAcceleration("Z", az);

        Serial.println();

        reportGyroRate("X", gx);
        reportGyroRate("Y", gy);
        reportGyroRate("Z", gz);

        Serial.println();;

        reportMagnetometer("X", mx);
        reportMagnetometer("Y", my);
        reportMagnetometer("Z", mz);

        Serial.println();;

        // Print temperature in degrees Centigrade      
        Serial.print("Gyro temperature is ");  
        Serial.print(temperature, 1);  
        Serial.println(" degrees C\n"); 
    }
}



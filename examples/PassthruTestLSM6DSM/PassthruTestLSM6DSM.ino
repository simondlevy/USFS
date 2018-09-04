/* 
   PassthruTestLSM6DSM.ino: Example sketch for running LSM6DSM SENtral 
   sensor hub in pass-through mode to talk directly to LSM6DSM

   Copyright (C) 2018 Simon D. Levy

   Additional dependencies:

       https://github.com/simondlevy/LSM

       https://github.com/simondlevy/CrossPlatformDataBus

   This file is part of LSM6DSM.

   LSM6DSM is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   LSM6DSM is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with LSM6DSM.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "LSM6DSM.h"

#include <LSM6DSM.h>

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

static const  LSM6DSM::Ascale_t ASCALE = LSM6DSM::AFS_2G;
static const  LSM6DSM::Gscale_t GSCALE = LSM6DSM::GFS_250DPS;
static const  LSM6DSM::Grate_t  GRATE  = LSM6DSM::GODR_208Hz;

// Instantiate LSM6DSM class in master mode
static LSM6DSM lsm6dsm(ASCALE, GSCALE, GRATE);

static void lsm6dsm_error(const char * errmsg) 
{
    Serial.print("LSM6DSM Error: ");
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

void setup()
{
    Serial.begin(115200);

#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    // Start the LSM6DSM in master mode
    switch (lsm6dsm.begin()) {

        case LSM6DSM::ERROR_ID:
            lsm6dsm_error("Bad IMU device ID");
        case LSM6DSM::ERROR_SELFTEST:
            lsm6dsm_error("Failed self-test");
        default:
            Serial.println("LSM6DSM online!\n");
    }
}


void loop()
{  
    static float ax, ay, az, gx, gy, gz;

    lsm6dsm.readData(ax, ay, az, gx, gy, gz);

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
    }
}



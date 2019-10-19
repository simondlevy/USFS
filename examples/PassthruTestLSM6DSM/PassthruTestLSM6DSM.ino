/* 
   Example sketch for running USFS in pass-through mode to talk directly to
   LSM6DSM IMU and LIS2MDL magenetometer

   Copyright (C) 2018 Simon D. Levy

   Additional dependencies:

       https://github.com/simondlevy/LSM6DSM
       https://github.com/simondlevy/LIS2MDL
       https://github.com/simondlevy/CrossPlatformDataBus

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

#include "USFS.h"

#include <LSM6DSM.h>
#include <LIS2MDL.h>
#include <LPS22HB.h>

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

// LSM6DSM params
static const  LSM6DSM::Ascale_t ASCALE = LSM6DSM::AFS_2G;
static const  LSM6DSM::Gscale_t GSCALE = LSM6DSM::GFS_250DPS;
static const  LSM6DSM::Rate_t   RATE   = LSM6DSM::ODR_208Hz;

// LIS2MDL params
static const  LIS2MDL::Rate_t MRATE = LIS2MDL::ODR_50Hz;

// LPS22HB params
static const  LPS22HB::Rate_t BRATE = LPS22HB::ODR_50Hz;

// Instantiate LSM6DSM class
static LSM6DSM lsm6dsm(ASCALE, RATE, GSCALE, RATE);

// Instantiate LIS2MDL class
static LIS2MDL lis2mdl(MRATE);

// Instantiate LPS22HB class
static LPS22HB lps22hb(BRATE);

// Instantiate USFS class
USFS usfs;

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
#elif defined(ESP8266)
    Wire.begin(0,2); // SDA (0), SCL (2) on ESP8266 
#else
    Wire.begin();
#endif

    delay(100);

    // Start the USFS
    if (!usfs.begin()) {

        while (true) {
            Serial.println(usfs.getErrorString());
        }
    }    

    delay(100);

    // Put the USFS into pass-through mode
    usfs.setPassThroughMode();

    delay(100);

    // Start the LSM6DSM
    if (lsm6dsm.begin()) {
        Serial.println("LSM6DSM online!\n");
    }
    else {
        Serial.println("Unable to connect to LSM6DSM");
        while (true) ;
    }

    // Start the LIS2MDL
    if (lis2mdl.begin()) {
        Serial.println("LIS2MDL online!\n");
    }
    else {
        Serial.println("Unable to connect to LIS2MDL");
        while (true) ;
    }

    // Start the LPS22HB
    if (lps22hb.begin()) {
        Serial.println("LPS22HB online!\n");
    }
    else {
        Serial.println("Unable to connect to LPS22HB");
        while (true) ;
    }
}


void loop()
{  
    static float ax, ay, az, gx, gy, gz, mx, my, mz, pressure, temperature;

    // Read from LSM6DSM
    if (lsm6dsm.checkNewData()) {
        lsm6dsm.readData(ax, ay, az, gx, gy, gz);
    }

    // Read from LIS2MDL
    if (lis2mdl.checkNewData()) {
        lis2mdl.readData(mx, my, mz);
    }

    // Read from LPS22HB
    if (lps22hb.checkNewData()) {
        pressure = lps22hb.readPressure();
        temperature = lps22hb.readTemperature();
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

        Serial.println();

        reportMagnetometer("X", mx);
        reportMagnetometer("Y", my);
        reportMagnetometer("Z", mz);

        Serial.println();

        Serial.print("Pressure: ");
        Serial.print(pressure);
        Serial.println(" mBar");

        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" C\n");

    }
}



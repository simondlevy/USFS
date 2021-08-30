/* 
   Example sketch for running USFS in master mode.

   Copyright (C) 2018 Simon D. Levy

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

#include "USFS_Master.h"

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

 
USFS_Master usfs;

void setup()
{
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#elif defined(ESP8266)
    Wire.begin(0,2); // SDA (0), SCL (2) on ESP8266 
#else
    Wire.begin();
#endif

    delay(100);

    Serial.begin(115200);

    // Start the USFS in master mode
    if (!usfs.begin()) {

        while (true) {
            Serial.println(usfs.getErrorString());
        }
    }
}

void loop()
{  
    usfs.checkEventStatus();

    if (usfs.gotError()) {
        Serial.print("ERROR: ");
        Serial.println(usfs.getErrorString());
        return;
    }

    /*
       Define output variables from updated quaternion---these are Tait-Bryan
       angles, commonly used in aircraft orientation.  In this coordinate
       system, the positive z-axis is down toward Earth.  Yaw is the angle
       between Sensor x-axis and Earth magnetic North (or true North if
       corrected for local declination, looking down on the sensor positive
       yaw is counterclockwise.  Pitch is angle between sensor x-axis and
       Earth ground plane, toward the Earth is positive, up toward the sky is
       negative.  Roll is angle between sensor y-axis and Earth ground plane,
       y-axis up is positive roll.  These arise from the definition of the
       homogeneous rotation matrix constructed from q.  Tait-Bryan
       angles as well as Euler angles are non-commutative; that is, the get
       the correct orientation the rotations must be applied in the correct
       order which for this configuration is yaw, pitch, and then roll.  For
       more see http://en.wikipedia.org/wiki/Conversion_between_q_and_Euler_angles 
       which has additional links.
     */

    if (usfs.gotQuaternion()) {

        float qw, qx, qy, qz;

        usfs.readQuaternion(qw, qx, qy, qz);

        float roll  = atan2(2.0f * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
        float pitch = -asin(2.0f * (qx * qz - qw * qy));
        float yaw   = atan2(2.0f * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);   

        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI; 
        yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        if(yaw < 0) yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
        roll  *= 180.0f / PI;

        Serial.print("Quaternion Roll, Pitch, Yaw: ");
        Serial.print(roll, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(yaw, 2);
    }

    if (usfs.gotAccelerometer()) {
        float ax, ay, az;
        usfs.readAccelerometer(ax, ay, az);
        
        Serial.print("Accel: ");
        Serial.print(ax);
        Serial.print(", ");
        Serial.print(ay);
        Serial.print(", ");
        Serial.println(az);
    }

    if (usfs.gotGyrometer()) {
        float gx, gy, gz;
        usfs.readGyrometer(gx, gy, gz);

        Serial.println(gx);
        Serial.print("Gyro: ");
        Serial.print(gx);
        Serial.print(", ");
        Serial.print(gy);
        Serial.print(", ");
        Serial.println(gz);
    }

    if (usfs.gotMagnetometer()) {
        
        float mx, my, mz;
        usfs.readMagnetometer(mx, my, mz);

        Serial.print("Mag: ");
        Serial.print(mx);
        Serial.print(", ");
        Serial.print(my);
        Serial.print(", ");
        Serial.println(mz);
    }

     /*
       Or define output variable according to the Android system, where
       heading (0 to 360) is defined by the angle between the y-axis and True
       North, pitch is rotation about the x-axis (-180 to +180), and roll is
       rotation about the y-axis (-90 to +90) In this systen, the z-axis is
       pointing away from Earth, the +y-axis is at the "top" of the device
       (cellphone) and the +x-axis points toward the right of the device.
     */ 

    if (usfs.gotBarometer()) 
    {
        float temperature, pressure;

        usfs.readBarometer(pressure, temperature);

        Serial.println("Baro:");
        Serial.print("  Altimeter temperature = "); 
        Serial.print( temperature, 2); 
        Serial.println(" C"); 
        Serial.print("  Altimeter pressure = "); 
        Serial.print(pressure, 2);  
        Serial.println(" mbar");
        float altitude = (1.0f - powf(pressure / 1013.25f, 0.190295f)) * 44330.0f;
        Serial.print("  Altitude = "); 
        Serial.print(altitude, 2); 
        Serial.println(" m\n");
    }

    delay(100);
}



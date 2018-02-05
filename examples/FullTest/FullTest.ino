/* 
   FullTest.ino: Example sketch for running EM7180 SENtral sensor hub in master mode.

   Adapted from

     https://github.com/kriswiner/EM7180_SENtral_sensor_hub/tree/master/WarmStartandAccelCal

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

void setup()
{
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    Serial.begin(38400);

    // Start the EM7180 in master mode, polling instead of interrupt
    if (!em7180.begin(8, 2000, 1000)) {

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

    if (em7180.gotQuaternions()) {

        float q[4];

        em7180.readQuaternions(q);

        float yaw   = atan2(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);   
        float pitch = -asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
        float roll  = atan2(2.0f * (q[3] * q[0] + q[1] * q[2]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI; 
        yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        if(yaw < 0) yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
        roll  *= 180.0f / PI;

        Serial.print("Quaternion Yaw, Pitch, Roll: ");
        Serial.print(yaw, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(roll, 2);
    }

    if (em7180.gotAccelerometer()) {
        int16_t a[3];
        em7180.readAccelerometer(a);
        Serial.print("Accel: ");
        Serial.print(a[0]);
        Serial.print(", ");
        Serial.print(a[1]);
        Serial.print(", ");
        Serial.println(a[2]);
    }

    if (em7180.gotGyrometer()) {
        int16_t g[3];
        em7180.readGyrometer(g);
        Serial.print("Gyro: ");
        Serial.print(g[0]);
        Serial.print(", ");
        Serial.print(g[1]);
        Serial.print(", ");
        Serial.println(g[2]);
    }

    /*
       Or define output variable according to the Android system, where
       heading (0 to 360) is defined by the angle between the y-axis and True
       North, pitch is rotation about the x-axis (-180 to +180), and roll is
       rotation about the y-axis (-90 to +90) In this systen, the z-axis is
       pointing away from Earth, the +y-axis is at the "top" of the device
       (cellphone) and the +x-axis points toward the right of the device.
     */ 

    if (em7180.gotBarometer()) 
    {
        float temperature, pressure;

        em7180.readBarometer(pressure, temperature);

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
}



/* 
   FullTest.ino: Example sketch for running EM7180 SENtral sensor hub in normal mode.

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/em7180.MPU9250_BMP280

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

#include <i2c_t3.h>

EM7180 em7180;

void setup()
{
    // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);

    delay(100);

    Serial.begin(38400);

    // Start the EM710
    uint8_t status = em7180.begin(8, 2000, 1000);
    while (status) {
        Serial.println(EM7180::errorToString(status));
    }
}

void loop()
{  
    static int count, sumCount;
    static uint32_t lastUpdate; // used to calculate integration interval
    static float sum;
    static float yaw, pitch, roll;

    static float quaternions[4];

    uint8_t errorStatus = em7180.update();

    if (errorStatus) {
        Serial.print("ERROR: ");
        Serial.println(EM7180::errorToString(errorStatus));
        return;
    }

    em7180.getQuaternions(quaternions);

    // keep track of rates
    uint32_t Now = micros();
    float deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;

    // Serial print and/or display at 0.5 s rate independent of data rates
    uint32_t delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

        Serial.println("Hardware quaternions:"); 
        Serial.print("Q0 = ");
        Serial.print(quaternions[0]);
        Serial.print(" Qx = ");
        Serial.print(quaternions[1]); 
        Serial.print(" Qy = ");
        Serial.print(quaternions[2]); 
        Serial.print(" Qz = ");
        Serial.println(quaternions[3]); 

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
           homogeneous rotation matrix constructed from quaternions.  Tait-Bryan
           angles as well as Euler angles are non-commutative; that is, the get
           the correct orientation the rotations must be applied in the correct
           order which for this configuration is yaw, pitch, and then roll.  For
           more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
           which has additional links.
         */

        // AHRS:
        float Yaw   = atan2(2.0f * (quaternions[0] * quaternions[1] + quaternions[3] * quaternions[2]), quaternions[3] * quaternions[3] + quaternions[0] * quaternions[0] - quaternions[1] * quaternions[1] - quaternions[2] * quaternions[2]);   
        float Pitch = -asin(2.0f * (quaternions[0] * quaternions[2] - quaternions[3] * quaternions[1]));
        float Roll  = atan2(2.0f * (quaternions[3] * quaternions[0] + quaternions[1] * quaternions[2]), quaternions[3] * quaternions[3] - quaternions[0] * quaternions[0] - quaternions[1] * quaternions[1] + quaternions[2] * quaternions[2]);
        Pitch *= 180.0f / PI;
        Yaw   *= 180.0f / PI; 
        Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        if(Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
        Roll  *= 180.0f / PI;

        /*
           Or define output variable according to the Android system, where
           heading (0 to 360) is defined by the angle between the y-axis and True
           North, pitch is rotation about the x-axis (-180 to +180), and roll is
           rotation about the y-axis (-90 to +90) In this systen, the z-axis is
           pointing away from Earth, the +y-axis is at the "top" of the device
           (cellphone) and the +x-axis points toward the right of the device.
         */ 

        Serial.print("Hardware Yaw, Pitch, Roll: ");
        Serial.print(Yaw, 2);
        Serial.print(", ");
        Serial.print(Pitch, 2);
        Serial.print(", ");
        Serial.println(Roll, 2);

        float temperature, pressure;

        em7180.getBaro(pressure, temperature);

        Serial.println("BMP280:");
        Serial.print("Altimeter temperature = "); 
        Serial.print( temperature, 2); 
        Serial.println(" C"); // temperature in degrees Celsius
        Serial.print("Altimeter temperature = "); 
        Serial.print(9.*temperature/5. + 32., 2); 
        Serial.println(" F"); // temperature in degrees Fahrenheit
        Serial.print("Altimeter pressure = "); 
        Serial.print(pressure, 2);  
        Serial.println(" mbar");// pressure in millibar
        float altitude = 145366.45f*(1.0f - pow((pressure/1013.25f), 0.190284f));
        Serial.print("Altitude = "); 
        Serial.print(altitude, 2); 
        Serial.println(" feet");
        Serial.println(" ");

        Serial.print((float)sumCount/sum, 2);
        Serial.println(" Hz");
        Serial.print(millis()/1000.0, 1);Serial.print(",");
        Serial.print(yaw);
        Serial.print(",");Serial.print(pitch);
        Serial.print(",");Serial.print(roll);
        Serial.print(",");
        Serial.print(Yaw);
        Serial.print(",");Serial.print(Pitch);
        Serial.print(",");Serial.println(Roll);  

        count = millis(); 
        sumCount = 0;
        sum = 0;    
    }
}



/* 
   USFS interrupt example

   Copyright (C) 2022 TleraCorp && Simon D. Levy

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

#include <math.h>

#include <Wire.h>

#include "USFS.h"

static const uint8_t LED_PIN       = 18; 
static const uint8_t INTERRUPT_PIN = 12; 

static const uint8_t AccBW = 0x03;
static const uint8_t GyroBW = 0x03;
static const uint8_t QRtDiv = 0x01;
static const uint8_t MagRt = 0x64;
static const uint8_t AccRt = 0x14;
static const uint8_t GyroRt = 0x14;
static const uint8_t BaroRt = 0x32;

static const uint16_t AccFS  = 0x0008;
static const uint16_t GyroFS = 0x07D0;
static const uint16_t MagFS  = 0x03E8;

static volatile bool _gotNewData;

static void interruptHandler()
{
    _gotNewData = true;
}

void setup()
{
    Serial.begin(115200);
    delay(4000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); 

    Wire.begin(TWI_PINS_20_21); 
    Wire.setClock(400000); 
    delay(1000);

    usfsReportChipId();        

    usfsLoadFirmware(); 

    usfsBegin(AccBW, GyroBW, AccFS, GyroFS, MagFS, QRtDiv, MagRt, AccRt, GyroRt, BaroRt); 

    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);  

    // Clear interrupts
    usfsCheckStatus();

    Serial.println("Enter '1' to proceed...");
    while (true) {
        if (Serial.read() == '1') {
            break;
        }
        delay(10);
    }
}

/* End of setup */

void loop() {

    static uint32_t _interruptCount;

    static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    
    static int16_t rawPressure, rawTemperature;            
    static float Temperature, Pressure, Altitude; 
    static float ax, ay, az, gx, gy, gz, mx, my, mz; 

    if (_gotNewData == true) { 

        _gotNewData = false;  

        _interruptCount++;

        uint8_t eventStatus = usfsCheckStatus(); 

        if (usfsEventStatusIsError(eventStatus)) { 

            switch (usfsCheckErrors()) {

                case USFS_ERROR_MAGNETOMETER:
                    Serial.println("Magnetometer failure!");
                    break;

                case USFS_ERROR_ACCELEROMETER:
                    Serial.println("Accelerometer failure!");
                    break;
                
                case USFS_ERROR_GYROMETER:
                    Serial.println("Gyro failure!");
                    break;
                
                case USFS_ERROR_MAGNETOMETER_INIT:
                    Serial.println("Magnetometer initialization failure!");
                    break;
               
                case USFS_ERROR_ACCELEROMETER_INIT:
                    Serial.println("Accelerometer initialization failure!");
                    break;
               
                case USFS_ERROR_GYROMETER_INIT:
                    Serial.println("Gyro initialization failure!");
                    break;
              
                case USFS_ERROR_MATH:
                    Serial.println("Math error!");
                    break;
            
                case USFS_ERROR_SAMPLE_RATE:
                    Serial.println("Invalid sample rate!");
                    break;
            }
        }

        if (eventStatus & 0x10) { 

            int16_t accelCount[3] = {};  

            usfsReadAccelerometer(accelCount);

            ax = (float)accelCount[0] * 0.000488f; 
            ay = (float)accelCount[1] * 0.000488f;
            az = (float)accelCount[2] * 0.000488f;
        }

        if (eventStatus & 0x20) { 

            int16_t gyroCount[3] = {};  

            usfsReadGyrometer(gyroCount);

            gx = (float)gyroCount[0] * 0.153f; 
            gy = (float)gyroCount[1] * 0.153f;
            gz = (float)gyroCount[2] * 0.153f;
        }

        if (eventStatus & 0x08) { 

            int16_t magCount[3] = {};  

            usfsreadMagnetometer(magCount);

            mx = (float)magCount[0] * 0.305176f; 
            my = (float)magCount[1] * 0.305176f;
            mz = (float)magCount[2] * 0.305176f;
        }

        if (eventStatus & 0x04) { 
            usfsReadQuaternion(q);
        }


        if (eventStatus & 0x40) { 

            rawPressure = usfsReadBarometer();
            Pressure = (float)rawPressure * 0.01f + 1013.25f; 

            rawTemperature = usfsReadTemperature();
            Temperature = (float) rawTemperature * 0.01f; 
        }
    } 

    static uint32_t _msec;

    uint32_t msec = millis();

    static float yaw, pitch, roll;

    if (msec-_msec > 500) { 

        Serial.print("Interrupts: ");
        Serial.println(_interruptCount);

        _msec = msec;

        Serial.print("Ax = ");
        Serial.print((int)1000 * ax);
        Serial.print(" Ay = ");
        Serial.print((int)1000 * ay);
        Serial.print(" Az = ");
        Serial.print((int)1000 * az);
        Serial.println(" mg");
        Serial.print("Gx = ");
        Serial.print( gx, 2);
        Serial.print(" Gy = ");
        Serial.print( gy, 2);
        Serial.print(" Gz = ");
        Serial.print( gz, 2);
        Serial.println(" deg/s");
        Serial.print("Mx = ");
        Serial.print( (int)mx);
        Serial.print(" My = ");
        Serial.print( (int)my);
        Serial.print(" Mz = ");
        Serial.print( (int)mz);
        Serial.println(" mG");

        Serial.println("Hardware quaternions:");
        Serial.print("Q0 = ");
        Serial.print(q[0]);
        Serial.print(" Qx = ");
        Serial.print(q[1]);
        Serial.print(" Qy = ");
        Serial.print(q[2]);
        Serial.print(" Qz = ");
        Serial.println(q[3]);


        float A12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
        float A22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        float A31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
        float A32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
        float A33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
        pitch = -asinf(A32);
        roll  = atan2f(A31, A33);
        yaw   = atan2f(A12, A22);
        pitch *= 180.0f / M_PI;
        yaw   *= 180.0f / M_PI;
        yaw   += 13.8f; 
        if (yaw < 0) yaw   += 360.0f ; 
        roll  *= 180.0f / M_PI;

        Serial.print("Hardware yaw, pitch, roll: ");
        Serial.print(yaw, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(roll, 2);

        Serial.print("Hardware Grav_x, Grav_y, Grav_z: ");
        Serial.print(-A31 * 1000, 2);
        Serial.print(", ");
        Serial.print(-A32 * 1000, 2);
        Serial.print(", ");
        Serial.print(A33 * 1000, 2);
        Serial.println(" mg");
        Serial.print("Hardware ax, ay, az: ");
        Serial.print(ax * 1000, 2);
        Serial.print(", ");
        Serial.print(ay * 1000, 2);
        Serial.print(", ");
        Serial.print(az * 1000, 2);
        Serial.println(" mg");

        Serial.println("MS5637:");
        Serial.print("Altimeter temperature = ");
        Serial.print(Temperature, 2);
        Serial.println(" C"); 
        Serial.print("Altimeter temperature = ");
        Serial.print(9.0f * Temperature / 5.0f + 32.0f, 2);
        Serial.println(" F"); 
        Serial.print("Altimeter pressure = ");
        Serial.print(Pressure, 2);
        Serial.println(" mbar");
        Altitude = 145366.45f * (1.0f - pow(((Pressure) / 1013.25f), 0.190284f));
        Serial.print("Altitude = ");
        Serial.print(Altitude, 2);
        Serial.println(" feet");
        Serial.println(" ");
    } 

    digitalWrite(LED_PIN, LOW); delay(10); digitalWrite(LED_PIN, HIGH);  
    STM32.sleep();

}  


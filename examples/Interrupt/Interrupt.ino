/* 
   Interrupt.ino: Example sketch for running EM7180 SENtral sensor hub with interrupts.

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/em7180.MPU9250_BMP280

     https://github.com/kriswiner/EM7180_SENtral_sensor_hub/wiki/E.-Polling-versus-Interrupts:-Reading-Data-from-the-EM7180

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

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

#include <EM7180.h>

EM7180 em7180;


#define EM7180_QX                 0x00  // this is a 32-bit normalized floating point number read from registers 0x00-03
#define EM7180_AX                 0x1A  // int16_t from registers 0x1A-1B
#define EM7180_GX                 0x22  // int16_t from registers 0x22-23
#define EM7180_MX                 0x12  // int16_t from registers 0x12-13
#define EM7180_Baro               0x2A  // start of two-byte MS5637 pressure data, 16-bit signed interger
#define EM7180_Temp               0x2E  // start of two-byte MS5637 temperature data, 16-bit signed interger
#define EM7180_EventStatus        0x35
#define EM7180_ErrorRegister      0x50

#define EM7180_ADDRESS           0x28   // Address of the EM7180 SENtral sensor hub

// Pin definitions
//static int intPin = 30;  // On Teensy Flight Controller
static int intPin = 12;  // On Ladybug Flight Controller

// MPU9250 variables
static int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
static int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
static int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
static int16_t rawPressure, rawTemperature;   // pressure, temperature raw count output
static float   temperature, pressure; // Stores the MPU9250 internal chip temperature in degrees Celsius

static uint8_t param[4];                         // used for param transfer
static uint16_t EM7180_mag_fs, EM7180_acc_fs, EM7180_gyro_fs; // EM7180 sensor full scale ranges

static float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 

// I2C read/write functions for the MPU9250 and AK8963 sensors

static uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data	 
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);	                 // Put slave register address in Tx buffer
    Wire.endTransmission(NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    //	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    //	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
    Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
    //	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    //        Wire.requestFrom(address, count);  // Read bytes from slave register address 
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

static float uint32_reg_to_float (uint8_t *buf)
{
    union {
        uint32_t ui32;
        float f;
    } u;

    u.ui32 =     (((uint32_t)buf[0]) +
            (((uint32_t)buf[1]) <<  8) +
            (((uint32_t)buf[2]) << 16) +
            (((uint32_t)buf[3]) << 24));
    return u.f;
}

static void readSENtralQuatData(float * destination)
{
    uint8_t rawData[16];  // x/y/z quaternion register data stored here
    readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]);       // Read the sixteen raw data registers into data array
    destination[0] = uint32_reg_to_float (&rawData[0]);
    destination[1] = uint32_reg_to_float (&rawData[4]);
    destination[2] = uint32_reg_to_float (&rawData[8]);
    destination[3] = uint32_reg_to_float (&rawData[12]);  // SENtral stores quats as qx, qy, qz, q0!

}

static void readSENtralAccelData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]);       // Read the six raw data registers into data array
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

static void readSENtralGyroData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

static void readSENtralMagData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

static int16_t readSENtralBaroData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    readBytes(EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}

static int16_t readSENtralTempData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    readBytes(EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}


// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

volatile bool newData;

void myinthandler()
{
    newData = true;
}

// ========================================================================================================

void setup()
{
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    Serial.begin(115200);

    // Start the EM710
    uint8_t status = em7180.begin(8, 2000, 1000);
    while (status) {
        Serial.println(EM7180::errorToString(status));
    }

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for INT pin output of EM7180

    // Check event status register to clear the EM7180 interrupt before the main loop
    readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register and interrupt

    Serial.println("Here we go ...");
}


void loop()
{  
    //If intPin goes high, the EM7180 has new data
    if (newData) {  // On interrupt, read data

        Serial.println("new data");
        newData = false;  // reset newData flag

        // Check event status register, after receipt of interrupt
        uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

        // Check for errors
        if (eventStatus & 0x02) { // error detected, what is it?

            uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
            if (errorStatus != 0x00) { // non-zero value indicates error, what is it?
                Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
                if (errorStatus == 0x11) Serial.print("Magnetometer failure!");
                if (errorStatus == 0x12) Serial.print("Accelerometer failure!");
                if (errorStatus == 0x14) Serial.print("Gyro failure!");
                if (errorStatus == 0x21) Serial.print("Magnetometer initialization failure!");
                if (errorStatus == 0x22) Serial.print("Accelerometer initialization failure!");
                if (errorStatus == 0x24) Serial.print("Gyro initialization failure!");
                if (errorStatus == 0x30) Serial.print("Math error!");
                if (errorStatus == 0x80) Serial.print("Invalid sample rate!");
            }

            // Handle errors ToDo

        }

        // if no errors, see if new data is ready
        if (eventStatus & 0x10) { // new acceleration data available
            readSENtralAccelData(accelCount);

            Serial.println("accel");

            // Now we'll calculate the acceleration value into actual g's
            ax = (float)accelCount[0]*0.000488;  // get actual g value
            ay = (float)accelCount[1]*0.000488;    
            az = (float)accelCount[2]*0.000488;  

            Serial.print(ax); Serial.print("\t");
            Serial.print(ay); Serial.print("\t");
            Serial.print(az); Serial.println();
        }

        if (eventStatus & 0x20) { // new gyro data available
            readSENtralGyroData(gyroCount);

            Serial.println("gyro");

            // Now we'll calculate the gyro value into actual dps's
            gx = (float)gyroCount[0]*0.153;  // get actual dps value
            gy = (float)gyroCount[1]*0.153;    
            gz = (float)gyroCount[2]*0.153;  
        }

        if (eventStatus & 0x08) { // new mag data available
            readSENtralMagData(magCount);

            Serial.println("mag");

            // Now we'll calculate the mag value into actual G's
            mx = (float)magCount[0]*0.305176;  // get actual G value
            my = (float)magCount[1]*0.305176;    
            mz = (float)magCount[2]*0.305176;  
        }

        if (eventStatus & 0x04) { // new quaternion data available

            Serial.println("quat");

            float Q[4];
            readSENtralQuatData(Q); 
        }

        // get MS5637 pressure
        if (eventStatus & 0x40) { // new baro data available

            Serial.println("baro");

            rawPressure = readSENtralBaroData();
            pressure = (float)rawPressure*0.01f +1013.25f; // pressure in mBar

            // get MS5637 temperature
            rawTemperature = readSENtralTempData();  
            temperature = (float) rawTemperature*0.01;  // temperature in degrees C
        }
    }
}

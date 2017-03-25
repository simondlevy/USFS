/* 
   PassThru.ino: Example sketch for running EM7180 SENtral sensor hub in pass-through mode.

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

#include "quaternionFilters.h"
#include "EM7180.h"

#include <i2c_t3.h>

// Pin definitions
static const int myLed     = 28;  // LED on the Teensy 3.1

static float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias

// Bias corrections for gyro, accelerometer, mag
static float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};  

static uint16_t dig_T1, dig_P1;
static int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static void magcalMPU9250(float * dest1, float * dest2) 
{
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {0xFF, 0xFF, 0xFF}, mag_min[3] = {0x7F, 0x7F, 0x7F}, mag_temp[3] = {0, 0, 0};

    if(Mmode == 0x02) sample_count = 128;
    if(Mmode == 0x06) sample_count = 1500;
    for(ii = 0; ii < sample_count; ii++) {
        readMagData(mag_temp);  // Read the mag data   
        for (int jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
        if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];  

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

}

static void BMP280Init()
{
    // Configure the BMP280
    // Set T and P oversampling rates and sensor mode
    writeByte(BMP280_ADDRESS, BMP280_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
    // Set standby time interval in normal mode and bandwidth
    writeByte(BMP280_ADDRESS, BMP280_CONFIG, SBy << 5 | IIRFilter << 2);
    // Read and store calibration data
    uint8_t calib[24];
    readBytes(BMP280_ADDRESS, BMP280_CALIB00, 24, &calib[0]);
    dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
    dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
    dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
    dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
    dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
    dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
    dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
    dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
    dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
    dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
    dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
    dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
static int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t bmp280_compensate_P(int32_t adc_P)
{
    long long var1, var2, p;
    var1 = ((long long)t_fine) - 128000;
    var2 = var1 * var1 * (long long)dig_P6;
    var2 = var2 + ((var1*(long long)dig_P5)<<17);
    var2 = var2 + (((long long)dig_P4)<<35);
    var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
    var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
    if(var1 == 0)
    {
        return 0;
        // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125)/var1;
    var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((long long)dig_P8) * p)>> 19;
    p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
    return (uint32_t)p;
}




void setup()
{
    int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
    int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
    int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

    // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    delay(1000);
    Serial.begin(38400);

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, LOW);

    // Read SENtral device information
    uint16_t ROM1 = readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
    uint16_t ROM2 = readByte(EM7180_ADDRESS, EM7180_ROMVersion2);
    Serial.print("EM7180 ROM Version: 0x");
    Serial.print(ROM1, HEX);
    Serial.println(ROM2, HEX);
    Serial.println("Should be: 0xE609");
    uint16_t RAM1 = readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
    uint16_t RAM2 = readByte(EM7180_ADDRESS, EM7180_RAMVersion2);
    Serial.print("EM7180 RAM Version: 0x");
    Serial.print(RAM1);
    Serial.println(RAM2);
    uint8_t PID = readByte(EM7180_ADDRESS, EM7180_ProductID);
    Serial.print("EM7180 ProductID: 0x");
    Serial.print(PID, HEX);
    Serial.println(" Should be: 0x80");
    uint8_t RID = readByte(EM7180_ADDRESS, EM7180_RevisionID);
    Serial.print("EM7180 RevisionID: 0x");
    Serial.print(RID, HEX);
    Serial.println(" Should be: 0x02");

    delay(1000); // give some time to read the screen

    // Check which sensors can be detected by the EM7180
    uint8_t featureflag = readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
    if(featureflag & 0x01)  Serial.println("A barometer is installed");
    if(featureflag & 0x02)  Serial.println("A humidity sensor is installed");
    if(featureflag & 0x04)  Serial.println("A temperature sensor is installed");
    if(featureflag & 0x08)  Serial.println("A custom sensor is installed");
    if(featureflag & 0x10)  Serial.println("A second custom sensor is installed");
    if(featureflag & 0x20)  Serial.println("A third custom sensor is installed");

    delay(1000); // give some time to read the screen

    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    byte STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
    int count = 0;
    while(!STAT) {
        writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
        delay(500);  
        count++;  
        STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
        if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
        if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
        if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
        if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
        if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
        if(count > 10) break;
    }

    if(!(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04))  Serial.println("EEPROM upload successful!");
    delay(1000); // give some time to read the screen


    // Put EM7180 SENtral into pass-through mode
    SENtralPassThroughMode();
    delay(1000);

    // Read first page of EEPROM
    uint8_t data[128];
    M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x00, 0x00, 128, data);
    Serial.println("EEPROM Signature Byte"); 
    Serial.print(data[0], HEX);
    Serial.println("  Should be 0x2A");
    Serial.print(data[1], HEX);
    Serial.println("  Should be 0x65");
    for (int i = 0; i < 128; i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    // Read the WHO_AM_I register, this is a good test of communication
    Serial.println("MPU9250 9-axis motion sensor...");
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    Serial.print("MPU9250 ");
    Serial.print("I AM ");
    Serial.print(c, HEX);
    Serial.print(" I should be ");
    Serial.println(0x71, HEX);
    if (c == 0x71) // WHO_AM_I should always be 0x71
    {  
        Serial.println("MPU9250 is online...");

        MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
        Serial.print("x-axis self test: acceleration trim within : ");
        Serial.print(SelfTest[0],1);
        Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : ");
        Serial.print(SelfTest[1],1);
        Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : ");
        Serial.print(SelfTest[2],1);
        Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : ");
        Serial.print(SelfTest[3],1);
        Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : ");
        Serial.print(SelfTest[4],1);
        Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : ");
        Serial.print(SelfTest[5],1);
        Serial.println("% of factory value");
        delay(1000);

        // get sensor resolutions, only need to do this once
        getAres();
        getGres();
        getMres();

        Serial.println(" Calibrate gyro and accel");
        accelgyrocalMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        Serial.println("accel biases (mg)");
        Serial.println(1000.*accelBias[0]);
        Serial.println(1000.*accelBias[1]);
        Serial.println(1000.*accelBias[2]);
        Serial.println("gyro biases (dps)");
        Serial.println(gyroBias[0]);
        Serial.println(gyroBias[1]);
        Serial.println(gyroBias[2]);

        delay(1000);  

        initMPU9250(); 
        Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22); 

        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
        Serial.print("AK8963 ");
        Serial.print("I AM ");
        Serial.print(d, HEX);
        Serial.print(" I should be ");
        Serial.println(0x48, HEX);

        delay(1000); 

        // Get magnetometer calibration from AK8963 ROM
        initAK8963(magCalibration);
        Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

        Serial.println("Mag Calibration: Wave device in a figure eight until done!");
        delay(4000);
        magcalMPU9250(magBias, magScale);
        Serial.println("Mag Calibration done!");

        Serial.println("AK8963 mag biases (mG)");
        Serial.println(magBias[0]);
        Serial.println(magBias[1]);
        Serial.println(magBias[2]); 
        Serial.println("AK8963 mag scale (mG)");
        Serial.println(magScale[0]);
        Serial.println(magScale[1]);
        Serial.println(magScale[2]); 
        delay(2000); // add delay to see results before serial spew of data

        //  Serial.println("Calibration values: ");
        Serial.print("X-Axis sensitivity adjustment value ");
        Serial.println(magCalibration[0], 2);
        Serial.print("Y-Axis sensitivity adjustment value ");
        Serial.println(magCalibration[1], 2);
        Serial.print("Z-Axis sensitivity adjustment value ");
        Serial.println(magCalibration[2], 2);

        delay(1000);  

        // Read the WHO_AM_I register of the BMP280 this is a good test of communication
        byte f = readByte(BMP280_ADDRESS, BMP280_ID);  // Read WHO_AM_I register for BMP280
        Serial.print("BMP280 "); 
        Serial.print("I AM "); 
        Serial.print(f, HEX); 
        Serial.print(" I should be "); 
        Serial.println(0x58, HEX);
        Serial.println(" ");

        delay(1000); 

        writeByte(BMP280_ADDRESS, BMP280_RESET, 0xB6); // reset BMP280 before initilization
        delay(100);

        BMP280Init(); // Initialize BMP280 altimeter
        Serial.println("Calibration coeficients:");
        Serial.print("dig_T1 ="); 
        Serial.println(dig_T1);
        Serial.print("dig_T2 ="); 
        Serial.println(dig_T2);
        Serial.print("dig_T3 ="); 
        Serial.println(dig_T3);
        Serial.print("dig_P1 ="); 
        Serial.println(dig_P1);
        Serial.print("dig_P2 ="); 
        Serial.println(dig_P2);
        Serial.print("dig_P3 ="); 
        Serial.println(dig_P3);
        Serial.print("dig_P4 ="); 
        Serial.println(dig_P4);
        Serial.print("dig_P5 ="); 
        Serial.println(dig_P5);
        Serial.print("dig_P6 ="); 
        Serial.println(dig_P6);
        Serial.print("dig_P7 ="); 
        Serial.println(dig_P7);
        Serial.print("dig_P8 ="); 
        Serial.println(dig_P8);
        Serial.print("dig_P9 ="); 
        Serial.println(dig_P9);

        delay(1000);  

    }
    else
    {
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);
        while(1) ; // Loop forever if communication doesn't happen
    }
}



void loop()
{  
    int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
    int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
    int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

    // If intPin goes high, all data registers have new data
    //  if (digitalRead(intACC2)) {  // On interrupt, read data
    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the acceleration value into actual g's
    float ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    float ay = (float)accelCount[1]*aRes - accelBias[1];   
    float az = (float)accelCount[2]*aRes - accelBias[2]; 

    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    float gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    float gy = (float)gyroCount[1]*gRes;  
    float gz = (float)gyroCount[2]*gRes;   

    //  if (digitalRead(intDRDYM)) {  // On interrupt, read data
    readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    float mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
    float my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
    float mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  

    // keep track of rates
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;

    // Sensors x (y)-axis of the accelerometer is aligned with the -y (x)-axis of the magnetometer;
    // the magnetometer z-axis (+ up) is aligned with z-axis (+ up) of accelerometer and gyro!
    // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
    // For the BMX-055, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    // in the MPU9250 sensor. This rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!  
    // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  mx,  my, mz, deltat, q);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

        Serial.print("ax = ");
        Serial.print((int)1000*ax);  
        Serial.print(" ay = ");
        Serial.print((int)1000*ay); 
        Serial.print(" az = ");
        Serial.print((int)1000*az);
        Serial.println(" mg");
        Serial.print("gx = ");
        Serial.print( gx, 2); 
        Serial.print(" gy = ");
        Serial.print( gy, 2); 
        Serial.print(" gz = ");
        Serial.print( gz, 2);
        Serial.println(" deg/s");
        Serial.print("mx = ");
        Serial.print( (int)mx); 
        Serial.print(" my = ");
        Serial.print( (int)my); 
        Serial.print(" mz = ");
        Serial.print( (int)mz);
        Serial.println(" mG");

        Serial.println("Software quaternions:"); 
        Serial.print("q0 = ");
        Serial.print(q[0]);
        Serial.print(" qx = ");
        Serial.print(q[1]); 
        Serial.print(" qy = ");
        Serial.print(q[2]); 
        Serial.print(" qz = ");
        Serial.println(q[3]); 
        rawPress =  readBMP280Pressure();
        pressure = (float) bmp280_compensate_P(rawPress)/25600.; // Pressure in mbar
        rawTemp =   readBMP280Temperature();
        temperature = (float) bmp280_compensate_T(rawTemp)/100.;


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
           more see
http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
which has additional links.
         */

        //Software AHRS:
        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI; 
        yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
        roll  *= 180.0f / PI;

        /*
           Or define output variable according to the Android system, where
           heading (0 to 360) is defined by the angle between the y-axis and True
           North, pitch is rotation about the x-axis (-180 to +180), and roll is
           rotation about the y-axis (-90 to +90) In this systen, the z-axis is
           pointing away from Earth, the +y-axis is at the "top" of the device
           (cellphone) and the +x-axis points toward the right of the device.
         */ 

        Serial.print("Software yaw, pitch, roll: ");
        Serial.print(yaw, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(roll, 2);

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
        altitude = 145366.45f*(1.0f - pow((pressure/1013.25f), 0.190284f));
        Serial.print("Altitude = "); 
        Serial.print(altitude, 2); 
        Serial.println(" feet");
        Serial.println(" ");

        //   Serial.print("rate = ");
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


        digitalWrite(myLed, !digitalRead(myLed));
        count = millis(); 
        sumCount = 0;
        sum = 0;    
    }

}



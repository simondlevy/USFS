/* EM7180_MPU9250_BMP280_t3 Basic Example Code
by: Kris Winer
date: September 11, 2015
license: Beerware - Use this code however you'd like. If you 
find it useful you can buy me a beer some time.

The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw
sensor data from a variety of motion sensors, in this case the MPU9250 (with
embedded MPU9250 + AK8963C), and does sensor fusion with quaternions as its
output. The SENtral loads firmware from the on-board M24512DRC 512 kbit EEPROM
upon startup, configures and manages the sensors on its dedicated master I2C
bus, and outputs scaled sensor data (accelerations, rotation rates, and
magnetic fields) as well as quaternions and heading/pitch/roll, if selected.

This sketch demonstrates basic EM7180 SENtral functionality including
parameterizing the register addresses, initializing the sensor, getting
properly scaled accelerometer, gyroscope, and magnetometer data out. Added
display functions to allow display to on breadboard monitor. Addition of 9 DoF
sensor fusion using open source Madgwick and Mahony filter algorithms to
compare with the hardware sensor fusion results.  Sketch runs on the 3.3 V 8
MHz Pro Mini and the Teensy 3.1.

This sketch is specifically for the Teensy 3.1 Mini Add-On shield with the
EM7180 SENtral sensor hub as master, the MPU9250 9-axis motion sensor
(accel/gyro/mag) as slave, a BMP280 pressure/temperature sensor, and an
M24512DRC 512kbit (64 kByte) EEPROM as slave all connected via I2C. The SENtral
can use the pressure data in the sensor fusion yet and there is a driver for
the BMP280 in the SENtral firmware. 

This sketch uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy
3.1-specific Wire library i2c_t3.h.  The BMP280 is a simple but high resolution
pressure sensor, which can be used in its high resolution mode but with power
consumption of 20 microAmp, or in a lower resolution mode with power
consumption of only 1 microAmp. The choice will depend on the application.

SDA and SCL should have external pull-up resistors (to 3.3V).
4k7 resistors are on the EM7180+MPU9250+BMP280+M24512DRC Mini Add-On board for Teensy 3.1.

Hardware setup:
EM7180 Mini Add-On ------- Teensy 3.1
VDD ---------------------- 3.3V
SDA ----------------------- 17
SCL ----------------------- 16
GND ---------------------- GND
INT------------------------ 8

Note: All the sensors n this board are I2C sensor and uses the Teensy 3.1 i2c_t3.h Wire library. 
Because the sensors are not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 */
#include "quaternionFilters.h"
#include "EM7180.h"

#include <i2c_t3.h>
#include <SPI.h>

static const bool SerialDebug = true;
static const bool passThru = false;


void setup()
{
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

    // Set up the SENtral as sensor bus in normal operating mode
    if(!passThru) {

        // Enter EM7180 initialized state
        writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
        writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off
        writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // Force initialize
        writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers

        //Setup LPF bandwidth (BEFORE setting ODR's)
        writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, 0x03); // 41Hz
        writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, 0x03); // 41Hz
        // Set accel/gyro/mage desired ODR rates
        writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02); // 100 Hz
        writeByte(EM7180_ADDRESS, EM7180_MagRate, 0x64); // 100 Hz
        writeByte(EM7180_ADDRESS, EM7180_AccelRate, 0x14); // 200/10 Hz
        writeByte(EM7180_ADDRESS, EM7180_GyroRate, 0x14); // 200/10 Hz
        writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | 0x32);  // set enable bit and set Baro rate to 25 Hz
        // writeByte(EM7180_ADDRESS, EM7180_TempRate, 0x19);  // set enable bit and set rate to 25 Hz

        // Configure operating mode
        writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
        // Enable interrupt to host upon certain events
        // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
        // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
        writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);
        // Enable EM7180 run mode
        writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
        delay(100);

        // EM7180 parameter adjustments
        Serial.println("Beginning Parameter Adjustments");

        // Read sensor default FS values from parameter space
        writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read parameter 74
        writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
        byte param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
        while(!(param_xfer==0x4A)) {
            param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
        }
        param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
        param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
        param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
        param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
        EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
        EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
        Serial.print("Magnetometer Default Full Scale Range: +/-");
        Serial.print(EM7180_mag_fs);
        Serial.println("uT");
        Serial.print("Accelerometer Default Full Scale Range: +/-");
        Serial.print(EM7180_acc_fs);
        Serial.println("g");
        writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
        while(!(param_xfer==0x4B)) {
            param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
        }
        param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
        param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
        param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
        param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
        EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
        Serial.print("Gyroscope Default Full Scale Range: +/-");
        Serial.print(EM7180_gyro_fs);
        Serial.println("dps");
        writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
        writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

        //Disable stillness mode
        EM7180_set_integer_param (0x49, 0x00);

        //Write desired sensor full scale ranges to the EM7180
        EM7180_set_mag_acc_FS (0x3E8, 0x08); // 1000 uT, 8 g
        EM7180_set_gyro_FS (0x7D0); // 2000 dps

        // Read sensor new FS values from parameter space
        writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read  parameter 74
        writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
        while(!(param_xfer==0x4A)) {
            param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
        }
        param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
        param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
        param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
        param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
        EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
        EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
        Serial.print("Magnetometer New Full Scale Range: +/-");
        Serial.print(EM7180_mag_fs);
        Serial.println("uT");
        Serial.print("Accelerometer New Full Scale Range: +/-");
        Serial.print(EM7180_acc_fs);
        Serial.println("g");
        writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
        while(!(param_xfer==0x4B)) {
            param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
        }
        param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
        param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
        param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
        param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
        EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
        Serial.print("Gyroscope New Full Scale Range: +/-");
        Serial.print(EM7180_gyro_fs);
        Serial.println("dps");
        writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
        writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm


        // Read EM7180 status
        uint8_t runStatus = readByte(EM7180_ADDRESS, EM7180_RunStatus);
        if(runStatus & 0x01) Serial.println(" EM7180 run status = normal mode");
        uint8_t algoStatus = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
        if(algoStatus & 0x01) Serial.println(" EM7180 standby status");
        if(algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
        if(algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
        if(algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
        if(algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
        if(algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");
        uint8_t passthruStatus = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
        if(passthruStatus & 0x01) Serial.print(" EM7180 in passthru mode!");
        uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus);
        if(eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
        if(eventStatus & 0x02) Serial.println(" EM7180 Error");
        if(eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
        if(eventStatus & 0x08) Serial.println(" EM7180 new mag result");
        if(eventStatus & 0x10) Serial.println(" EM7180 new accel result");
        if(eventStatus & 0x20) Serial.println(" EM7180 new gyro result"); 

        delay(1000); // give some time to read the screen

        // Check sensor status
        uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);
        Serial.print(" EM7180 sensor status = ");
        Serial.println(sensorStatus);
        if(sensorStatus & 0x01) reporterr("Magnetometer not acknowledging!");
        if(sensorStatus & 0x02) reporterr("Accelerometer not acknowledging!");
        if(sensorStatus & 0x04) reporterr("Gyro not acknowledging!");
        if(sensorStatus & 0x10) reporterr("Magnetometer ID not recognized!");
        if(sensorStatus & 0x20) reporterr("Accelerometer ID not recognized!");
        if(sensorStatus & 0x40) reporterr("Gyro ID not recognized!");

        Serial.print("Actual MagRate = ");
        Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualMagRate));
        Serial.println(" Hz"); 
        Serial.print("Actual AccelRate = ");
        Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualAccelRate));
        Serial.println(" Hz"); 
        Serial.print("Actual GyroRate = ");
        Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualGyroRate));
        Serial.println(" Hz"); 
        Serial.print("Actual BaroRate = ");
        Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualBaroRate));
        Serial.println(" Hz"); 
        //  Serial.print("Actual TempRate = ");
        Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualTempRate));
        Serial.println(" Hz"); 

        delay(1000); // give some time to read the screen

    }

    // If pass through mode desired, set it up here
    if(passThru) {

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

            if(SerialDebug) {
                //  Serial.println("Calibration values: ");
                Serial.print("X-Axis sensitivity adjustment value ");
                Serial.println(magCalibration[0], 2);
                Serial.print("Y-Axis sensitivity adjustment value ");
                Serial.println(magCalibration[1], 2);
                Serial.print("Z-Axis sensitivity adjustment value ");
                Serial.println(magCalibration[2], 2);
            }

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
}



void loop()
{  
    if (!passThru) {

        // Check event status register, way to chech data ready by polling rather than interrupt
        uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

        // Check for errors
        if(eventStatus & 0x02) { // error detected, what is it?

            uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
            if(!errorStatus) {
                if(errorStatus == 0x11) reporterr("Magnetometer failure!");
                if(errorStatus == 0x12) reporterr("Accelerometer failure!");
                if(errorStatus == 0x14) reporterr("Gyro failure!");
                if(errorStatus == 0x21) reporterr("Magnetometer initialization failure!");
                if(errorStatus == 0x22) reporterr("Accelerometer initialization failure!");
                if(errorStatus == 0x24) reporterr("Gyro initialization failure!");
                if(errorStatus == 0x30) reporterr("Math error!");
                if(errorStatus == 0x80) reporterr("Invalid sample rate!");
            }

            // Handle errors ToDo

        }

        // if no errors, see if new data is ready
        if(eventStatus & 0x10) { // new acceleration data available
            readSENtralAccelData(accelCount);

            // Now we'll calculate the accleration value into actual g's
            ax = (float)accelCount[0]*0.000488;  // get actual g value
            ay = (float)accelCount[1]*0.000488;    
            az = (float)accelCount[2]*0.000488;  
        }

        if(readByte(EM7180_ADDRESS, EM7180_EventStatus) & 0x20) { // new gyro data available

            readSENtralGyroData(gyroCount);

            // Now we'll calculate the gyro value into actual dps's
            gx = (float)gyroCount[0]*0.153;  // get actual dps value
            gy = (float)gyroCount[1]*0.153;    
            gz = (float)gyroCount[2]*0.153;  
        }

        if(readByte(EM7180_ADDRESS, EM7180_EventStatus) & 0x08) { // new mag data available

            readSENtralMagData(magCount);

            // Now we'll calculate the mag value into actual G's
            mx = (float)magCount[0]*0.305176;  // get actual G value
            my = (float)magCount[1]*0.305176;    
            mz = (float)magCount[2]*0.305176;  
        }

        //   if(readByte(EM7180_ADDRESS, EM7180_EventStatus) & 0x04) { // new quaternion data available
        readSENtralQuatData(Quat); 
        //  }

        // get BMP280 pressure
        if(readByte(EM7180_ADDRESS, EM7180_EventStatus) & 0x40) { // new baro data available
            //   Serial.println("new Baro data!");
            rawPressure = readSENtralBaroData();
            pressure = (float)rawPressure*0.01f +1013.25f; // pressure in mBar

            // get BMP280 temperature
            rawTemperature = readSENtralTempData();  
            temperature = (float) rawTemperature*0.01;  // temperature in degrees C
        }
    }

    if(passThru) {

        // If intPin goes high, all data registers have new data
        //  if (digitalRead(intACC2)) {  // On interrupt, read data
        readAccelData(accelCount);  // Read the x/y/z adc values

        // Now we'll calculate the acceleration value into actual g's
        ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes - accelBias[1];   
        az = (float)accelCount[2]*aRes - accelBias[2]; 
        // } 
        //  if (digitalRead(intGYRO2)) {  // On interrupt, read data
        readGyroData(gyroCount);  // Read the x/y/z adc values

        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes;  
        gz = (float)gyroCount[2]*gRes;   
        // }
        //  if (digitalRead(intDRDYM)) {  // On interrupt, read data
        readMagData(magCount);  // Read the x/y/z adc values

        // Calculate the magnetometer values in milliGauss
        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
        //    mx *= magScale[0];
        //    my *= magScale[1];
        //    mz *= magScale[2]; 
        //   }
    } 


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
    //  if(passThru)MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -my, mx, mz, deltat, q);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

        if(SerialDebug) {
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
            Serial.println("Hardware quaternions:"); 
            Serial.print("Q0 = ");
            Serial.print(Quat[0]);
            Serial.print(" Qx = ");
            Serial.print(Quat[1]); 
            Serial.print(" Qy = ");
            Serial.print(Quat[2]); 
            Serial.print(" Qz = ");
            Serial.println(Quat[3]); 
        }               


        if(passThru) {
            rawPress =  readBMP280Pressure();
            pressure = (float) bmp280_compensate_P(rawPress)/25600.; // Pressure in mbar
            rawTemp =   readBMP280Temperature();
            temperature = (float) bmp280_compensate_T(rawTemp)/100.;

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
        //Hardware AHRS:
        Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);   
        Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
        Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
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

        if(SerialDebug) {
            Serial.print("Software yaw, pitch, roll: ");
            Serial.print(yaw, 2);
            Serial.print(", ");
            Serial.print(pitch, 2);
            Serial.print(", ");
            Serial.println(roll, 2);

            Serial.print("Hardware Yaw, Pitch, Roll: ");
            Serial.print(Yaw, 2);
            Serial.print(", ");
            Serial.print(Pitch, 2);
            Serial.print(", ");
            Serial.println(Roll, 2);


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
        }

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



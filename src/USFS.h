/* 
   USFS source header

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

#pragma once

#include <stdint.h>

#include <Arduino.h>
#include <Wire.h>

enum {

    USFS_INTERRUPT_RESET_REQUIRED = 0x01,
    USFS_INTERRUPT_ERROR = 0x02,
    USFS_INTERRUPT_QUAT = 0x04,
    USFS_INTERRUPT_MAG = 0x08,
    USFS_INTERRUPT_ACCEL = 0x10,
    USFS_INTERRUPT_GYRO = 0x20,
    USFS_INTERRUPT_ANY = 0x40
};

typedef enum {

    AFS_2G,
    AFS_4G,
    AFS_8G,
    AFS_16G

} ascale_t;

typedef enum {

    GFS_250DPS,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS

} gscale_t;

typedef enum {

    MFS_14BITS, // 0.6 mG per LSB
    MFS_16BITS  // 0.15 mG per LSB

} mscale_t;

// Full-scale ranges are fixed in master mode
static const float USFS_GYRO_SCALE  = 1.53e-1;
static const float USFS_ACCEL_SCALE = 4.88e-4;

void usfsAlgorithmControlReset(void); 

bool usfsAlgorithmStatusIsStandby(uint8_t status);
bool usfsAlgorithmStatusIsAlgorithmSlow(uint8_t status);
bool usfsAlgorithmStatusIsStillnessMode(uint8_t status);
bool usfsAlgorithmStatusIsCalibrationCompleted(uint8_t status);
bool usfsAlgorithmStatusIsMagneticAnomaly(uint8_t status);
bool usfsAlgorithmStatusIsSensorUnreliable(uint8_t status);

void usfsBegin(
        uint8_t accelBandwidth,
        uint8_t gyroBandwidth,
        uint8_t quatDivisor,
        uint8_t magRate,
        uint8_t accelRateTenth,
        uint8_t gyroRateTenth,
        uint8_t baroRate, 
        uint8_t interruptEnable=USFS_INTERRUPT_GYRO,
        bool verbose=false);

uint8_t usfsCheckErrors();

void usfsCheckSensorStatus(uint8_t status);

uint8_t usfsCheckStatus();

void usfsEnableEvents(uint8_t mask);

static bool eventStatusIsAccelerometer(uint8_t status);
static bool eventStatusIsBarometer(uint8_t status);
static bool eventStatusIsError(uint8_t status);
static bool eventStatusIsGyrometer(uint8_t status);
static bool eventStatusIsMagnetometer(uint8_t status);
static bool eventStatusIsQuaternion(uint8_t status);
static bool eventStatusIsReset(uint8_t status);

uint8_t usfsGetAlgorithmStatus(void);
uint8_t usfsGetEventStatus(void);
uint8_t usfsGetParamAcknowledge(void);
uint8_t usfsGetPassThruStatus(void);
uint8_t usfsGetRunStatus(void);
uint8_t usfsGetSensorStatus(void);
uint8_t usfsGetSentralStatus(void);


bool usfsIsInPassThroughMode(void);

void usfsLoadFirmware(bool verbose = false);

int16_t readBarometerRaw();

void readAccelerometerRaw(int16_t counts[3]);

void readGyrometerRaw(int16_t counts[3]);

// Returns Gs
void readAccelerometerScaled(float & x, float & y, float & z);

// Returns degrees per second
void readGyrometerScaled(float & x, float & y, float & z);

void usfsreadMagnetometerScaled(float & x, float & y, float & z);

void readQuaternion(float & qw, float & qx, float & qy, float & qz);

int16_t readTemperatureRaw();

void  usfsReportChipId();

void readSavedParamBytes(uint8_t bytes[4]);

void usfsReportError(uint8_t errorStatus);

bool usfsRunStatusIsNormal(uint8_t status);

void usfsSetMasterMode(void);
void usfsSetPassThroughMode(void);

void usfsSetRatesAndBandwidths(
        uint8_t accelLpfBandwidth,
        uint8_t accelRateTenth,
        uint8_t baroRate,
        uint8_t gyroLpfBandwidth,
        uint8_t gyroRateTenth,
        uint8_t magRate,
        uint8_t quatDivisor);

void usfsSetRunEnable(void);
void usfsSetRunDisable(void);

void usfsWriteByte(uint8_t subAddress, uint8_t value);

class Usfs {

    private:

        static const uint8_t ADDRESS= 0x28;

        // this is a 32-bit normalized floating point number read from registers = 0x00-03
        static const uint8_t QX = 0x00;

        static const uint8_t QRateDivisor = 0x32;
        static const uint8_t EnableEvents = 0x33;
        static const uint8_t HostControl= 0x34;
        static const uint8_t EventStatus= 0x35;
        static const uint8_t SensorStatus = 0x36;
        static const uint8_t SentralStatus= 0x37;
        static const uint8_t AlgorithmStatus= 0x38;
        static const uint8_t FeatureFlags = 0x39;
        static const uint8_t ParamAcknowledge = 0x3A;
        static const uint8_t SavedParamByte0= 0x3B;
        static const uint8_t SavedParamByte1= 0x3C;
        static const uint8_t SavedParamByte2= 0x3D;
        static const uint8_t SavedParamByte3= 0x3E;
        static const uint8_t ActualMagRate= 0x45;
        static const uint8_t ActualAccelRate= 0x46;
        static const uint8_t ActualGyroRate = 0x47;
        static const uint8_t ActualBaroRate = 0x48;
        static const uint8_t ActualTempRate = 0x49;
        static const uint8_t ErrorRegister= 0x50;
        static const uint8_t AlgorithmControl = 0x54;
        static const uint8_t MagRate= 0x55;
        static const uint8_t AccelRate= 0x56;
        static const uint8_t GyroRate = 0x57;
        static const uint8_t BaroRate = 0x58;
        static const uint8_t TempRate = 0x59;
        static const uint8_t LoadParamByte0 = 0x60;
        static const uint8_t LoadParamByte1 = 0x61;
        static const uint8_t LoadParamByte2 = 0x62;
        static const uint8_t LoadParamByte3 = 0x63;
        static const uint8_t ParamRequest = 0x64;
        static const uint8_t ROMVersion1= 0x70;
        static const uint8_t ROMVersion2= 0x71;
        static const uint8_t RAMVersion1= 0x72;
        static const uint8_t RAMVersion2= 0x73;
        static const uint8_t ProductID= 0x90;
        static const uint8_t RevisionID = 0x91;
        static const uint8_t RunStatus= 0x92;
        static const uint8_t UploadAddress= 0x94 ;// uint16_t registers = 0x94 (MSB)-5(LSB);
        static const uint8_t UploadData = 0x96;
        static const uint8_t CRCHost= 0x97;// uint32_t from registers = 0x97-9A
        static const uint8_t ResetRequest = 0x9B ;
        static const uint8_t PassThruStatus = 0x9E ;
        static const uint8_t PassThruControl= 0xA0;

        static const uint8_t MX = 0x12;// int16_t from registers = 0x12-13
        static const uint8_t MY = 0x14;// int16_t from registers = 0x14-15
        static const uint8_t MZ = 0x16;// int16_t from registers = 0x16-17
        static const uint8_t MTIME = 0x18;// uint16_t from registers = 0x18-19
        static const uint8_t AX = 0x1A;// int16_t from registers = 0x1A-1B
        static const uint8_t AY = 0x1C;// int16_t from registers = 0x1C-1D
        static const uint8_t AZ = 0x1E;// int16_t from registers = 0x1E-1F
        static const uint8_t ATIME= 0x20;// uint16_t from registers = 0x20-21
        static const uint8_t GX = 0x22;// int16_t from registers = 0x22-23
        static const uint8_t GY = 0x24;// int16_t from registers = 0x24-25
        static const uint8_t GZ = 0x26;// int16_t from registers = 0x26-27
        static const uint8_t GTIME= 0x28;// uint16_t from registers = 0x28-29

        // start of two-byte MS5637 pressure data, 16-bit signed interger
        static const uint8_t Baro = 0x2A;

        // start of two-byte MS5637 temperature data, 16-bit signed interger
        static const uint8_t Temp = 0x2E;

        static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) 
        {
            Wire.beginTransmission(address);
            Wire.write(subAddress);
            Wire.write(data);
            Wire.endTransmission();
        }

        static uint8_t readByte(uint8_t address, uint8_t subAddress) 
        {
            Wire.beginTransmission(address);
            Wire.write(subAddress);
            Wire.endTransmission();
            Wire.requestFrom((int)address, (int)1);
            return Wire.read();
        }

        static void readBytes(
                uint8_t address,
                uint8_t subAddress,
                uint8_t count,
                uint8_t * dst)
        {  
            Wire.beginTransmission(address);   
            Wire.write(subAddress);
            Wire.endTransmission(false);      
            uint32_t i = 0;
            Wire.requestFrom((int)address, (int)count);
            while (Wire.available()) {
                dst[i++] = Wire.read(); 
            } 
        }


        static uint8_t readUsfsByte(uint8_t subAddress) 
        {
            return readByte(ADDRESS, subAddress);
        }

        static void readUsfsBytes( uint8_t subAddress, uint8_t count, uint8_t * dest) 
        {
            readBytes(ADDRESS, subAddress, count, dest);
        }

       static void setGyroFullScale(uint16_t gyro_fs)
        {
            uint8_t byte0 = gyro_fs & (0xFF);
            uint8_t byte1 = (gyro_fs >> 8) & (0xFF);
            uint8_t byte2 = 0x00;
            uint8_t byte3 = 0x00;

            setFullScale(byte0, byte1, byte2, byte3, 0xCB);
        }

        static void setMagAccFullScale (uint16_t mag_fs, uint16_t acc_fs)
        {
            uint8_t byte0 = mag_fs & (0xFF);
            uint8_t byte1 = (mag_fs >> 8) & (0xFF);
            uint8_t byte2 = acc_fs & (0xFF);
            uint8_t byte3 = (acc_fs >> 8) & (0xFF);

            setFullScale(byte0, byte1, byte2, byte3, 0xCA);
        }

        static void setFullScale(
                uint8_t byte0,
                uint8_t byte1,
                uint8_t byte2,
                uint8_t byte3,
                uint8_t request)
        {
            uint8_t bytes[4] = {byte0, byte1, byte2, byte3};
            writeByte(ADDRESS, LoadParamByte0, bytes[0]); 
            writeByte(ADDRESS, LoadParamByte1, bytes[1]); 
            writeByte(ADDRESS, LoadParamByte2, bytes[2]); 
            writeByte(ADDRESS, LoadParamByte3, bytes[3]); 
            writeByte(ADDRESS, ParamRequest, request); 
            writeByte(ADDRESS, AlgorithmControl, 0x80); 
            uint8_t status = readByte(ADDRESS, ParamAcknowledge); 
            while (status!=request) {
                status = readByte(ADDRESS, ParamAcknowledge);
            }
            writeByte(ADDRESS, ParamRequest, 0x00); 
            writeByte(ADDRESS, AlgorithmControl, 0x00); 
        }

        static void readThreeAxisRaw(uint8_t subAddress, int16_t counts[3])
        {
            uint8_t rawData[6] = {};

            // Read the six raw data registers into data array
            readUsfsBytes(subAddress, 6, &rawData[0]);       

            // Turn the MSB and LSB into a signed 16-bit value
            counts[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  
            counts[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
            counts[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
        }

        static void readThreeAxisScaled(uint8_t subAddress, float scale,
                float & x, float & y, float & z)
        {
            int16_t counts[3] = {};

            // Read the six raw data registers into data array
            readThreeAxisRaw(subAddress, counts);       

            // Turn the MSB and LSB into a signed 16-bit value
            x = scale * counts[0];
            y = scale * counts[1];
            z = scale * counts[2];
        }


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

        static int16_t read16BitValue(uint8_t subAddress)
        {
            uint8_t counts[2] = {};

            // Read the two raw data registers sequentially into data array
            readUsfsBytes(subAddress, 2, &counts[0]);  

            // Turn the MSB and LSB into a signed 16-bit value
            return  (int16_t) (((int16_t)counts[1] << 8) | counts[0]);   
        }

    public:

        static void algorithmControlRequestParameterTransfer(void)
        {
            usfsWriteByte(AlgorithmControl, 0x80);
        }

        static void requestParamRead(uint8_t param)
        {
            usfsWriteByte(ParamRequest, param); 
        }

        static void set_integer_param (uint8_t param, uint32_t param_val) 
        {
            uint8_t bytes[4] = {};
            bytes[0] = param_val & (0xFF);
            bytes[1] = (param_val >> 8) & (0xFF);
            bytes[2] = (param_val >> 16) & (0xFF);
            bytes[3] = (param_val >> 24) & (0xFF);

            // Parameter is the decimal value with the MSB set high to indicate a
            // paramter write processs
            param = param | 0x80; 

            loadParamBytes(bytes);

            requestParamRead(param);

            // Request parameter transfer procedure
            usfsWriteByte(AlgorithmControl, 0x80); 

            // Check the parameter acknowledge register and loop until the result
            // matches parameter request byte
            uint8_t status = usfsGetParamAcknowledge(); 

            while(!(status==param)) {
                status = usfsGetParamAcknowledge();
            }

            // Parameter request = 0 to end parameter transfer process
            usfsWriteByte(ParamRequest, 0x00); 

            usfsWriteByte(AlgorithmControl, 0x00); // Re-start algorithm
        }


        static void loadParamBytes(uint8_t byte[4])
        {
            usfsWriteByte(LoadParamByte0, byte[0]);
            usfsWriteByte(LoadParamByte1, byte[1]);
            usfsWriteByte(LoadParamByte2, byte[2]);
            usfsWriteByte(LoadParamByte3, byte[3]);
        }

        static void readSavedParamBytes(uint8_t bytes[4])
        {
            bytes[0] = readUsfsByte(SavedParamByte0);
            bytes[1] = readUsfsByte(SavedParamByte1);
            bytes[2] = readUsfsByte(SavedParamByte2);
            bytes[3] = readUsfsByte(SavedParamByte3);
        }

        void reportChipId(void)
        {
            // Read SENtral device information
            uint16_t ROM1 = readUsfsByte(ROMVersion1);
            uint16_t ROM2 = readUsfsByte(ROMVersion2);
            Serial.print("EM7180 ROM Version: 0x");
            Serial.print(ROM1, HEX);
            Serial.println(ROM2, HEX);
            Serial.println("Should be: 0xE609");
            uint16_t RAM1 = readUsfsByte(RAMVersion1);
            uint16_t RAM2 = readUsfsByte(RAMVersion2);
            Serial.print("EM7180 RAM Version: 0x");
            Serial.print(RAM1);
            Serial.println(RAM2);
            uint8_t PID = readUsfsByte(ProductID);
            Serial.print("EM7180 ProductID: 0x");
            Serial.print(PID, HEX);
            Serial.println(" Should be: 0x80");
            uint8_t RID = readUsfsByte(RevisionID);
            Serial.print("EM7180 RevisionID: 0x");
            Serial.print(RID, HEX);
            Serial.println(" Should be: 0x02");
        }    

        void loadFirmware(bool verbose)
        {
            // Check which sensors can be detected by the EM7180
            uint8_t featureflag = readUsfsByte(FeatureFlags);

            if (verbose) {
                if (featureflag & 0x01)  {
                    Serial.println("A barometer is installed");
                }
                if (featureflag & 0x02)  {
                    Serial.println("A humidity sensor is installed");
                }
                if (featureflag & 0x04)  {
                    Serial.println("A temperature sensor is installed");
                }
                if (featureflag & 0x08)  {
                    Serial.println("A custom sensor is installed");
                }
                if (featureflag & 0x10)  {
                    Serial.println("A second custom sensor is installed");
                }
                if (featureflag & 0x20)  {
                    Serial.println("A third custom sensor is installed");
                }
                delay(1000); // give some time to read the screen
            }


            bool okay = true;

            for (uint8_t k=0; k<10; ++k) {

                usfsWriteByte(ResetRequest, 0x01);

                delay(100);  

                uint8_t status = usfsGetSentralStatus();

                if (verbose) {
                    if (status & 0x01)  {
                        Serial.println("EEPROM detected on the sensor bus!");
                    }
                    if (status & 0x02)  {
                        Serial.println("EEPROM uploaded config file!");
                    }

                    if (status & 0x04)  {
                        Serial.println("EEPROM CRC incorrect!");
                        okay = false;
                    }

                    if (status & 0x08)  {
                        Serial.println("EM7180 in initialized state!");
                    }
                }

                if (status & 0x10)  {
                    Serial.println("No EEPROM detected!");
                    okay = false;
                }

                if (okay) {
                    break;
                }
            }

            if (okay) {
                if (!(usfsGetSentralStatus() & 0x04)) {
                    if (verbose) {
                        Serial.println("EEPROM upload successful!");
                    }
                }
            }
            else {
                while (true) {
                    Serial.println("Failed to load firmware");
                    delay(500);
                }
            }
        }

        void begin(
                uint8_t accelBandwidth,
                uint8_t gyroBandwidth,
                uint8_t quatDivisor,
                uint8_t magRate,
                uint8_t accelRateTenth,
                uint8_t gyroRateTenth,
                uint8_t baroRate,
                uint8_t interruptEnable,
                bool verbose)
        {
            uint16_t magFs,
            accelFs,
            gyroFs; // EM7180 sensor full scale ranges

            uint8_t param[4];      

            // Enter EM7180 initialized state
            usfsSetRunDisable();

            // Make sure pass through mode is off
            usfsWriteByte(PassThruControl, 0x00); 
            usfsSetRunEnable();

            // Set sensor rates and bandwidths
            usfsSetRatesAndBandwidths(
                    accelBandwidth,
                    accelRateTenth,
                    baroRate,
                    gyroBandwidth,
                    gyroRateTenth,
                    magRate,
                    quatDivisor);

            // Configure operating mode
            usfsWriteByte(AlgorithmControl, 0x00); // read scale sensor data

            // Enable interrupt to host upon certain events
            usfsEnableEvents(interruptEnable);

            // Enable EM7180 run mode
            usfsSetRunEnable();
            delay(100);

            if (verbose) {
                Serial.println("Beginning Parameter Adjustments");
            }

            // Read sensor default FS values from parameter space
            usfsWriteByte(ParamRequest, 0x4A); // Request to read parameter 74

            // Request parameter transfer process
            usfsWriteByte(AlgorithmControl, 0x80); 
            byte param_xfer = usfsGetParamAcknowledge();
            while(!(param_xfer==0x4A)) {
                param_xfer = usfsGetParamAcknowledge();
            }

            readSavedParamBytes(param);

            magFs = ((int16_t)(param[1]<<8) | param[0]);
            accelFs = ((int16_t)(param[3]<<8) | param[2]);

            if (verbose) {
                Serial.print("Magnetometer Default Full Scale Range: +/-");
                Serial.print(magFs);
                Serial.println("uT");
                Serial.print("Accelerometer Default Full Scale Range: +/-");
                Serial.print(accelFs);
                Serial.println("g");
            }

            // Request to read  parameter 75
            usfsWriteByte(ParamRequest, 0x4B); 

            param_xfer = usfsGetParamAcknowledge();
            while(!(param_xfer==0x4B)) {
                param_xfer = usfsGetParamAcknowledge();
            }
            readSavedParamBytes(param);

            gyroFs = ((int16_t)(param[1]<<8) | param[0]);

            if (verbose) {
                Serial.print("Gyroscope Default Full Scale Range: +/-");
                Serial.print(gyroFs);
                Serial.println("dps");
            }

            usfsWriteByte(ParamRequest, 0x00); //End parameter transfer
            usfsWriteByte(AlgorithmControl, 0x00); // re-enable algorithm

            // Disable stillness mode for balancing robot application
            set_integer_param (0x49, 0x00);

            // Full-scale ranges are fixed
            setMagAccFullScale (1000, 8); // 1000 uT, 8 g
            setGyroFullScale(2000);       // 2000 dps

            // Read sensor new FS values from parameter space
            usfsWriteByte(ParamRequest, 0x4A); // Request to read  parameter 74

            // Request parameter transfer process
            usfsWriteByte(AlgorithmControl, 0x80); 
            param_xfer = usfsGetParamAcknowledge();
            while(!(param_xfer==0x4A)) {
                param_xfer = usfsGetParamAcknowledge();
            }
            readSavedParamBytes(param);
            magFs = ((int16_t)(param[1]<<8) | param[0]);
            accelFs = ((int16_t)(param[3]<<8) | param[2]);

            if (verbose) {
                Serial.print("Magnetometer New Full Scale Range: +/-");
                Serial.print(magFs);
                Serial.println("uT");
                Serial.print("Accelerometer New Full Scale Range: +/-");
                Serial.print(accelFs);
                Serial.println("g");
            }

            usfsWriteByte(ParamRequest, 0x4B); // Request to read  parameter 75
            param_xfer = usfsGetParamAcknowledge();
            while(!(param_xfer==0x4B)) {
                param_xfer = usfsGetParamAcknowledge();
            }
            readSavedParamBytes(param);
            gyroFs = ((int16_t)(param[1]<<8) | param[0]);

            if (verbose) {
                Serial.print("Gyroscope New Full Scale Range: +/-");
                Serial.print(gyroFs);
                Serial.println("dps");
            }

            usfsWriteByte(ParamRequest, 0x00); //End parameter transfer
            usfsWriteByte(AlgorithmControl, 0x00); // re-enable algorithm

            // Read EM7180 status
            uint8_t runStatus = readUsfsByte(RunStatus);
            if (runStatus & 0x01) {
                if (verbose) {
                    Serial.println("EM7180 run status = normal mode");
                }
            }

            uint8_t algoStatus = readUsfsByte(AlgorithmStatus);

            if (verbose) {
                if (algoStatus & 0x01) Serial.println("EM7180 standby status");
                if (algoStatus & 0x02) Serial.println("EM7180 algorithm slow");
                if (algoStatus & 0x04) Serial.println("EM7180 in stillness mode");
                if (algoStatus & 0x08) Serial.println("EM7180 mag calibration completed");
                if (algoStatus & 0x10) Serial.println("EM7180 magnetic anomaly detected");
                if (algoStatus & 0x20) Serial.println("EM7180 unreliable sensor data");
            }

            uint8_t passthruStatus = readUsfsByte(PassThruStatus);

            if (passthruStatus & 0x01) Serial.print("EM7180 in passthru mode!");

            uint8_t eventStatus = readUsfsByte(EventStatus);

            if (eventStatus & 0x02) Serial.println("EM7180 Error");

            if (verbose) {
                if (eventStatus & 0x01) Serial.println("EM7180 CPU reset");
                if (eventStatus & 0x04) Serial.println("EM7180 new quaternion result");
                if (eventStatus & 0x08) Serial.println("EM7180 new mag result");
                if (eventStatus & 0x10) Serial.println("EM7180 new accel result");
                if (eventStatus & 0x20) Serial.println("EM7180 new gyro result"); 
                delay(1000); // give some time to read the screen
            }


            // Check sensor status
            uint8_t sensorStatus = readUsfsByte(SensorStatus);

            if (verbose) {

                Serial.print("EM7180 sensor status = ");
                Serial.println(sensorStatus);
            }

            usfsCheckSensorStatus(sensorStatus);

            if (verbose) {

                Serial.print("Actual MagRate = ");
                Serial.print(readUsfsByte(ActualMagRate));
                Serial.println(" Hz"); 
                Serial.print("Actual AccelRate = ");
                Serial.print(10*readUsfsByte(ActualAccelRate));
                Serial.println(" Hz"); 
                Serial.print("Actual GyroRate = ");
                Serial.print(10*readUsfsByte(ActualGyroRate));
                Serial.println(" Hz"); 
                Serial.print("Actual BaroRate = ");
                Serial.print(readUsfsByte(ActualBaroRate));
                Serial.println(" Hz"); 
            }
        }

        uint8_t checkStatus()
        {
            // Check event status register, way to check data ready by polling rather
            // than interrupt.  Reading clears the register and interrupt.
            return readUsfsByte(EventStatus); 
        }

        static bool eventStatusIsError(uint8_t status)
        {
            return status & 0x02;
        }

        static bool eventStatusIsReset(uint8_t status)
        {
            return status & 0x02;
        }

        static void reportError(uint8_t errorStatus)
        {

            switch (errorStatus) {

                case 0x11:
                    Serial.println("Magnetometer failure!");
                    break;

                case 0x12:
                    Serial.println("Accelerometer failure!");
                    break;

                case 0x14:
                    Serial.println("Gyro failure!");
                    break;

                case 0x21:
                    Serial.println("Magnetometer initialization failure!");
                    break;

                case 0x22:
                    Serial.println("Accelerometer initialization failure!");
                    break;

                case 0x24:
                    Serial.println("Gyro initialization failure!");
                    break;

                case 0x30:
                    Serial.println("Math error!");
                    break;

                case 0x80:
                    Serial.println("Invalid sample rate!");
                    break;
            }
        }
        void usfsReportError(uint8_t errorStatus)
        {

            switch (errorStatus) {

                case 0x11:
                    Serial.println("Magnetometer failure!");
                    break;

                case 0x12:
                    Serial.println("Accelerometer failure!");
                    break;

                case 0x14:
                    Serial.println("Gyro failure!");
                    break;

                case 0x21:
                    Serial.println("Magnetometer initialization failure!");
                    break;

                case 0x22:
                    Serial.println("Accelerometer initialization failure!");
                    break;

                case 0x24:
                    Serial.println("Gyro initialization failure!");
                    break;

                case 0x30:
                    Serial.println("Math error!");
                    break;

                case 0x80:
                    Serial.println("Invalid sample rate!");
                    break;
            }
        }

        static bool eventStatusIsAccelerometer(uint8_t status)
        {
            return status & 0x10;
        }

        static bool eventStatusIsGyrometer(uint8_t status)
        {
            return status & 0x20;
        }

        static bool eventStatusIsMagnetometer(uint8_t status)
        {
            return status & 0x08;
        }

        static bool eventStatusIsQuaternion(uint8_t status)
        {
            return status & 0x04;
        }

        static bool eventStatusIsBarometer(uint8_t status)
        {
            return status & 0x40;
        }

        // Returns scaled values (G)
        void readAccelerometerScaled(float & x, float & y, float & z)
        {
            readThreeAxisScaled(AX, USFS_ACCEL_SCALE, x, y, z);
        }

        // Returns scaled values (degrees per second)
        void readGyrometerScaled(float & x, float & y, float & z)
        {
            readThreeAxisScaled(GX, USFS_GYRO_SCALE, x, y, z);
        }

        // Returns scaled values (mGauss)
        void readMagnetometerScaled(float & x, float & y, float & z)
        {
            readThreeAxisScaled(MX, 0.305176f, x, y, z);
        }

        static void readAccelerometerRaw(int16_t counts[3])
        {
            readThreeAxisRaw(AX, counts);
        }

        int16_t readBarometerRaw()
        {
            return read16BitValue(Baro);
        }

        int16_t readTemperatureRaw()
        {
            return read16BitValue(Temp);
        }

        void readQuaternion(float & qw, float & qx, float & qy, float & qz)
        {
            uint8_t counts[16];  // x/y/z quaternion register data stored here

            // Read the sixteen raw data registers into data array
            readUsfsBytes(QX, 16, &counts[0]); 

            // SENtral stores quats as qx, qy, qz, qw!
            qx = uint32_reg_to_float (&counts[0]);
            qy = uint32_reg_to_float (&counts[4]);
            qz =  uint32_reg_to_float (&counts[8]);
            qw = uint32_reg_to_float (&counts[12]);   
        }

        static void setPassThroughMode()
        {

            usfsWriteByte(AlgorithmControl, 0x01);
            delay(5);

            usfsWriteByte(PassThruControl, 0x01);
            while (true) {
                if (readUsfsByte(PassThruStatus) & 0x01) break;
                delay(5);
            }
        }

}; // class Usfs


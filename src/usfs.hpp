/* 
   USFS source header

   Copyright (C) 2022 TleraCorp && Simon D. Levy

   Adapted from

https://github.com/kriswiner/SENtral_sensor_hub/tree/master/WarmStartandAccelCal

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

class Usfs {

    public:

        enum {

            INTERRUPT_RESET_REQUIRED = 0x01,
            INTERRUPT_ERROR = 0x02,
            INTERRUPT_QUAT = 0x04,
            INTERRUPT_MAG = 0x08,
            INTERRUPT_ACCEL = 0x10,
            INTERRUPT_GYRO = 0x20,
            INTERRUPT_ANY = 0x40
        };

        static constexpr float ACCEL_SCALE = 4.88e-4;

    private:

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
        static constexpr float GYRO_SCALE  = 1.53e-1;

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

        static const uint8_t ACC_LPF_BW = 0x5B;//Register GP36;
        static const uint8_t GYRO_LPF_BW= 0x5C;//Register GP37
        static const uint8_t BARO_LPF_BW= 0x5D;//Register GP38

        static uint8_t readByte(uint8_t subAddress) 
        {
            return readByte(ADDRESS, subAddress);
        }

        static void readBytes( uint8_t subAddress, uint8_t count, uint8_t * dest) 
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
            readBytes(subAddress, 6, &rawData[0]);       

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
            readBytes(subAddress, 2, &counts[0]);  

            // Turn the MSB and LSB into a signed 16-bit value
            return  (int16_t) (((int16_t)counts[1] << 8) | counts[0]);   
        }

    public:

        static bool algorithmStatusIsStillnessMode(uint8_t status)
        {
            return (bool)(status & 0x04);
        }

        static bool algorithmStatusIsCalibrationCompleted(uint8_t status)
        {
            return (bool)(status & 0x08);
        }

        static bool algorithmStatusIsMagneticAnomaly(uint8_t status)
        {
            return (bool)(status & 0x10);
        }

        static bool algorithmStatusIsSensorUnreliable(uint8_t status)
        {
            return (bool)(status & 0x20);
        }

        static bool isInPassThroughMode(void)
        {
            return (bool)readByte(PassThruStatus);
        }

        static bool algorithmStatusIsStandby(uint8_t status)
        {
            return (bool)(status & 0x01);
        }

        static bool algorithmStatusIsAlgorithmSlow(uint8_t status)
        {
            return (bool)(status & 0x02);
        }

        static uint8_t getSensorStatus(void)
        {
            return readByte(SensorStatus);
        }

        static void checkSensorStatus(uint8_t status) 
        {
            if (status & 0x01) dbgprintf("Magnetometer not acknowledging!\n");
            if (status & 0x02) dbgprintf("Accelerometer not acknowledging!\n");
            if (status & 0x04) dbgprintf("Gyro not acknowledging!\n");
            if (status & 0x10) dbgprintf("Magnetometer ID not recognized!\n");
            if (status & 0x20) dbgprintf("Accelerometer ID not recognized!\n");
            if (status & 0x40) dbgprintf("Gyro ID not recognized!\n");
        }

        static uint8_t getSentralStatus(void)
        {
            return readByte(SentralStatus); 
        }

        static uint8_t getPassThruStatus(void)
        {
            return readByte(PassThruStatus);
        }

        static uint8_t getRunStatus(void)
        {
            return readByte(RunStatus);
        }

        static void enableEvents(uint8_t mask)
        {
            writeByte(EnableEvents, mask);
        }

        static void setRatesAndBandwidths(
                uint8_t accelLpfBandwidth,
                uint8_t accelRateTenth,
                uint8_t baroRate,
                uint8_t gyroLpfBandwidth,
                uint8_t gyroRateTenth,
                uint8_t magRate,
                uint8_t quatDivisor)
        {
            setRunDisable();

            writeByte(ACC_LPF_BW, accelLpfBandwidth);   
            writeByte(AccelRate, accelRateTenth); 
            writeByte(BaroRate, 0x80 | baroRate);  
            writeByte(GYRO_LPF_BW, gyroLpfBandwidth); 
            writeByte(GyroRate, gyroRateTenth); 
            writeByte(MagRate, magRate); 
            writeByte(QRateDivisor, quatDivisor); 
        }

        static void setRunEnable(void)
        {
            writeByte(HostControl, 0x01); 
        }

        static void setRunDisable(void)
        {
            writeByte(HostControl, 0x00); 
        }

        static void setPassThroughMode()
        {

            writeByte(AlgorithmControl, 0x01);
            delayMsec(5);

            writeByte(PassThruControl, 0x01);
            while (true) {
                if (readByte(PassThruStatus) & 0x01) break;
                delayMsec(5);
            }
        }

        static void setMasterMode()
        {

            writeByte(PassThruControl, 0x00);
            while (true) {
                if (!(readByte(PassThruStatus) & 0x01)) break;
                delayMsec(5);
            }


            writeByte(AlgorithmControl, 0x00);
            while (true) {
                if (!(readByte(AlgorithmStatus) & 0x01)) break;
                delayMsec(5);
            }
        }

        static void writeByte(uint8_t subAddress, uint8_t data) 
        {
            writeByte(ADDRESS, subAddress, data);
        }


        static uint8_t getEventStatus(void)
        {
            return readByte(EventStatus);
        }

        static uint8_t getAlgorithmStatus(void)
        {
            return readByte(AlgorithmStatus);
        }

        static bool runStatusIsNormal(uint8_t status)
        {
            return (bool)(status & 0x01);
        }

        static void algorithmControlReset(void)
        {
            writeByte(AlgorithmControl, 0x00);
        }

        static uint8_t getParamAcknowledge(void)
        {
            return readByte(ParamAcknowledge);
        }

        static void algorithmControlRequestParameterTransfer(void)
        {
            writeByte(AlgorithmControl, 0x80);
        }

        static void requestParamRead(uint8_t param)
        {
            writeByte(ParamRequest, param); 
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
            writeByte(AlgorithmControl, 0x80); 

            // Check the parameter acknowledge register and loop until the result
            // matches parameter request byte
            uint8_t status = getParamAcknowledge(); 

            while(!(status==param)) {
                status = getParamAcknowledge();
            }

            // Parameter request = 0 to end parameter transfer process
            writeByte(ParamRequest, 0x00); 

            writeByte(AlgorithmControl, 0x00); // Re-start algorithm
        }


        static void loadParamBytes(uint8_t byte[4])
        {
            writeByte(LoadParamByte0, byte[0]);
            writeByte(LoadParamByte1, byte[1]);
            writeByte(LoadParamByte2, byte[2]);
            writeByte(LoadParamByte3, byte[3]);
        }

        static void readSavedParamBytes(uint8_t bytes[4])
        {
            bytes[0] = readByte(SavedParamByte0);
            bytes[1] = readByte(SavedParamByte1);
            bytes[2] = readByte(SavedParamByte2);
            bytes[3] = readByte(SavedParamByte3);
        }

        void reportChipId(void)
        {
            // Read SENtral device information
            uint16_t ROM1 = readByte(ROMVersion1);
            uint16_t ROM2 = readByte(ROMVersion2);
            dbgprintf("EM7180: ROM Version: 0x%02X%02X\n", ROM1, ROM2);
            dbgprintf("Should be: 0xE609\n");
            uint16_t RAM1 = readByte(RAMVersion1);
            uint16_t RAM2 = readByte(RAMVersion2);
            dbgprintf("EM7180: RAM Version: 0x%02X%02X\n", RAM1, RAM2);
            uint8_t PID = readByte(ProductID);
            dbgprintf("EM7180: ProductID: 0x%02X\n", PID);
            dbgprintf(" Should be: 0x80");
            uint8_t RID = readByte(RevisionID);
            dbgprintf("EM7180: RevisionID: 0x%02X\n", RID);
            dbgprintf(" Should be: 0x02\n");
        }    

        void loadFirmware(bool verbose=false)
        {
            // Check which sensors can be detected by the EM7180
            uint8_t featureflag = readByte(FeatureFlags);

            if (verbose) {
                if (featureflag & 0x01)  {
                    dbgprintf("EM7180: A barometer is installed\n");
                }
                if (featureflag & 0x02)  {
                    dbgprintf("EM7180: A humidity sensor is installed\n");
                }
                if (featureflag & 0x04)  {
                    dbgprintf("EM7180: A temperature sensor is installed\n");
                }
                if (featureflag & 0x08)  {
                    dbgprintf("EM7180: A custom sensor is installed\n");
                }
                if (featureflag & 0x10)  {
                    dbgprintf("EM7180: A second custom sensor is installed\n");
                }
                if (featureflag & 0x20)  {
                    dbgprintf("EM7180: A third custom sensor is installed\n");
                }
                delayMsec(1000); // give some time to read the screen
            }


            bool okay = true;

            for (uint8_t k=0; k<10; ++k) {

                writeByte(ResetRequest, 0x01);

                delayMsec(100);  

                uint8_t status = getSentralStatus();

                if (verbose) {
                    if (status & 0x01)  {
                        dbgprintf("EEPROM detected on the sensor bus!\n");
                    }
                    if (status & 0x02)  {
                        dbgprintf("EEPROM uploaded config file!\n");
                    }

                    if (status & 0x04)  {
                        dbgprintf("EEPROM CRC incorrect!\n");
                        okay = false;
                    }

                    if (status & 0x08)  {
                        dbgprintf("EM7180 in initialized state!\n");
                    }
                }

                if (status & 0x10)  {
                    dbgprintf("No EEPROM detected!\n");
                    okay = false;
                }

                if (okay) {
                    break;
                }
            }

            if (okay) {
                if (!(getSentralStatus() & 0x04)) {
                    if (verbose) {
                        dbgprintf("EEPROM upload successful!\n");
                    }
                }
            }
            else {
                while (true) {
                    dbgprintf("Failed to load firmware\n");
                    delayMsec(500);
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
                bool verbose=false)
        {
            uint16_t magFs,
            accelFs,
            gyroFs; // EM7180 sensor full scale ranges

            uint8_t param[4];      

            // Enter EM7180 initialized state
            setRunDisable();

            // Make sure pass through mode is off
            writeByte(PassThruControl, 0x00); 
            setRunEnable();

            // Set sensor rates and bandwidths
            setRatesAndBandwidths(
                    accelBandwidth,
                    accelRateTenth,
                    baroRate,
                    gyroBandwidth,
                    gyroRateTenth,
                    magRate,
                    quatDivisor);

            // Configure operating mode
            writeByte(AlgorithmControl, 0x00); // read scale sensor data

            // Enable interrupt to host upon certain events
            enableEvents(interruptEnable);

            // Enable EM7180 run mode
            setRunEnable();
            delayMsec(100);

            if (verbose) {
                dbgprintf("Beginning Parameter Adjustments\n");
            }

            // Read sensor default FS values from parameter space
            writeByte(ParamRequest, 0x4A); // Request to read parameter 74

            // Request parameter transfer process
            writeByte(AlgorithmControl, 0x80); 
            auto param_xfer = getParamAcknowledge();
            while(!(param_xfer==0x4A)) {
                param_xfer = getParamAcknowledge();
            }

            readSavedParamBytes(param);

            magFs = ((int16_t)(param[1]<<8) | param[0]);
            accelFs = ((int16_t)(param[3]<<8) | param[2]);

            if (verbose) {
                dbgprintf("Magnetometer Default Full Scale Range: +/-%d uT\n", magFs);
                dbgprintf("Accelerometer Default Full Scale Range: +/-%d g\n", accelFs);
            }

            // Request to read  parameter 75
            writeByte(ParamRequest, 0x4B); 

            param_xfer = getParamAcknowledge();
            while(!(param_xfer==0x4B)) {
                param_xfer = getParamAcknowledge();
            }
            readSavedParamBytes(param);

            gyroFs = ((int16_t)(param[1]<<8) | param[0]);

            if (verbose) {
                dbgprintf("Gyroscope Default Full Scale Range: +/-%d dps\n", gyroFs);
            }

            writeByte(ParamRequest, 0x00); //End parameter transfer
            writeByte(AlgorithmControl, 0x00); // re-enable algorithm

            // Disable stillness mode for balancing robot application
            set_integer_param (0x49, 0x00);

            // Full-scale ranges are fixed
            setMagAccFullScale (1000, 8); // 1000 uT, 8 g
            setGyroFullScale(2000);       // 2000 dps

            // Read sensor new FS values from parameter space
            writeByte(ParamRequest, 0x4A); // Request to read  parameter 74

            // Request parameter transfer process
            writeByte(AlgorithmControl, 0x80); 
            param_xfer = getParamAcknowledge();
            while(!(param_xfer==0x4A)) {
                param_xfer = getParamAcknowledge();
            }
            readSavedParamBytes(param);
            magFs = ((int16_t)(param[1]<<8) | param[0]);
            accelFs = ((int16_t)(param[3]<<8) | param[2]);

            if (verbose) {
                dbgprintf("Magnetometer New Full Scale Range: +/-%d uT\n", magFs);
                dbgprintf("Accelerometer New Full Scale Range: +/-%d g\n", accelFs);
            }

            writeByte(ParamRequest, 0x4B); // Request to read  parameter 75
            param_xfer = getParamAcknowledge();
            while(!(param_xfer==0x4B)) {
                param_xfer = getParamAcknowledge();
            }
            readSavedParamBytes(param);
            gyroFs = ((int16_t)(param[1]<<8) | param[0]);

            if (verbose) {
                dbgprintf("Gyroscope New Full Scale Range: +/-%d dps\n", gyroFs);
            }

            writeByte(ParamRequest, 0x00); //End parameter transfer
            writeByte(AlgorithmControl, 0x00); // re-enable algorithm

            // Read EM7180 status
            uint8_t runStatus = readByte(RunStatus);
            if (runStatus & 0x01) {
                if (verbose) {
                    dbgprintf("EM7180 run status = normal mode\n");
                }
            }

            uint8_t algoStatus = readByte(AlgorithmStatus);

            if (verbose) {
                if (algoStatus & 0x01) dbgprintf("EM7180 standby status\n");
                if (algoStatus & 0x02) dbgprintf("EM7180 algorithm slow\n");
                if (algoStatus & 0x04) dbgprintf("EM7180 in stillness mode\n");
                if (algoStatus & 0x08) dbgprintf("EM7180 mag calibration completed\n");
                if (algoStatus & 0x10) dbgprintf("EM7180 magnetic anomaly detected\n");
                if (algoStatus & 0x20) dbgprintf("EM7180 unreliable sensor data\n");
            }

            uint8_t passthruStatus = readByte(PassThruStatus);

            if (passthruStatus & 0x01) dbgprintf("EM7180 in passthru mode!\n");

            uint8_t eventStatus = readByte(EventStatus);

            if (eventStatus & 0x02) dbgprintf("EM7180 Error\n");

            if (verbose) {
                if (eventStatus & 0x01) dbgprintf("EM7180 CPU reset\n");
                if (eventStatus & 0x04) dbgprintf("EM7180 new quaternion result\n");
                if (eventStatus & 0x08) dbgprintf("EM7180 new mag result\n");
                if (eventStatus & 0x10) dbgprintf("EM7180 new accel result\n");
                if (eventStatus & 0x20) dbgprintf("EM7180 new gyro result\n"); 
                delayMsec(1000); // give some time to read the screen
            }

            // Check sensor status
            uint8_t sensorStatus = readByte(SensorStatus);

            if (verbose) {

                dbgprintf("EM7180 sensor status = %d\n", sensorStatus);
            }

            checkSensorStatus(sensorStatus);

            if (verbose) {

                dbgprintf("Actual MagRate = %d Hz\n", readByte(ActualMagRate));
                dbgprintf("Actual AccelRate = %d Hz\n", 10*readByte(ActualAccelRate));
                dbgprintf("Actual GyroRate = %d Hz\n", 10*readByte(ActualGyroRate));
                dbgprintf("Actual BaroRate = %d Hz\n", readByte(ActualBaroRate));
            }
        }

        static uint8_t checkStatus()
        {
            // Check event status register, way to check data ready by polling rather
            // than interrupt.  Reading clears the register and interrupt.
            return readByte(EventStatus); 
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
                    dbgprintf("Magnetometer failure!\n");
                    break;

                case 0x12:
                    dbgprintf("Accelerometer failure!\n");
                    break;

                case 0x14:
                    dbgprintf("Gyro failure!\n");
                    break;

                case 0x21:
                    dbgprintf("Magnetometer initialization failure!\n");
                    break;

                case 0x22:
                    dbgprintf("Accelerometer initialization failure!\n");
                    break;

                case 0x24:
                    dbgprintf("Gyro initialization failure!\n");
                    break;

                case 0x30:
                    dbgprintf("Math error!\n");
                    break;

                case 0x80:
                    dbgprintf("Invalid sample rate!\n");
                    break;
            }
        }
        void usfsReportError(uint8_t errorStatus)
        {

            switch (errorStatus) {

                case 0x11:
                    dbgprintf("Magnetometer failure!\n");
                    break;

                case 0x12:
                    dbgprintf("Accelerometer failure!\n");
                    break;

                case 0x14:
                    dbgprintf("Gyro failure!\n");
                    break;

                case 0x21:
                    dbgprintf("Magnetometer initialization failure!\n");
                    break;

                case 0x22:
                    dbgprintf("Accelerometer initialization failure!\n");
                    break;

                case 0x24:
                    dbgprintf("Gyro initialization failure!\n");
                    break;

                case 0x30:
                    dbgprintf("Math error!\n");
                    break;

                case 0x80:
                    dbgprintf("Invalid sample rate!\n");
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
            readThreeAxisScaled(AX, ACCEL_SCALE, x, y, z);
        }

        // Returns scaled values (degrees per second)
        void readGyrometerScaled(float & x, float & y, float & z)
        {
            readThreeAxisScaled(GX, GYRO_SCALE, x, y, z);
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

        static void readGyrometerRaw(int16_t counts[3])
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
            readBytes(QX, 16, &counts[0]); 

            // SENtral stores quats as qx, qy, qz, qw!
            qx = uint32_reg_to_float (&counts[0]);
            qy = uint32_reg_to_float (&counts[4]);
            qz =  uint32_reg_to_float (&counts[8]);
            qw = uint32_reg_to_float (&counts[12]);   
        }

        // Platform-dependent  ------------------------------------------------

        static void delayMsec(const uint32_t msec);

        static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

        static uint8_t readByte(uint8_t address, uint8_t subAddress) ;

        static void readBytes(
                uint8_t address,
                uint8_t subAddress,
                uint8_t count,
                uint8_t * dst);

        static void dbgprintf(const char * fmt, ...);

}; // class Usfs


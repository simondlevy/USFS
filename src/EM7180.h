/* 

   EM7180.h: Class header for EM7180 SENtral Sensor

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

#pragma once

#include <stdint.h>

class EM7180 {

    friend class EM7180_Master;

    protected:

        // EM7180 SENtral register map
        // see http://www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf
        static const uint8_t EM7180_QX                 = 0x00;  // this is a 32-bit normalized floating point number read from registers = 0x00-03
        static const uint8_t EM7180_QY                 = 0x04;  // this is a 32-bit normalized floating point number read from registers = 0x04-07
        static const uint8_t EM7180_QZ                 = 0x08;  // this is a 32-bit normalized floating point number read from registers = 0x08-0B
        static const uint8_t EM7180_QW                 = 0x0C;  // this is a 32-bit normalized floating point number read from registers = 0x0C-0F
        static const uint8_t EM7180_QTIME              = 0x10;  // this is a 16-bit unsigned integer read from registers = 0x10-11
        static const uint8_t EM7180_MX                 = 0x12;  // int16_t from registers = 0x12-13
        static const uint8_t EM7180_MY                 = 0x14;  // int16_t from registers = 0x14-15
        static const uint8_t EM7180_MZ                 = 0x16;  // int16_t from registers = 0x16-17
        static const uint8_t EM7180_MTIME              = 0x18;  // uint16_t from registers = 0x18-19
        static const uint8_t EM7180_AX                 = 0x1A;  // int16_t from registers = 0x1A-1B
        static const uint8_t EM7180_AY                 = 0x1C;  // int16_t from registers = 0x1C-1D
        static const uint8_t EM7180_AZ                 = 0x1E;  // int16_t from registers = 0x1E-1F
        static const uint8_t EM7180_ATIME              = 0x20;  // uint16_t from registers = 0x20-21
        static const uint8_t EM7180_GX                 = 0x22;  // int16_t from registers = 0x22-23
        static const uint8_t EM7180_GY                 = 0x24;  // int16_t from registers = 0x24-25
        static const uint8_t EM7180_GZ                 = 0x26;  // int16_t from registers = 0x26-27
        static const uint8_t EM7180_GTIME              = 0x28;  // uint16_t from registers = 0x28-29
        static const uint8_t EM7180_Baro               = 0x2A;  // start of two-byte MS5637 pressure data, 16-bit signed interger
        static const uint8_t EM7180_BaroTIME           = 0x2C;  // start of two-byte MS5637 pressure timestamp, 16-bit unsigned
        static const uint8_t EM7180_Temp               = 0x2E;  // start of two-byte MS5637 temperature data, 16-bit signed interger
        static const uint8_t EM7180_TempTIME           = 0x30;  // start of two-byte MS5637 temperature timestamp, 16-bit unsigned
        static const uint8_t EM7180_QRateDivisor       = 0x32;  // uint8_t 
        static const uint8_t EM7180_EnableEvents       = 0x33;
        static const uint8_t EM7180_HostControl        = 0x34;
        static const uint8_t EM7180_EventStatus        = 0x35;
        static const uint8_t EM7180_SensorStatus       = 0x36;
        static const uint8_t EM7180_SentralStatus      = 0x37;
        static const uint8_t EM7180_AlgorithmStatus    = 0x38;
        static const uint8_t EM7180_FeatureFlags       = 0x39;
        static const uint8_t EM7180_ParamAcknowledge   = 0x3A;
        static const uint8_t EM7180_SavedParamByte0    = 0x3B;
        static const uint8_t EM7180_SavedParamByte1    = 0x3C;
        static const uint8_t EM7180_SavedParamByte2    = 0x3D;
        static const uint8_t EM7180_SavedParamByte3    = 0x3E;
        static const uint8_t EM7180_ActualMagRate      = 0x45;
        static const uint8_t EM7180_ActualAccelRate    = 0x46;
        static const uint8_t EM7180_ActualGyroRate     = 0x47;
        static const uint8_t EM7180_ActualBaroRate     = 0x48;
        static const uint8_t EM7180_ActualTempRate     = 0x49;
        static const uint8_t EM7180_ErrorRegister      = 0x50;
        static const uint8_t EM7180_AlgorithmControl   = 0x54;
        static const uint8_t EM7180_MagRate            = 0x55;
        static const uint8_t EM7180_AccelRate          = 0x56;
        static const uint8_t EM7180_GyroRate           = 0x57;
        static const uint8_t EM7180_BaroRate           = 0x58;
        static const uint8_t EM7180_TempRate           = 0x59;
        static const uint8_t EM7180_LoadParamByte0     = 0x60;
        static const uint8_t EM7180_LoadParamByte1     = 0x61;
        static const uint8_t EM7180_LoadParamByte2     = 0x62;
        static const uint8_t EM7180_LoadParamByte3     = 0x63;
        static const uint8_t EM7180_ParamRequest       = 0x64;
        static const uint8_t EM7180_ROMVersion1        = 0x70;
        static const uint8_t EM7180_ROMVersion2        = 0x71;
        static const uint8_t EM7180_RAMVersion1        = 0x72;
        static const uint8_t EM7180_RAMVersion2        = 0x73;
        static const uint8_t EM7180_ProductID          = 0x90;
        static const uint8_t EM7180_RevisionID         = 0x91;
        static const uint8_t EM7180_RunStatus          = 0x92;
        static const uint8_t EM7180_UploadAddress      = 0x94; // uint16_t registers = 0x94 (MSB)-5(LSB)
        static const uint8_t EM7180_UploadData         = 0x96;  
        static const uint8_t EM7180_CRCHost            = 0x97; // uint32_t from registers = 0x97-9A
        static const uint8_t EM7180_ResetRequest       = 0x9B;   
        static const uint8_t EM7180_PassThruStatus     = 0x9E;   
        static const uint8_t EM7180_PassThruControl    = 0xA0;
        static const uint8_t EM7180_ACC_LPF_BW         = 0x5B;  //Register GP36
        static const uint8_t EM7180_GYRO_LPF_BW        = 0x5C;  //Register GP37
        static const uint8_t EM7180_BARO_LPF_BW        = 0x5D;  //Register GP38
        static const uint8_t EM7180_GP36               = 0x5B;
        static const uint8_t EM7180_GP37               = 0x5C;
        static const uint8_t EM7180_GP38               = 0x5D;
        static const uint8_t EM7180_GP39               = 0x5E;
        static const uint8_t EM7180_GP40               = 0x5F;
        static const uint8_t EM7180_GP50               = 0x69;
        static const uint8_t EM7180_GP51               = 0x6A;
        static const uint8_t EM7180_GP52               = 0x6B;
        static const uint8_t EM7180_GP53               = 0x6C;
        static const uint8_t EM7180_GP54               = 0x6D;
        static const uint8_t EM7180_GP55               = 0x6E;
        static const uint8_t EM7180_GP56               = 0x6F;

        static const uint8_t EM7180_ADDRESS           = 0x28;   // Address of the EM7180 SENtral sensor hub


        bool hasFeature(uint8_t features);

        static const uint8_t TEMP_OUT_H       = 0x41;
        static const uint8_t TEMP_OUT_L       = 0x42;

        // Cross-platform support
        uint8_t  _i2c;

        uint8_t errorStatus;


        void readThreeAxis(uint8_t xreg, int16_t & x, int16_t & y, int16_t & z);

        uint8_t readRegister(uint8_t subAddress);
        void    readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest);
        void    writeRegister(uint8_t subAddress, uint8_t data);

    public:

        bool begin(void);

        const char * getErrorString(void);

        uint8_t  getProductId(void); 
        uint8_t  getRevisionId(void); 
        uint16_t getRamVersion(void);
        uint16_t getRomVersion(void);

        uint8_t  getSentralStatus(void);

        void requestReset(void);
        void setPassThroughMode(void);
        void setMasterMode(void);
        void setRunEnable(void);
        void setRunDisable(void);

        void setAccelLpfBandwidth(uint8_t bw);
        void setGyroLpfBandwidth(uint8_t bw);

        void setQRateDivisor(uint8_t divisor);
        void setMagRate(uint8_t rate);
        void setAccelRate(uint8_t rate);
        void setGyroRate(uint8_t rate);
        void setBaroRate(uint8_t rate);

        uint8_t  getActualMagRate();
        uint16_t getActualAccelRate();
        uint16_t getActualGyroRate();
        uint8_t  getActualBaroRate();
        uint8_t  getActualTempRate();

        void algorithmControlRequestParameterTransfer(void);
        void algorithmControlReset(void); 

        void enableEvents(uint8_t mask);

        void    requestParamRead(uint8_t param);
        uint8_t getParamAcknowledge(void);
        uint8_t readSavedParamByte0(void);
        uint8_t readSavedParamByte1(void);
        uint8_t readSavedParamByte2(void);
        uint8_t readSavedParamByte3(void);

        uint8_t getRunStatus(void);
        uint8_t getAlgorithmStatus(void);
        uint8_t getPassThruStatus(void);
        uint8_t getEventStatus(void);
        uint8_t getSensorStatus(void);
        uint8_t getErrorStatus(void);

        void setGyroFs(uint16_t gyro_fs);
        void setMagAccFs(uint16_t mag_fs, uint16_t acc_fs);

        void loadParamByte0(uint8_t value);
        void loadParamByte1(uint8_t value);
        void loadParamByte2(uint8_t value);
        void loadParamByte3(uint8_t value);

        void writeGp36(uint8_t value);
        void writeGp37(uint8_t value);
        void writeGp38(uint8_t value);
        void writeGp39(uint8_t value);
        void writeGp40(uint8_t value);
        void writeGp50(uint8_t value);
        void writeGp51(uint8_t value);
        void writeGp52(uint8_t value);
        void writeGp53(uint8_t value);
        void writeGp54(uint8_t value);
        void writeGp55(uint8_t value);
        void writeGp56(uint8_t value);

        void readAccelerometer(int16_t & ax, int16_t & ay, int16_t & az);
        void readMagnetometer(int16_t & mx, int16_t & my, int16_t & mz);
        void readGyrometer(int16_t & gx, int16_t & gy, int16_t & gz);
        void readQuaternion(float & qw, float & qx, float & qy, float & qz);
        void readBarometer(float & pressure, float & temperature);

        void setIntegerParam (uint8_t param, uint32_t param_val);

        void getFullScaleRanges(uint8_t& accFs, uint16_t& gyroFs, uint16_t& magFs);

        static float uint32_reg_to_float (uint8_t *buf);

}; // class EM7180

class EM7180_Master {

    private:

        EM7180 _em7180;

        uint8_t _eventStatus;

        bool algorithmStatus(uint8_t status);

        void setGyroFs(uint16_t gyro_fs);
        void setMagAccFs(uint16_t mag_fs, uint16_t acc_fs);
        void setIntegerParam (uint8_t param, uint32_t param_val);

        uint8_t  _aRes;         // Gs
        uint16_t _gRes;         // radians per second
        uint16_t _mRes;         // microTeslas
        uint8_t  _magRate;      // Hz
        uint16_t _accelRate;    // Hz
        uint16_t _gyroRate;     // Hz
        uint8_t  _baroRate;     // Hz
        uint8_t  _qRateDivisor; // 1/3 gyro rate

    public:

        EM7180_Master(
                uint8_t  aRes,          // Gs
                uint16_t gRes,          // radians per second
                uint16_t mRes,          // microTeslas
                uint8_t  magRate,       // Hz
                uint16_t accelRate,     // Hz
                uint16_t gyroRate,      // Hz
                uint8_t  baroRate,      // Hz
                uint8_t  qRateDivisor); // 1/3 gyro rate

        const char * getErrorString(void);

        uint8_t  getProductId(void); 

        uint8_t  getRevisionId(void); 

        uint16_t getRamVersion(void);

        uint16_t getRomVersion(void);

        bool hasBaro(void);

        bool hasHumidity(void);

        bool hasTemperature(void);

        bool hasCustom1(void);

        bool hasCustom2(void);

        bool hasCustom3(void);

        bool begin(int8_t pin=-1);

        void checkEventStatus(void);

        bool gotError(void);

        bool gotInterrupt(void);

        bool gotQuaternion(void);

        bool gotMagnetometer(void);

        bool gotAccelerometer(void);

        bool gotGyrometer(void);

        bool gotBarometer(void);

        void readMagnetometer(int16_t & mx, int16_t & my, int16_t & mz);

        void readAccelerometer(int16_t & ax, int16_t & ay, int16_t & az);

        void readGyrometer(int16_t & gx, int16_t & gy, int16_t & gz);

        void readQuaternion(float & qw, float & qx, float & qy, float & qz);

        void readBarometer(float & pressure, float & temperature);

        uint8_t getActualMagRate();

        uint16_t getActualAccelRate();

        uint16_t getActualGyroRate();

        uint8_t getActualBaroRate();

        uint8_t getActualTempRate();

        bool runStatusNormal(void);

        bool algorithmStatusStandby(void);

        bool algorithmStatusSlow(void);

        bool algorithmStatusStillness(void);

        bool algorithmStatusMagCalibrationCompleted(void);

        bool algorithmStatusMagneticAnomalyDetected(void);

        bool algorithmStatusUnreliableData(void);

        void getFullScaleRanges(uint8_t& accFs, uint16_t& gyroFs, uint16_t& magFs);
};

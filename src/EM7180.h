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

// EM7180 SENtral register map
// see http://www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf
#define EM7180_QX                 0x00  // this is a 32-bit normalized floating point number read from registers 0x00-03
#define EM7180_QY                 0x04  // this is a 32-bit normalized floating point number read from registers 0x04-07
#define EM7180_QZ                 0x08  // this is a 32-bit normalized floating point number read from registers 0x08-0B
#define EM7180_QW                 0x0C  // this is a 32-bit normalized floating point number read from registers 0x0C-0F
#define EM7180_QTIME              0x10  // this is a 16-bit unsigned integer read from registers 0x10-11
#define EM7180_MX                 0x12  // int16_t from registers 0x12-13
#define EM7180_MY                 0x14  // int16_t from registers 0x14-15
#define EM7180_MZ                 0x16  // int16_t from registers 0x16-17
#define EM7180_MTIME              0x18  // uint16_t from registers 0x18-19
#define EM7180_AX                 0x1A  // int16_t from registers 0x1A-1B
#define EM7180_AY                 0x1C  // int16_t from registers 0x1C-1D
#define EM7180_AZ                 0x1E  // int16_t from registers 0x1E-1F
#define EM7180_ATIME              0x20  // uint16_t from registers 0x20-21
#define EM7180_GX                 0x22  // int16_t from registers 0x22-23
#define EM7180_GY                 0x24  // int16_t from registers 0x24-25
#define EM7180_GZ                 0x26  // int16_t from registers 0x26-27
#define EM7180_GTIME              0x28  // uint16_t from registers 0x28-29
#define EM7180_Baro               0x2A  // start of two-byte MS5637 pressure data, 16-bit signed interger
#define EM7180_BaroTIME           0x2C  // start of two-byte MS5637 pressure timestamp, 16-bit unsigned
#define EM7180_Temp               0x2E  // start of two-byte MS5637 temperature data, 16-bit signed interger
#define EM7180_TempTIME           0x30  // start of two-byte MS5637 temperature timestamp, 16-bit unsigned
#define EM7180_QRateDivisor       0x32  // uint8_t 
#define EM7180_EnableEvents       0x33
#define EM7180_HostControl        0x34
#define EM7180_EventStatus        0x35
#define EM7180_SensorStatus       0x36
#define EM7180_SentralStatus      0x37
#define EM7180_AlgorithmStatus    0x38
#define EM7180_FeatureFlags       0x39
#define EM7180_ParamAcknowledge   0x3A
#define EM7180_SavedParamByte0    0x3B
#define EM7180_SavedParamByte1    0x3C
#define EM7180_SavedParamByte2    0x3D
#define EM7180_SavedParamByte3    0x3E
#define EM7180_ActualMagRate      0x45
#define EM7180_ActualAccelRate    0x46
#define EM7180_ActualGyroRate     0x47
#define EM7180_ActualBaroRate     0x48
#define EM7180_ActualTempRate     0x49
#define EM7180_ErrorRegister      0x50
#define EM7180_AlgorithmControl   0x54
#define EM7180_MagRate            0x55
#define EM7180_AccelRate          0x56
#define EM7180_GyroRate           0x57
#define EM7180_BaroRate           0x58
#define EM7180_TempRate           0x59
#define EM7180_LoadParamByte0     0x60
#define EM7180_LoadParamByte1     0x61
#define EM7180_LoadParamByte2     0x62
#define EM7180_LoadParamByte3     0x63
#define EM7180_ParamRequest       0x64
#define EM7180_ROMVersion1        0x70
#define EM7180_ROMVersion2        0x71
#define EM7180_RAMVersion1        0x72
#define EM7180_RAMVersion2        0x73
#define EM7180_ProductID          0x90
#define EM7180_RevisionID         0x91
#define EM7180_RunStatus          0x92
#define EM7180_UploadAddress      0x94 // uint16_t registers 0x94 (MSB)-5(LSB)
#define EM7180_UploadData         0x96  
#define EM7180_CRCHost            0x97  // uint32_t from registers 0x97-9A
#define EM7180_ResetRequest       0x9B   
#define EM7180_PassThruStatus     0x9E   
#define EM7180_PassThruControl    0xA0
#define EM7180_ACC_LPF_BW         0x5B  //Register GP36
#define EM7180_GYRO_LPF_BW        0x5C  //Register GP37
#define EM7180_BARO_LPF_BW        0x5D  //Register GP38
#define EM7180_GP36               0x5B
#define EM7180_GP37               0x5C
#define EM7180_GP38               0x5D
#define EM7180_GP39               0x5E
#define EM7180_GP40               0x5F
#define EM7180_GP50               0x69
#define EM7180_GP51               0x6A
#define EM7180_GP52               0x6B
#define EM7180_GP53               0x6C
#define EM7180_GP54               0x6D
#define EM7180_GP55               0x6E
#define EM7180_GP56               0x6F

#define EM7180_ADDRESS           0x28   // Address of the EM7180 SENtral sensor hub
#define M24512DFM_DATA_ADDRESS   0x50   // Address of the 500 page M24512DRC EEPROM data buffer, 1024 bits (128 8-bit bytes) per page

//
class EM7180 {

    private:

        bool hasFeature(uint8_t features);

    protected:

        static const uint8_t TEMP_OUT_H       = 0x41;
        static const uint8_t TEMP_OUT_L       = 0x42;

       // Cross-platform support
        uint8_t  _i2c;

        bool begin(void);

        uint8_t errorStatus;

        
        void readThreeAxis(uint8_t xreg, int16_t & x, int16_t & y, int16_t & z);

        uint8_t readRegister(uint8_t subAddress);
        void    readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest);
        void    writeRegister(uint8_t subAddress, uint8_t data);

    public:

        const char * getErrorString(void);

        uint8_t getProductId(void); 
        uint8_t getRevisionId(void); 
        uint16_t getRamVersion(void);
        uint16_t getRomVersion(void);

        bool hasBaro(void);
        bool hasHumidity(void);
        bool hasTemperature(void);
        bool hasCustom1(void);
        bool hasCustom2(void);
        bool hasCustom3(void);
};

class EM7180Master : public EM7180 {

    private:

        uint8_t _eventStatus;

        bool algorithmStatus(uint8_t status);

        void setGyroFs(uint16_t gyro_fs);
        void setMagAccFs(uint16_t mag_fs, uint16_t acc_fs);
        void setIntegerParam (uint8_t param, uint32_t param_val);

        static float uint32_reg_to_float (uint8_t *buf);

        uint8_t  _aRes;         // Gs
        uint16_t _gRes;         // radians per second
        uint16_t _mRes;         // microTeslas
        uint8_t  _magRate;      // Hz
        uint16_t _accelRate;    // Hz
        uint16_t _gyroRate;     // Hz
        uint8_t  _baroRate;     // Hz
        uint8_t  _qRateDivisor; // 1/3 gyro rate

    public:

        EM7180Master(
                uint8_t  aRes,          // Gs
                uint16_t gRes,          // radians per second
                uint16_t mRes,          // microTeslas
                uint8_t  magRate,       // Hz
                uint16_t accelRate,     // Hz
                uint16_t gyroRate,      // Hz
                uint8_t  baroRate,      // Hz
                uint8_t  qRateDivisor); // 1/3 gyro rate

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

class EM7180Passthru : public EM7180 {

    public:

        bool begin(void);
};

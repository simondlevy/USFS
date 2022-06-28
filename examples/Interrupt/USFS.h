#pragma once

#include "Arduino.h"
#include "Wire.h"   

class USFS
{
    public: 

        USFS(uint8_t intPin, bool passThru);

        uint8_t checkEM7180Errors();

        uint8_t checkEM7180Status();

        void  getChipID();

        void initEM7180(uint8_t accBW,
                uint8_t gyroBW,
                uint16_t accFS,
                uint16_t gyroFS,
                uint16_t magFS,
                uint8_t QRtDiv,
                uint8_t magRt,
                uint8_t accRt,
                uint8_t gyroRt,
                uint8_t baroRt);

        void loadfwfromEEPROM();

        void    readSENtralAccelData(int16_t * destination);
        int16_t readSENtralBaroData();
        void    readSENtralGyroData(int16_t * destination);
        void    readSENtralMagData(int16_t * destination);
        void    readSENtralQuatData(float * destination);
        int16_t readSENtralTempData();

    private:

        bool _passThru;
        float _aRes;
        float _gRes;
        float _mRes;
        uint8_t _Mmode;
        float _fuseROMx;
        float _fuseROMy;
        float _fuseROMz;
        float _q[4];
        float _beta;
        float _deltat;
        float _Kp;
        float _Ki;

        float uint32_reg_to_float (uint8_t *buf);
        float int32_reg_to_float (uint8_t *buf);
        void float_to_bytes (float param_val, uint8_t *buf);
        void EM7180_set_gyro_FS (uint16_t gyro_fs);
        void EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs);
        void EM7180_set_integer_param (uint8_t param, uint32_t param_val);
        void EM7180_set_float_param (uint8_t param, float param_val);
        void readAccelData(int16_t * destination);
        void readGyroData(int16_t * destination);
        void readMagData(int16_t * destination);
        void initAK8963(uint8_t Mscale, uint8_t Mmode, float * destination);
        void initMPU9250(uint8_t Ascale, uint8_t Gscale);
        void accelgyrocalMPU9250(float * dest1, float * dest2);
        void MPU9250SelfTest(float * destination);
        void SENtralPassThroughMode();
        void M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data);
        void M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest);
        uint8_t M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2);
        void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest);
        void MS5637Reset();
        void MS5637PromRead(uint16_t * destination);
        unsigned char MS5637checkCRC(uint16_t * n_prom);
        void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
        void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
        void I2Cscan();
        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
        uint8_t readByte(uint8_t address, uint8_t subAddress);
        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
        uint8_t _intPin;
        int16_t readTempData();

};

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

class _EM7180 {

    private:

        static bool hasFeature(uint8_t features);

    protected:

        uint8_t begin(void);

        static void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, 
                uint8_t count, uint8_t * dest);

        static void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
        static uint8_t readByte(uint8_t address, uint8_t subAddress);
        static void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data);


    public:

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

class EM7180 : public _EM7180 {

    private:

        int16_t accelCount[3];
        int16_t gyroCount[3];
        int16_t magCount[3];
    
        float quaternions[4];

        float temperature;
        float pressure;

        static bool algorithmStatus(uint8_t status);

        static void setGyroFs(uint16_t gyro_fs);
        static void setMagAccFs(uint16_t mag_fs, uint16_t acc_fs);
        static void setIntegerParam (uint8_t param, uint32_t param_val);

        static void    readSENtralMagData(int16_t * destination);
        static void    readSENtralGyroData(int16_t * destination);
        static void    readSENtralAccelData(int16_t * destination);
        static void    readSENtralQuatData(float * destination);
        static int16_t readSENtralTempData(void);
        static int16_t readSENtralBaroData();

        static float uint32_reg_to_float (uint8_t *buf);

    public:

        uint8_t begin(uint8_t ares, uint16_t gres, uint16_t mres);

        uint8_t update(void);

        void getAccelRaw(int16_t& ax, int16_t& ay, int16_t& az);
        void getGyroRaw(int16_t& gx, int16_t& gy, int16_t& gz);
        void getMagRaw(int16_t& mx, int16_t& my, int16_t& mz);

        void getQuaternions(float q[4]);

        void getBaro(float & press, float & temp);

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

        static const char * errorToString(uint8_t errorStatus);

        void getFullScaleRanges(uint8_t& accFs, uint16_t& gyroFs, uint16_t& magFs);
};

class EM7180_Passthru : public _EM7180 {

    public:

        uint8_t begin(void);
};

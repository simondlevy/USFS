#pragma once

#include <stdint.h>

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

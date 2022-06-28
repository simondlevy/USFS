#pragma once

#include <stdint.h>

uint8_t usfsCheckErrors();

uint8_t usfsCheckStatus();

void  getChipID();

void usfsBegin(
        uint8_t accBW,
        uint8_t gyroBW,
        uint16_t accFS,
        uint16_t gyroFS,
        uint16_t magFS,
        uint8_t QRtDiv,
        uint8_t magRt,
        uint8_t accRt,
        uint8_t gyroRt,
        uint8_t baroRt);

void usfsLoadFirmware();

void    usfsReadAccelerometer(int16_t * destination);
int16_t usfsReadBarometer();
void    usfsReadGyrometer(int16_t * destination);
void    usfsreadMagnetometer(int16_t * destination);
void    usfsReadQuaternion(float * destination);
int16_t usfsReadTemperature();

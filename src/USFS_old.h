#pragma once

#include <stdint.h>

void usfs2_algorithmControlReset(void); 
void usfs2_algorithmControlRequestParameterTransfer(void);

bool usfs2_begin(void);

void usfs2_enableEvents(uint8_t usfs2_mask);

uint8_t usfs2_getParamAcknowledge(void);

void usfs2_requestParamRead(uint8_t usfs2_param);

uint8_t usfs2_getSentralStatus(void);

void usfs2_requestReset(void);

void usfs2_setAccelLpfBandwidth(uint8_t usfs2_bw);

void usfs2_readAccelerometer(int16_t & ax, int16_t & ay, int16_t & az);
void usfs2_readQuaternion(float & qw, float & qx, float & qy, float & qz);

void usfs2_setAccelRate(uint8_t usfs2_rate);
void usfs2_setBaroRate(uint8_t usfs2_rate);
void usfs2_setGyroFs(uint16_t gyro_fs);
void usfs2_setGyroLpfBandwidth(uint8_t usfs2_bw);
void usfs2_setGyroRate(uint8_t usfs2_rate);
void usfs2_setMagAccFs(uint16_t mag_fs, uint16_t acc_fs);
void usfs2_setMagRate(uint8_t usfs2_rate);
void usfs2_setQRateDivisor(uint8_t usfs2_divisor);

void usfs2_setMasterMode(void);
void usfs2_setPassThroughMode(void);

void usfs2_setRunDisable(void);
void usfs2_setRunEnable(void);

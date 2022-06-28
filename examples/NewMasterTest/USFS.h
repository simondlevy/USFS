#pragma once

enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

enum Mscale {
    MFS_14BITS = 0, // 0.6 mG per LSB
    MFS_16BITS      // 0.15 mG per LSB
};

enum Mmode {
    MMODE_8HZ = 2,
    MMODE_100HZ = 6
};


void usfsSetGyroFs (uint16_t gyro_fs);

void usfsSetMagAccFs (uint16_t mag_fs, uint16_t acc_fs);

void usfsSetIntegerParam (uint8_t param, uint32_t param_val);

void usfsReadQuaternion(float * destination);

void usfsReadAccelerometer(int16_t * destination);

void usfsReadGyrometer(int16_t * destination);

void usfsReadMagnetometer(int16_t * destination);

int16_t usfsReadBarometer();

int16_t usfsReadTemperature();

uint8_t usfsReadRom1(void);

uint8_t usfsReadRom2(void);

uint8_t usfsReadRam1(void);

uint8_t usfsReadRam2(void);

uint8_t usfsReadPid(void);

uint8_t usfsReadRid(void);

bool usfsHasBarometer(void);

bool usfsHasHumiditySensor(void);

bool usfsHasTemperatureSensor(void);

bool usfsHasCustomSensor(void);

bool usfsHasSecondCustomSensor(void);

bool usfsHasThirdCustomSensor(void);

bool usfsEepromDetected(void);

bool usfsEepromUploaded(void);

bool usfsEepromIncorrect(void);

bool usfsEepromInitialized(void);

bool usfsEepromNotDetected(void);

void usfsReset(void);

bool usfsEepromUploadSuccessful(void);

void usfsBegin(
        uint8_t quatRateDivisor,
        uint8_t magRate,
        uint8_t accelRate,
        uint8_t gyroRate,
        uint8_t baroRate,
        bool verbose=false);

uint8_t usfsGetEventStatus(void);

bool usfsEventStatusIsError(uint8_t eventStatus);

void usfsReportEventError();

bool usfsEventIsAccelerometer(uint8_t eventStatus);

bool usfsEventIsGyrometer(uint8_t eventStatus);

bool usfsEventIsMagnetometer(uint8_t eventStatus);

bool usfsEventIsQuaternion(uint8_t eventStatus);

bool usfsEventIsBarometer(uint8_t eventStatus);

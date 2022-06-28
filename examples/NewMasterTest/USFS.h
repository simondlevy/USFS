#pragma once

// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
static const uint8_t MS5637_RESET      = 0x1E;
static const uint8_t MS5637_CONVERT_D1 = 0x40;
static const uint8_t MS5637_CONVERT_D2 = 0x50;
static const uint8_t MS5637_ADC_READ   = 0x00;
;
//Magnetometer Registers
static const uint8_t AK8963_ADDRESS   = 0x0C;
static const uint8_t WHO_AM_I_AK8963  = 0x00; // should return = 0x48;
static const uint8_t INFO             = 0x01;
static const uint8_t AK8963_ST1       = 0x02;  // data ready status bit 0;
static const uint8_t AK8963_XOUT_L   = 0x03;  // data;
static const uint8_t AK8963_XOUT_H  = 0x04;
static const uint8_t AK8963_YOUT_L  = 0x05;
static const uint8_t AK8963_YOUT_H  = 0x06;
static const uint8_t AK8963_ZOUT_L  = 0x07;
static const uint8_t AK8963_ZOUT_H  = 0x08;
static const uint8_t AK8963_ST2       = 0x09;  // Data overflow bit 3 and data read error status bit 2;
static const uint8_t AK8963_CNTL      = 0x0A;  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0;
static const uint8_t AK8963_ASTC      = 0x0C;  // Self test control;
static const uint8_t AK8963_I2CDIS    = 0x0F;  // I2C disable;
static const uint8_t AK8963_ASAX      = 0x10;  // Fuse ROM x-axis sensitivity adjustment value;
static const uint8_t AK8963_ASAY      = 0x11;  // Fuse ROM y-axis sensitivity adjustment value;
static const uint8_t AK8963_ASAZ      = 0x12;  // Fuse ROM z-axis sensitivity adjustment value;

static const uint8_t SELF_TEST_X_GYRO = 0x00;
static const uint8_t SELF_TEST_Y_GYRO = 0x01;
static const uint8_t SELF_TEST_Z_GYRO = 0x02;

static const uint8_t SELF_TEST_X_ACCEL = 0x0D;
static const uint8_t SELF_TEST_Y_ACCEL = 0x0E;
static const uint8_t SELF_TEST_Z_ACCEL = 0x0F;

static const uint8_t SELF_TEST_A= 0x10;

static const uint8_t XG_OFFSET_H= 0x13;// User-defined trim values for gyroscope
static const uint8_t XG_OFFSET_L= 0x14;
static const uint8_t YG_OFFSET_H= 0x15;
static const uint8_t YG_OFFSET_L= 0x16;
static const uint8_t ZG_OFFSET_H= 0x17;
static const uint8_t ZG_OFFSET_L= 0x18;
static const uint8_t SMPLRT_DIV = 0x19;
static const uint8_t CONFIG = 0x1A;
static const uint8_t GYRO_CONFIG= 0x1B;
static const uint8_t ACCEL_CONFIG = 0x1C;
static const uint8_t ACCEL_CONFIG2= 0x1D;
static const uint8_t LP_ACCEL_ODR = 0x1E;
static const uint8_t WOM_THR= 0x1F;

static const uint8_t MOT_DUR= 0x20;// Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
static const uint8_t ZMOT_THR = 0x21;// Zero-motion detection threshold bits [7:0]
static const uint8_t ZRMOT_DUR= 0x22;// Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

static const uint8_t FIFO_EN= 0x23;
static const uint8_t I2C_MST_CTRL = 0x24 ;
static const uint8_t I2C_SLV0_ADDR= 0x25;
static const uint8_t I2C_SLV0_REG = 0x26;
static const uint8_t I2C_SLV0_CTRL= 0x27;
static const uint8_t I2C_SLV1_ADDR= 0x28;
static const uint8_t I2C_SLV1_REG = 0x29;
static const uint8_t I2C_SLV1_CTRL= 0x2A;
static const uint8_t I2C_SLV2_ADDR= 0x2B;
static const uint8_t I2C_SLV2_REG = 0x2C;
static const uint8_t I2C_SLV2_CTRL= 0x2D;
static const uint8_t I2C_SLV3_ADDR= 0x2E;
static const uint8_t I2C_SLV3_REG = 0x2F;
static const uint8_t I2C_SLV3_CTRL= 0x30;
static const uint8_t I2C_SLV4_ADDR= 0x31;
static const uint8_t I2C_SLV4_REG = 0x32;
static const uint8_t I2C_SLV4_DO= 0x33;
static const uint8_t I2C_SLV4_CTRL= 0x34;
static const uint8_t I2C_SLV4_DI= 0x35;
static const uint8_t I2C_MST_STATUS = 0x36;
static const uint8_t INT_PIN_CFG= 0x37;
static const uint8_t INT_ENABLE = 0x38;
static const uint8_t DMP_INT_STATUS = 0x39;// Check DMP interrupt;
static const uint8_t INT_STATUS = 0x3A;
static const uint8_t ACCEL_XOUT_H = 0x3B;
static const uint8_t ACCEL_XOUT_L = 0x3C;
static const uint8_t ACCEL_YOUT_H = 0x3D;
static const uint8_t ACCEL_YOUT_L = 0x3E;
static const uint8_t ACCEL_ZOUT_H = 0x3F;
static const uint8_t ACCEL_ZOUT_L = 0x40;
static const uint8_t TEMP_OUT_H = 0x41;
static const uint8_t TEMP_OUT_L = 0x42;
static const uint8_t GYRO_XOUT_H= 0x43;
static const uint8_t GYRO_XOUT_L= 0x44;
static const uint8_t GYRO_YOUT_H= 0x45;
static const uint8_t GYRO_YOUT_L= 0x46;
static const uint8_t GYRO_ZOUT_H= 0x47;
static const uint8_t GYRO_ZOUT_L= 0x48;
static const uint8_t EXT_SENS_DATA_00 = 0x49;
static const uint8_t EXT_SENS_DATA_01 = 0x4A;
static const uint8_t EXT_SENS_DATA_02 = 0x4B;
static const uint8_t EXT_SENS_DATA_03 = 0x4C;
static const uint8_t EXT_SENS_DATA_04 = 0x4D;
static const uint8_t EXT_SENS_DATA_05 = 0x4E;
static const uint8_t EXT_SENS_DATA_06 = 0x4F;
static const uint8_t EXT_SENS_DATA_07 = 0x50;
static const uint8_t EXT_SENS_DATA_08 = 0x51;
static const uint8_t EXT_SENS_DATA_09 = 0x52;
static const uint8_t EXT_SENS_DATA_10 = 0x53;
static const uint8_t EXT_SENS_DATA_11 = 0x54;
static const uint8_t EXT_SENS_DATA_12 = 0x55;
static const uint8_t EXT_SENS_DATA_13 = 0x56;
static const uint8_t EXT_SENS_DATA_14 = 0x57;
static const uint8_t EXT_SENS_DATA_15 = 0x58;
static const uint8_t EXT_SENS_DATA_16 = 0x59;
static const uint8_t EXT_SENS_DATA_17 = 0x5A;
static const uint8_t EXT_SENS_DATA_18 = 0x5B;
static const uint8_t EXT_SENS_DATA_19 = 0x5C;
static const uint8_t EXT_SENS_DATA_20 = 0x5D;
static const uint8_t EXT_SENS_DATA_21 = 0x5E;
static const uint8_t EXT_SENS_DATA_22 = 0x5F;
static const uint8_t EXT_SENS_DATA_23 = 0x60;
static const uint8_t MOT_DETECT_STATUS = 0x61;
static const uint8_t I2C_SLV0_DO= 0x63;
static const uint8_t I2C_SLV1_DO= 0x64;
static const uint8_t I2C_SLV2_DO= 0x65;
static const uint8_t I2C_SLV3_DO= 0x66;
static const uint8_t I2C_MST_DELAY_CTRL = 0x67;
static const uint8_t SIGNAL_PATH_RESET= 0x68;
static const uint8_t MOT_DETECT_CTRL= 0x69;
static const uint8_t USER_CTRL= 0x6A;// Bit 7 enable DMP, bit 3 reset DMP;
static const uint8_t PWR_MGMT_1 = 0x6B ;// Device defaults to the SLEEP mode
static const uint8_t PWR_MGMT_2 = 0x6C;
static const uint8_t DMP_BANK = 0x6D;// Activates a specific bank in the DMP
static const uint8_t DMP_RW_PNT = 0x6E;// Set read/write pointer to a specific start address in specified DMP bank
static const uint8_t DMP_REG= 0x6F;// Register in DMP from which to read or to which to write
static const uint8_t DMP_REG_1= 0x70;
static const uint8_t DMP_REG_2= 0x71 ;
static const uint8_t FIFO_COUNTH= 0x72;
static const uint8_t FIFO_COUNTL= 0x73;
static const uint8_t FIFO_R_W = 0x74;
static const uint8_t WHO_AM_I_MPU9250 = 0x75 ;// Should return = 0x71;
static const uint8_t XA_OFFSET_H= 0x77;
static const uint8_t XA_OFFSET_L= 0x78;
static const uint8_t YA_OFFSET_H= 0x7A;
static const uint8_t YA_OFFSET_L= 0x7B;
static const uint8_t ZA_OFFSET_H= 0x7D;
static const uint8_t ZA_OFFSET_L= 0x7E;

// EM7180 SENtral register map
// see http:;//www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf
//
static const uint8_t EM7180_QX = 0x00;// this is a 32-bit normalized floating point number read from registers = 0x00-03
static const uint8_t EM7180_QY = 0x04;// this is a 32-bit normalized floating point number read from registers = 0x04-07
static const uint8_t EM7180_QZ = 0x08;// this is a 32-bit normalized floating point number read from registers = 0x08-0B
static const uint8_t EM7180_QW = 0x0C;// this is a 32-bit normalized floating point number read from registers = 0x0C-0F
static const uint8_t EM7180_QTIME= 0x10;// this is a 16-bit unsigned integer read from registers = 0x10-11
static const uint8_t EM7180_MX = 0x12;// int16_t from registers = 0x12-13
static const uint8_t EM7180_MY = 0x14;// int16_t from registers = 0x14-15
static const uint8_t EM7180_MZ = 0x16;// int16_t from registers = 0x16-17
static const uint8_t EM7180_MTIME= 0x18;// uint16_t from registers = 0x18-19
static const uint8_t EM7180_AX = 0x1A;// int16_t from registers = 0x1A-1B
static const uint8_t EM7180_AY = 0x1C;// int16_t from registers = 0x1C-1D
static const uint8_t EM7180_AZ = 0x1E;// int16_t from registers = 0x1E-1F
static const uint8_t EM7180_ATIME= 0x20;// uint16_t from registers = 0x20-21
static const uint8_t EM7180_GX = 0x22;// int16_t from registers = 0x22-23
static const uint8_t EM7180_GY = 0x24;// int16_t from registers = 0x24-25
static const uint8_t EM7180_GZ = 0x26;// int16_t from registers = 0x26-27
static const uint8_t EM7180_GTIME= 0x28;// uint16_t from registers = 0x28-29
static const uint8_t EM7180_Baro = 0x2A;// start of two-byte MS5637 pressure data, 16-bit signed interger
static const uint8_t EM7180_BaroTIME = 0x2C;// start of two-byte MS5637 pressure timestamp, 16-bit unsigned
static const uint8_t EM7180_Temp = 0x2E;// start of two-byte MS5637 temperature data, 16-bit signed interger
static const uint8_t EM7180_TempTIME = 0x30;// start of two-byte MS5637 temperature timestamp, 16-bit unsigned
static const uint8_t EM7180_QRateDivisor = 0x32;// uint8_t 
static const uint8_t EM7180_EnableEvents = 0x33;
static const uint8_t EM7180_HostControl= 0x34;
static const uint8_t EM7180_EventStatus= 0x35;
static const uint8_t EM7180_SensorStatus = 0x36;
static const uint8_t EM7180_SentralStatus= 0x37;
static const uint8_t EM7180_AlgorithmStatus= 0x38;
static const uint8_t EM7180_FeatureFlags = 0x39;
static const uint8_t EM7180_ParamAcknowledge = 0x3A;
static const uint8_t EM7180_SavedParamByte0= 0x3B;
static const uint8_t EM7180_SavedParamByte1= 0x3C;
static const uint8_t EM7180_SavedParamByte2= 0x3D;
static const uint8_t EM7180_SavedParamByte3= 0x3E;
static const uint8_t EM7180_ActualMagRate= 0x45;
static const uint8_t EM7180_ActualAccelRate= 0x46;
static const uint8_t EM7180_ActualGyroRate = 0x47;
static const uint8_t EM7180_ActualBaroRate = 0x48;
static const uint8_t EM7180_ActualTempRate = 0x49;
static const uint8_t EM7180_ErrorRegister= 0x50;
static const uint8_t EM7180_AlgorithmControl = 0x54;
static const uint8_t EM7180_MagRate= 0x55;
static const uint8_t EM7180_AccelRate= 0x56;
static const uint8_t EM7180_GyroRate = 0x57;
static const uint8_t EM7180_BaroRate = 0x58;
static const uint8_t EM7180_TempRate = 0x59;
static const uint8_t EM7180_LoadParamByte0 = 0x60;
static const uint8_t EM7180_LoadParamByte1 = 0x61;
static const uint8_t EM7180_LoadParamByte2 = 0x62;
static const uint8_t EM7180_LoadParamByte3 = 0x63;
static const uint8_t EM7180_ParamRequest = 0x64;
static const uint8_t EM7180_ROMVersion1= 0x70;
static const uint8_t EM7180_ROMVersion2= 0x71;
static const uint8_t EM7180_RAMVersion1= 0x72;
static const uint8_t EM7180_RAMVersion2= 0x73;
static const uint8_t EM7180_ProductID= 0x90;
static const uint8_t EM7180_RevisionID = 0x91;
static const uint8_t EM7180_RunStatus= 0x92;
static const uint8_t EM7180_UploadAddress= 0x94 ;// uint16_t registers = 0x94 (MSB)-5(LSB);
static const uint8_t EM7180_UploadData = 0x96;
static const uint8_t EM7180_CRCHost= 0x97;// uint32_t from registers = 0x97-9A
static const uint8_t EM7180_ResetRequest = 0x9B ;
static const uint8_t EM7180_PassThruStatus = 0x9E ;
static const uint8_t EM7180_PassThruControl= 0xA0;
static const uint8_t EM7180_ACC_LPF_BW = 0x5B;//Register GP36;
static const uint8_t EM7180_GYRO_LPF_BW= 0x5C;//Register GP37
static const uint8_t EM7180_BARO_LPF_BW= 0x5D;//Register GP38

static const uint8_t EM7180_ADDRESS= 0x28 ;// Address of the EM7180 SENtral sensor hub
static const uint8_t M24512DFM_DATA_ADDRESS = 0x50 ;// Address of the 500 page M24512DFM EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
static const uint8_t M24512DFM_IDPAGE_ADDRESS = 0x58 ;// Address of the single M24512DFM lockable EEPROM ID page
static const uint8_t MPU9250_ADDRESS = 0x68;// Device address when ADO = 0
static const uint8_t MS5637_ADDRESS = 0x76;// Address of altimeter

static const uint8_t ADC_256= 0x00 ;// define pressure and temperature conversion rates
static const uint8_t ADC_512= 0x02;
static const uint8_t ADC_1024 = 0x04;
static const uint8_t ADC_2048 = 0x06;
static const uint8_t ADC_4096 = 0x08;
static const uint8_t ADC_8192 = 0x0A;
static const uint8_t ADC_D1 = 0x40;
static const uint8_t ADC_D2 = 0x50;

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

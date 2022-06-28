/* 06/29/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include "USFS.h"

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



static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) 
{
    uint8_t temp[2];
    temp[0] = subAddress;
    temp[1] = data;
    Wire.transfer(address, &temp[0], 2, NULL, 0); 
}

static uint8_t readByte(uint8_t address, uint8_t subAddress) 
{
    uint8_t temp[1];
    Wire.transfer(address, &subAddress, 1, &temp[0], 1);
    return temp[0];
}

static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) 
{
    Wire.transfer(address, &subAddress, 1, dest, count); 
}

USFS::USFS(uint8_t intPin, bool passThru)
{
    pinMode(intPin, INPUT);
    _intPin = intPin;
    _passThru = passThru;   
}

uint8_t USFS::checkEM7180Errors(){
    uint8_t c = readByte(EM7180_ADDRESS, EM7180_ErrorRegister); // check error register
    return c;
}

uint8_t USFS::checkEM7180Status(){
    // Check event status register, way to check data ready by polling rather than interrupt
    uint8_t c = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register and interrupt
    return c;
}


void USFS::getChipID()
{
    // Read SENtral device information
    uint16_t ROM1 = readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
    uint16_t ROM2 = readByte(EM7180_ADDRESS, EM7180_ROMVersion2);
    Serial.print("EM7180 ROM Version: 0x"); Serial.print(ROM1, HEX); Serial.println(ROM2, HEX); Serial.println("Should be: 0xE609");
    uint16_t RAM1 = readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
    uint16_t RAM2 = readByte(EM7180_ADDRESS, EM7180_RAMVersion2);
    Serial.print("EM7180 RAM Version: 0x"); Serial.print(RAM1); Serial.println(RAM2);
    uint8_t PID = readByte(EM7180_ADDRESS, EM7180_ProductID);
    Serial.print("EM7180 ProductID: 0x"); Serial.print(PID, HEX); Serial.println(" Should be: 0x80");
    uint8_t RID = readByte(EM7180_ADDRESS, EM7180_RevisionID);
    Serial.print("EM7180 RevisionID: 0x"); Serial.print(RID, HEX); Serial.println(" Should be: 0x02");
}


void USFS::initEM7180(uint8_t accBW, uint8_t gyroBW, uint16_t accFS, uint16_t gyroFS, uint16_t magFS, uint8_t QRtDiv, uint8_t magRt, uint8_t accRt, uint8_t gyroRt, uint8_t baroRt)
{
    uint16_t EM7180_mag_fs, EM7180_acc_fs, EM7180_gyro_fs; // EM7180 sensor full scale ranges
    uint8_t param[4];      

    // Enter EM7180 initialized state
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // Force initialize
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers

    //Setup LPF bandwidth (BEFORE setting ODR's)
    writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, accBW);   // accBW = 3 = 41Hz
    writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, gyroBW); // gyroBW = 3 = 41Hz
    // Set accel/gyro/mag desired ODR rates
    writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, QRtDiv); // quat rate = gyroRt/(1 QRTDiv)
    writeByte(EM7180_ADDRESS, EM7180_MagRate, magRt); // 0x64 = 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_AccelRate, accRt); // 200/10 Hz, 0x14 = 200 Hz
    writeByte(EM7180_ADDRESS, EM7180_GyroRate, gyroRt); // 200/10 Hz, 0x14 = 200 Hz
    writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | baroRt);  // set enable bit and set Baro rate to 25 Hz, rate = baroRt/2, 0x32 = 25 Hz

    // Configure operating mode
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
    // Enable interrupt to host upon certain events
    // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);
    // Enable EM7180 run mode
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
    delay(100);

    // EM7180 parameter adjustments
    Serial.println("Beginning Parameter Adjustments");

    // Read sensor default FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read parameter 74
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    byte param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4A)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
    EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
    Serial.print("Magnetometer Default Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
    Serial.print("Accelerometer Default Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4B)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
    Serial.print("Gyroscope Default Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

    //Disable stillness mode for balancing robot application
    EM7180_set_integer_param (0x49, 0x00);

    //Write desired sensor full scale ranges to the EM7180
    EM7180_set_mag_acc_FS (magFS, accFS); // 1000 uT == 0x3E8, 8 g == 0x08
    EM7180_set_gyro_FS (gyroFS); // 2000 dps == 0x7D0

    // Read sensor new FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read  parameter 74
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4A)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
    EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
    Serial.print("Magnetometer New Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
    Serial.print("Accelerometer New Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4B)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
    Serial.print("Gyroscope New Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm


    // Read EM7180 status
    uint8_t runStatus = readByte(EM7180_ADDRESS, EM7180_RunStatus);
    if(runStatus & 0x01) Serial.println(" EM7180 run status = normal mode");
    uint8_t algoStatus = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
    if(algoStatus & 0x01) Serial.println(" EM7180 standby status");
    if(algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
    if(algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
    if(algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
    if(algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
    if(algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");
    uint8_t passthruStatus = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
    if(passthruStatus & 0x01) Serial.print(" EM7180 in passthru mode!");
    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus);
    if(eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
    if(eventStatus & 0x02) Serial.println(" EM7180 Error");
    if(eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
    if(eventStatus & 0x08) Serial.println(" EM7180 new mag result");
    if(eventStatus & 0x10) Serial.println(" EM7180 new accel result");
    if(eventStatus & 0x20) Serial.println(" EM7180 new gyro result"); 

    delay(1000); // give some time to read the screen

    // Check sensor status
    uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);
    Serial.print(" EM7180 sensor status = "); Serial.println(sensorStatus);
    if(sensorStatus & 0x01) Serial.print("Magnetometer not acknowledging!");
    if(sensorStatus & 0x02) Serial.print("Accelerometer not acknowledging!");
    if(sensorStatus & 0x04) Serial.print("Gyro not acknowledging!");
    if(sensorStatus & 0x10) Serial.print("Magnetometer ID not recognized!");
    if(sensorStatus & 0x20) Serial.print("Accelerometer ID not recognized!");
    if(sensorStatus & 0x40) Serial.print("Gyro ID not recognized!");

    Serial.print("Actual MagRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualMagRate)); Serial.println(" Hz"); 
    Serial.print("Actual AccelRate = "); Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualAccelRate)); Serial.println(" Hz"); 
    Serial.print("Actual GyroRate = "); Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualGyroRate)); Serial.println(" Hz"); 
    Serial.print("Actual BaroRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualBaroRate)); Serial.println(" Hz"); 
}


void USFS::loadfwfromEEPROM()
{
    // Check which sensors can be detected by the EM7180
    uint8_t featureflag = readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
    if(featureflag & 0x01)  Serial.println("A barometer is installed");
    if(featureflag & 0x02)  Serial.println("A humidity sensor is installed");
    if(featureflag & 0x04)  Serial.println("A temperature sensor is installed");
    if(featureflag & 0x08)  Serial.println("A custom sensor is installed");
    if(featureflag & 0x10)  Serial.println("A second custom sensor is installed");
    if(featureflag & 0x20)  Serial.println("A third custom sensor is installed");

    delay(1000); // give some time to read the screen

    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    byte STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
    int count = 0;
    while(!STAT) {
        writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
        delay(500);  
        count++;  
        STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
        if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
        if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
        if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
        if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
        if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
        if(count > 10) break;
    }

    if(!(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04))  Serial.println("EEPROM upload successful!");
}

void USFS::readSENtralAccelData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]);       // Read the six raw data registers into data array
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

int16_t USFS::readSENtralBaroData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    readBytes(EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}

void USFS::readSENtralGyroData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void USFS::readSENtralMagData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void USFS::readSENtralQuatData(float * destination)
{
    uint8_t rawData[16];  // x/y/z quaternion register data stored here
    readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]); // Read the sixteen raw data registers into data array
    destination[1] = uint32_reg_to_float (&rawData[0]);
    destination[2] = uint32_reg_to_float (&rawData[4]);
    destination[3] = uint32_reg_to_float (&rawData[8]);
    destination[0] = uint32_reg_to_float (&rawData[12]);   // SENtral stores quats as qx, qy, qz, q0!

}

int16_t USFS::readSENtralTempData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    readBytes(EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}



// ============================================================================


void USFS::readAccelData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void USFS::readGyroData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


float USFS::uint32_reg_to_float (uint8_t *buf)
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

float USFS::int32_reg_to_float (uint8_t *buf)
{
    union {
        int32_t i32;
        float f;
    } u;

    u.i32 =      (((int32_t)buf[0]) +
            (((int32_t)buf[1]) <<  8) +
            (((int32_t)buf[2]) << 16) +
            (((int32_t)buf[3]) << 24));
    return u.f;
}

void USFS::float_to_bytes (float param_val, uint8_t *buf) {
    union {
        float f;
        uint8_t comp[sizeof(float)];
    } u;
    u.f = param_val;
    for (uint8_t i=0; i < sizeof(float); i++) {
        buf[i] = u.comp[i];
    }
    //Convert to LITTLE ENDIAN
    for (uint8_t i=0; i < sizeof(float); i++) {
        buf[i] = buf[(sizeof(float)-1) - i];
    }
}

void USFS::EM7180_set_gyro_FS (uint16_t gyro_fs) {
    uint8_t bytes[4], STAT;
    bytes[0] = gyro_fs & (0xFF);
    bytes[1] = (gyro_fs >> 8) & (0xFF);
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCB)) {
        STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void USFS::EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs) {
    uint8_t bytes[4], STAT;
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Mag LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Mag MSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Acc LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Acc MSB
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCA)) {
        STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void USFS::EM7180_set_integer_param (uint8_t param, uint32_t param_val) {
    uint8_t bytes[4], STAT;
    bytes[0] = param_val & (0xFF);
    bytes[1] = (param_val >> 8) & (0xFF);
    bytes[2] = (param_val >> 16) & (0xFF);
    bytes[3] = (param_val >> 24) & (0xFF);
    param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==param)) {
        STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void USFS::EM7180_set_float_param (uint8_t param, float param_val) {
    uint8_t bytes[4], STAT;
    float_to_bytes (param_val, &bytes[0]);
    param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==param)) {
        STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}


void USFS::readMagData(int16_t * destination)
{
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
            destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
        }
    }
}

int16_t USFS::readTempData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
    return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void USFS::initAK8963(uint8_t Mscale, uint8_t Mmode, float * destination)
{
    _Mmode = Mmode;
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
    delay(20);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(20);
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
    destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
    destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
    _fuseROMx = destination[0];
    _fuseROMy = destination[1];
    _fuseROMz = destination[2];
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
    delay(20);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    delay(20);
}


void USFS::initMPU9250(uint8_t Ascale, uint8_t Gscale)
{  
    // wake up device
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
    delay(100); // Wait for all registers to reset 

    // get stable time source
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    delay(200); 

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
    // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x02; // Clear Fchoice bits [1:0] 
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer 
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void USFS::accelgyrocalMPU9250(float * dest1, float * dest2)
{  
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
    // else use the internal oscillator, bits 2:0 = 001
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
    writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    delay(200);                                    

    // Configure device for bias calculation
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++) {
        if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFE;
    data[1] = (accel_bias_reg[0])      & 0xFE;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFE;
    data[3] = (accel_bias_reg[1])      & 0xFE;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFE;
    data[5] = (accel_bias_reg[2])      & 0xFE;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    /*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
        writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
        writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
        writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
        writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
        writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
     */
    // Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void USFS::MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[6];
    int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
    float factoryTrim[6];
    uint8_t FS = 0;

    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

    for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }

    for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the accelerometer for self-test
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    delay(25);  // Delay a while to let the device stabilize

    for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }

    for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }   

    // Configure the gyro and accelerometer for normal operation
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
    delay(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
    selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
    selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
    selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
    selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
    selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    for (int i = 0; i < 3; i++) {
        destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
        destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
    }

}

void USFS::SENtralPassThroughMode()
{
    // First put SENtral in standby mode
    uint8_t c = readByte(EM7180_ADDRESS, EM7180_AlgorithmControl);
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, c | 0x01);
    //  c = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
    //  Serial.print("c = "); Serial.println(c);
    // Verify standby status
    // if(readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus) & 0x01) {
    Serial.println("SENtral in standby mode"); 
    // Place SENtral in pass-through mode
    writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x01); 
    if(readByte(EM7180_ADDRESS, EM7180_PassThruStatus) & 0x01) {
        Serial.println("SENtral in pass-through mode");
    }
    else {
        Serial.println("ERROR! SENtral not in pass-through mode!");
    }
}



// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

void USFS::M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data)
{
    uint8_t temp[2] = {data_address1, data_address2};
    Wire.transfer(device_address, &temp[0], 2, NULL, 0); 
    Wire.transfer(device_address, &data, 1, NULL, 0); 
}


void USFS::M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{
    if(count > 128) 
    {
        count = 128;
        Serial.print("Page count cannot be more than 128 bytes!");
    }       
    uint8_t temp[2] = {data_address1, data_address2};
    Wire.transfer(device_address, &temp[0], 2, NULL, 0); 
    Wire.transfer(device_address, &dest[0], count, NULL, 0); 
}


uint8_t USFS::M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2)
{
    uint8_t data; // `data` will store the register data   
    Wire.beginTransmission(device_address);         // Initialize the Tx buffer
    Wire.write(data_address1);                // Put slave register address in Tx buffer
    Wire.write(data_address2);                // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(device_address, 1);  // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void USFS::M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{  
    uint8_t temp[2] = {data_address1, data_address2};
    Wire.transfer(device_address, &temp[0], 2, dest, count); 
}


// I2C communication with the MS5637 is a little different from that with the MPU9250 and most other sensors
// For the MS5637, we write commands, and the MS5637 sends data in response, rather than directly reading
// MS5637 registers

void USFS::MS5637Reset()
{
    uint8_t temp[1] = {MS5637_RESET};
    Wire.transfer(MS5637_ADDRESS, &temp[0], 1, NULL, 0);    
}

void USFS::MS5637PromRead(uint16_t * destination)
{
    uint8_t data[2] = {0,0};
    uint8_t temp[1];
    for (uint8_t ii = 0; ii < 7; ii++) {
        temp[0] = 0xA0 | ii << 1;
        Wire.transfer(MS5637_ADDRESS, &temp[0], 1, data, 2); 
        destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
    }
}

unsigned char USFS::MS5637checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
{
    int cnt;
    unsigned int n_rem = 0;
    unsigned char n_bit;

    n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
    n_prom[7] = 0;
    for(cnt = 0; cnt < 16; cnt++)
    {
        if(cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else         n_rem ^= (unsigned short)  (n_prom[cnt>>1]>>8);
        for(n_bit = 8; n_bit > 0; n_bit--)
        {
            if(n_rem & 0x8000)    n_rem = (n_rem<<1) ^ 0x3000;
            else                  n_rem = (n_rem<<1);
        }
    }
    n_rem = ((n_rem>>12) & 0x000F);
    return (n_rem ^ 0x00);
}

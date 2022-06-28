#include <Wire.h>   

#include "USFS.h"

/*static*/ void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  
    Wire.write(subAddress);           
    Wire.write(data);                 
    Wire.endTransmission();           
}

/*static*/ uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; 
    Wire.beginTransmission(address);         
    Wire.write(subAddress);	                 
    Wire.endTransmission(false);        
    
    
    Wire.requestFrom(address, (size_t) 1);   
    data = Wire.read();                      
    return data;                             
}

static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(address);   
    Wire.write(subAddress);            
    Wire.endTransmission(false);  
    
    uint8_t i = 0;
    
    Wire.requestFrom(address, (size_t) count);  
    while (Wire.available()) {
        dest[i++] = Wire.read(); }         
}

static float uint32_reg_to_float (uint8_t *buf)
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



void usfsSetGyroFs (uint16_t gyro_fs) {
    uint8_t bytes[4], STAT;
    bytes[0] = gyro_fs & (0xFF);
    bytes[1] = (gyro_fs >> 8) & (0xFF);
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); 
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); 
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); 
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); 
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB); 
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); 
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); 
    while(!(STAT==0xCB)) {
        STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); 
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); 
}

void usfsSetMagAccFs (uint16_t mag_fs, uint16_t acc_fs) {
    uint8_t bytes[4], STAT;
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); 
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); 
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); 
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); 
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA); 
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); 
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); 
    while(!(STAT==0xCA)) {
        STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); 
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); 
}

void usfsSetIntegerParam (uint8_t param, uint32_t param_val) {
    uint8_t bytes[4], STAT;
    bytes[0] = param_val & (0xFF);
    bytes[1] = (param_val >> 8) & (0xFF);
    bytes[2] = (param_val >> 16) & (0xFF);
    bytes[3] = (param_val >> 24) & (0xFF);
    param = param | 0x80; 
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); 
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); 
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); 
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); 
    while(!(STAT==param)) {
        STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); 
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); 
}

void usfsReadQuaternion(float * destination)
{
    uint8_t rawData[16];  
    readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]);       
    destination[0] = uint32_reg_to_float (&rawData[0]);
    destination[1] = uint32_reg_to_float (&rawData[4]);
    destination[2] = uint32_reg_to_float (&rawData[8]);
    destination[3] = uint32_reg_to_float (&rawData[12]);  

}

void usfsReadAccelerometer(int16_t * destination)
{
    uint8_t rawData[6];  
    readBytes(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]);       
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void usfsReadGyrometer(int16_t * destination)
{
    uint8_t rawData[6];  
    readBytes(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]);  
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void usfsReadMagnetometer(int16_t * destination)
{
    uint8_t rawData[6];  
    readBytes(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]);  
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

int16_t usfsReadBarometer()
{
    uint8_t rawData[2];  
    readBytes(EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0]);  
    return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   
}

int16_t usfsReadTemperature()
{
    uint8_t rawData[2];  
    readBytes(EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0]);  
    return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   
}

uint8_t usfsReadRom1()
{
    return readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
}

uint8_t usfsReadRom2(void)
{
    return readByte(EM7180_ADDRESS, EM7180_ROMVersion2);
}

uint8_t usfsReadRam1(void)
{
    return readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
}

uint8_t usfsReadRam2(void)
{
    return readByte(EM7180_ADDRESS, EM7180_RAMVersion2);
}

uint8_t usfsReadPid(void)
{
    return readByte(EM7180_ADDRESS, EM7180_ProductID);
}

uint8_t usfsReadRid(void)
{
    return readByte(EM7180_ADDRESS, EM7180_RevisionID);
}

static bool hasFeature(uint8_t mask)
{
    return (bool)readByte(EM7180_ADDRESS, EM7180_FeatureFlags) & mask;
}

bool usfsHasBarometer(void)
{
    return hasFeature(0x01);
}

bool usfsHasHumiditySensor(void)
{
    return hasFeature(0x02);
}

bool usfsHasTemperatureSensor(void)
{
    return hasFeature(0x04);
}

bool usfsHasCustomSensor(void)
{
    return hasFeature(0x08);
}

bool usfsHasSecondCustomSensor(void)
{
    return hasFeature(0x10);
}

bool usfsHasThirdCustomSensor(void)
{
    return hasFeature(0x20);
}

static bool eepromStatus(uint8_t mask)
{
    return bool(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & mask);
}

bool usfsEepromDetected(void)
{
    return eepromStatus(0x01);
}

bool usfsEepromUploaded(void)
{
    return eepromStatus(0x02);
}

bool usfsEepromIncorrect(void)
{
    return eepromStatus(0x04);
}

bool usfsEepromInitialized(void)
{
    return eepromStatus(0x08);
}

bool usfsEepromNotDetected(void)
{
    return eepromStatus(0x10);
}

void usfsReset(void)
{
    writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
}

bool usfsEepromUploadSuccessful(void)
{
    return (bool)(!(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04));
}

void usfsBegin(
        uint8_t quatRateDivisor,
        uint8_t magRate,
        uint8_t accelRate,
        uint8_t gyroRate,
        uint8_t baroRate)
{
    // Set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); 

    // Make sure pass through mode is off
    writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); 

    // Force initialize
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); 

    // Set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); 

    // Set up LPF bandwidth (BEFORE setting ODR's)
    writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, 0x03); // 41Hz
    writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, 0x03); // 41Hz

    // Set accel/gyro/mage desired ODR rates
    writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, quatRateDivisor);
    writeByte(EM7180_ADDRESS, EM7180_MagRate, magRate); 
    writeByte(EM7180_ADDRESS, EM7180_AccelRate, accelRate);
    writeByte(EM7180_ADDRESS, EM7180_GyroRate, gyroRate);

    // Set enable bit and set Baro rate
    writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | baroRate);  

    // Configure operating mode
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data

    // Enable interrupt to host upon certain events choose host interrupts when
    // any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02),
    // or the SENtral needs to be reset(0x01)
    writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);

    // Enable EM7180 run mode
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); 
 }

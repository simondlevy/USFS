#include <Arduino.h>   
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
        uint8_t baroRate,
        bool verbose)
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

    // EM7180 parameter adjustments
    if (verbose) {
        Serial.println("Beginning Parameter Adjustments");
    }

    // Read sensor default FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); 
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);
    uint8_t param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4A)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    uint8_t param[4] = {};
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    uint16_t EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
    uint16_t EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);

    if (verbose) {
        Serial.print("Magnetometer Default Full Scale Range: +/-");
        Serial.print(EM7180_mag_fs);
        Serial.println("uT");
        Serial.print("Accelerometer Default Full Scale Range: +/-");
        Serial.print(EM7180_acc_fs);
        Serial.println("g");
    }

    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B);
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4B)) {
        param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    uint16_t EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);

    if (verbose) {
        Serial.print("Gyroscope Default Full Scale Range: +/-");
        Serial.print(EM7180_gyro_fs);
        Serial.println("dps");
    }

    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

    //Disable stillness mode
    usfsSetIntegerParam (0x49, 0x00);

    //Write desired sensor full scale ranges to the EM7180
    usfsSetMagAccFs (0x3E8, 0x08); // 1000 uT, 8 g
    usfsSetGyroFs (0x7D0); // 2000 dps

    // Read sensor new FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); 
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); 
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

    if (verbose) {
        Serial.print("Magnetometer New Full Scale Range: +/-");
        Serial.print(EM7180_mag_fs);
        Serial.println("uT");
        Serial.print("Accelerometer New Full Scale Range: +/-");
        Serial.print(EM7180_acc_fs);
        Serial.println("g");
    }

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

    if (verbose) {
        Serial.print("Gyroscope New Full Scale Range: +/-");
        Serial.print(EM7180_gyro_fs);
        Serial.println("dps");
    }

    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

    // Read EM7180 status
    uint8_t runStatus = readByte(EM7180_ADDRESS, EM7180_RunStatus);
    uint8_t algoStatus = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);

    if (verbose) {
        if (runStatus & 0x01) Serial.println(" EM7180 run status = normal mode");
        if (algoStatus & 0x01) Serial.println(" EM7180 standby status");
        if (algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
        if (algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
        if (algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
        if (algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
        if (algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");
    }

    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus);
    uint8_t passthruStatus = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);

    if (verbose) {
        if (passthruStatus & 0x01) Serial.print(" EM7180 in passthru mode!");
        if (eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
        if (eventStatus & 0x02) Serial.println(" EM7180 Error");
        if (eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
        if (eventStatus & 0x08) Serial.println(" EM7180 new mag result");
        if (eventStatus & 0x10) Serial.println(" EM7180 new accel result");
        if (eventStatus & 0x20) Serial.println(" EM7180 new gyro result"); 
    }

    delay(1000); // give some time to read the screen

    uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);

    // Check sensor status
    if (verbose) {
        Serial.print(" EM7180 sensor status = ");
        Serial.println(sensorStatus);
        if (sensorStatus & 0x01) Serial.print("Magnetometer not acknowledging!");
        if (sensorStatus & 0x02) Serial.print("Accelerometer not acknowledging!");
        if (sensorStatus & 0x04) Serial.print("Gyro not acknowledging!");
        if (sensorStatus & 0x10) Serial.print("Magnetometer ID not recognized!");
        if (sensorStatus & 0x20) Serial.print("Accelerometer ID not recognized!");
        if (sensorStatus & 0x40) Serial.print("Gyro ID not recognized!");

        Serial.print("Actual MagRate = ");
        Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualMagRate));
        Serial.println(" Hz"); 
        Serial.print("Actual AccelRate = ");
        Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualAccelRate));
        Serial.println(" Hz"); 
        Serial.print("Actual GyroRate = ");
        Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualGyroRate));
        Serial.println(" Hz"); 
        Serial.print("Actual BaroRate = ");
        Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualBaroRate));
        Serial.println(" Hz"); 
    }

} // usfsBegin()

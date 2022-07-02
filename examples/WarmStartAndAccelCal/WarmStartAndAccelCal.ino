/* 
   Calibration sketch support for USFS

   Copyright (C) 2018 Simon D. Levy

   Adapted from Kris Winer's sketch

   This file is part of USFS.

   USFS is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   USFS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with USFS.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Wire.h>

#include "USFS.h"
#include "USFS_old.h"

// Calibration registers
static const uint8_t GP36 = 0x5B;
static const uint8_t GP37 = 0x5C;
static const uint8_t GP38 = 0x5D;
static const uint8_t GP39 = 0x5E;
static const uint8_t GP40 = 0x5F;
static const uint8_t GP50 = 0x69;
static const uint8_t GP51 = 0x6A;
static const uint8_t GP52 = 0x6B;
static const uint8_t GP53 = 0x6C;
static const uint8_t GP54 = 0x6D;
static const uint8_t GP55 = 0x6E;
static const uint8_t GP56 = 0x6F;

static const uint8_t M24512DFM_DATA_ADDRESS   = 0x50;   // Address of the 500 page M24512DRC EEPROM data buffer, 1024 bits (128 8-bit bytes) per page

static const float MAGNETIC_DECLINATION =  13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04


struct acc_cal
{
    int16_t accZero_max[3];
    int16_t accZero_min[3];
};

struct Sentral_WS_params
{
    uint8_t Sen_param[35][4];
};

/*************************************************************************************************/
/*************                                                                     ***************/
/*************                        Global Scope Variables                       ***************/
/*************                                                                     ***************/
/*************************************************************************************************/

// General purpose variables
static bool               accel_cal;
static int16_t            accel_cal_saved;
static uint16_t           calibratingA;
static acc_cal            global_conf;
static Sentral_WS_params  WS_params;

//===================================================================================================================
//====== Sentral parameter management functions
//===================================================================================================================

static void set_integer_param (uint8_t param, uint32_t param_val)
{
    uint8_t bytes[4];
    bytes[0] = param_val & (0xFF);
    bytes[1] = (param_val >> 8) & (0xFF);
    bytes[2] = (param_val >> 16) & (0xFF);
    bytes[3] = (param_val >> 24) & (0xFF);

    // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    param = param | 0x80;

    usfsLoadParamBytes(bytes);

    usfs2_requestParamRead(param);

    // Request parameter transfer procedure
    usfs2_algorithmControlRequestParameterTransfer();

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    while (true) {
        if (usfs2_getParamAcknowledge() == param) break;
    }

    // Parameter request = 0 to end parameter transfer process
    usfs2_requestParamRead(0x00);
    usfs2_algorithmControlReset(); // Re-start algorithm
}

static void set_WS_params()
{
    uint8_t param = 1;
    uint8_t stat;

    // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    param = param | 0x80;

    usfsLoadParamBytes(WS_params.Sen_param[0]);

    usfs2_requestParamRead(param);

    // Request parameter transfer procedure
    usfs2_algorithmControlRequestParameterTransfer();

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    stat = usfs2_getParamAcknowledge();
    while(!(stat==param)) {
        stat = usfs2_getParamAcknowledge();
    }
    for(uint8_t i=1; i<35; i++) {
        param = (i+1) | 0x80;
        usfsLoadParamBytes(WS_params.Sen_param[i]);
        usfs2_requestParamRead(param);

        // Check the parameter acknowledge register and loop until the result matches parameter request byte
        stat = usfs2_getParamAcknowledge();
        while(!(stat==param))
        {
            stat = usfs2_getParamAcknowledge();
        }
    }
    // Parameter request = 0 to end parameter transfer process
    usfs2_requestParamRead(0x00);
}

static void USFS_get_WS_params()
{
    uint8_t param = 1;
    uint8_t stat;

    usfs2_requestParamRead(param);
    delay(10);

    // Request parameter transfer procedure
    usfs2_algorithmControlRequestParameterTransfer();
    delay(10);

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    stat = usfs2_getParamAcknowledge();
    while(!(stat==param))
    {
        stat = usfs2_getParamAcknowledge();
    }

    // Parameter is the decimal value with the MSB set low (default) to
    // indicate a paramter read processs
    usfsReadSavedParamBytes(WS_params.Sen_param[0]);

    for(uint8_t i=1; i<35; i++)
    {
        param = (i+1);
        usfs2_requestParamRead(param);
        delay(10);

        // Check the parameter acknowledge register and loop until the result matches parameter request byte
        stat = usfs2_getParamAcknowledge();
        while(!(stat==param))
        {
            stat = usfs2_getParamAcknowledge();
        }
        usfsReadSavedParamBytes(WS_params.Sen_param[0]);
    }
    // Parameter request = 0 to end parameter transfer process
    usfs2_requestParamRead(0x00);

    // Re-start algorithm
    usfs2_algorithmControlReset();
}

static void USFS_acc_cal_upload()
{
    int64_t big_cal_num;
    union {
        int16_t cal_num;
        unsigned char cal_num_byte[2];
    };

    big_cal_num = (4096000000/(global_conf.accZero_max[0] -
                global_conf.accZero_min[0])) - 1000000;
    cal_num = (int16_t)big_cal_num;

    usfsWriteByte(GP36, cal_num_byte[0]);
    usfsWriteByte(GP37, cal_num_byte[1]);

    big_cal_num = (4096000000/(global_conf.accZero_max[1] -
                global_conf.accZero_min[1])) - 1000000;
    cal_num = (int16_t)big_cal_num;

    usfsWriteByte(GP38, cal_num_byte[0]);
    usfsWriteByte(GP39, cal_num_byte[1]);  

    big_cal_num = (4096000000/(global_conf.accZero_max[2] -
                global_conf.accZero_min[2])) - 1000000;
    cal_num = (int16_t)big_cal_num;

    usfsWriteByte(GP40, cal_num_byte[0]);
    usfsWriteByte(GP50, cal_num_byte[1]);

    big_cal_num = (((2048 - global_conf.accZero_max[0]) + (-2048 -
                    global_conf.accZero_min[0]))*100000)/4096;
    cal_num = (int16_t)big_cal_num;

    usfsWriteByte(GP51, cal_num_byte[0]);
    usfsWriteByte(GP52, cal_num_byte[1]);

    big_cal_num = (((2048 - global_conf.accZero_max[1]) + (-2048 -
                    global_conf.accZero_min[1]))*100000)/4096;
    cal_num = (int16_t)big_cal_num;

    usfsWriteByte(GP53, cal_num_byte[0]);
    usfsWriteByte(GP54, cal_num_byte[1]);

    big_cal_num = (((2048 - global_conf.accZero_max[2]) + (-2048 -
                    global_conf.accZero_min[2]))*100000)/4096;
    cal_num = -(int16_t)big_cal_num;

    usfsWriteByte(GP55, cal_num_byte[0]);
    usfsWriteByte(GP56, cal_num_byte[1]);
}

static void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(device_address);            // Initialize the Tx buffer
    Wire.write(data_address1);                         // Put slave register address in Tx buffer
    Wire.write(data_address2);                         // Put slave register address in Tx buffer
    Wire.endTransmission(false);                       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(device_address, (size_t)count);  // Read bytes from slave register address 
    while (Wire.available())
    {
        dest[i++] = Wire.read();
    }                                                   // Put read results in the Rx buffer
}

static void readSenParams()
{
    uint8_t data[140];
    uint8_t paramnum;
    M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 12, &data[128]); // Page 255
    delay(100);
    M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
    for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
    {
        for (uint8_t i= 0; i < 4; i++)
        {
            WS_params.Sen_param[paramnum][i] = data[(paramnum*4 + i)];
        }
    }
}

static void M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{
    if (count > 128)
    {
        count = 128;
        Serial.print("Page count cannot be more than 128 bytes!");
    }
    Wire.beginTransmission(device_address);   // Initialize the Tx buffer
    Wire.write(data_address1);                // Put slave register address in Tx buffer
    Wire.write(data_address2);                // Put slave register address in Tx buffer
    for(uint8_t i=0; i < count; i++)
    {
        Wire.write(dest[i]);                    // Put data in Tx buffer
    }
    Wire.endTransmission();                   // Send the Tx buffer
}

static void writeSenParams()
{
    uint8_t data[140];
    uint8_t paramnum;
    for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
    {
        for (uint8_t i= 0; i < 4; i++)
        {
            data[(paramnum*4 + i)] = WS_params.Sen_param[paramnum][i];
        }
    }
    M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 12, &data[128]); // Page 255
    delay(100);
    M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
}

static void writeAccCal()
{
    uint8_t data[12];
    uint8_t axis;
    for (axis = 0; axis < 3; axis++)
    {
        data[2*axis] = (global_conf.accZero_max[axis] & 0xff);
        data[(2*axis + 1)] = (global_conf.accZero_max[axis] >> 8);
        data[(2*axis + 6)] = (global_conf.accZero_min[axis] & 0xff);
        data[(2*axis + 7)] = (global_conf.accZero_min[axis] >> 8);
    }
    M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x8c, 12, data); // Page 255
}

static void Accel_cal_check(int16_t accelCount[3])
{
    static int64_t a[3] = {0, 0, 0}, b[3] = {0, 0, 0};

    if (calibratingA > 0)
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            if (accelCount[axis] > 1024)
            {
                // Sum up 512 readings
                a[axis] += accelCount[axis];
            }
            if (accelCount[axis] < -1024)
            {
                b[axis] += accelCount[axis];
            }
            // Clear global variables for next reading
            accelCount[axis] = 0;
        }

        // Calculate averages, and store values in EEPROM at end of calibration
        if (calibratingA == 1)
        {
            for (uint8_t axis = 0; axis < 3; axis++)
            {
                if (a[axis]>>9 > 1024)
                {
                    global_conf.accZero_max[axis] = a[axis]>>9;
                }
                if (b[axis]>>9 < -1024)
                {
                    global_conf.accZero_min[axis] = b[axis]>>9;
                }
                a[axis] = 0;
                b[axis] = 0;
            }

            //Write accZero to EEPROM
            delay(100);

            // Put the Sentral in pass-thru mode
            usfs2_setPassThroughMode();

            // Store accelerometer calibration data to the M24512DFM I2C EEPROM
            writeAccCal();

            // Take Sentral out of pass-thru mode and re-start algorithm
            usfs2_setMasterMode();
            accel_cal_saved++;
            if (accel_cal_saved > 6) accel_cal_saved = 0;
        }
        calibratingA--;
    }
}

static void readAccelCal()
{
    uint8_t data[12];
    uint8_t axis;

    M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x8c, 12, data); // Page 255
    for (axis = 0; axis < 3; axis++)
    {
        global_conf.accZero_max[axis] = ((int16_t)(data[(2*axis + 1)]<<8) | data[2*axis]);
        global_conf.accZero_min[axis] = ((int16_t)(data[(2*axis + 7)]<<8) | data[(2*axis + 6)]);
    }
}

//===================================================================================================================
//====== I2C Communication Support Functions
//===================================================================================================================

// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

void M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data)
{
    Wire.beginTransmission(device_address);   // Initialize the Tx buffer
    Wire.write(data_address1);                // Put slave register address in Tx buffer
    Wire.write(data_address2);                // Put slave register address in Tx buffer
    Wire.write(data);                         // Put data in Tx buffer
    Wire.endTransmission();                   // Send the Tx buffer
}

static void readParams(uint8_t paramId, uint8_t param[4])
{
    usfs2_requestParamRead(paramId); // Request to read parameter 74
    usfs2_algorithmControlRequestParameterTransfer(); // Request parameter transfer process
    while (true) {
        if (usfs2_getParamAcknowledge() == paramId) break;
    }
    usfsReadSavedParamBytes(param);
}

// ======================================================================================

void setup(void)
{  

#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif
    delay(100);

    Serial.begin(115200);
    delay(1000);

    // Start USFS interaction
    usfs2_begin();

    usfsReportChipId();

    Serial.flush();

    // Give some time to read the screen
    delay(4000);

    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    usfsLoadFirmware(true);

    // Take user input to choose Warm Start or not...
    Serial.println("Send '1' for Warm Start, '0' for no Warm Start");
    uint8_t serial_input = Serial.read();
    while(!(serial_input == '1') && !(serial_input == '0'))
    {
        serial_input = Serial.read();
        delay(500);
    }

    bool warm_start = (serial_input == '1');

    if (warm_start)
    {
        Serial.println("!!!Warm Start active!!!");

        // Put the Sentral in pass-thru mode
        usfs2_setPassThroughMode();

        // Fetch the WarmStart data from the M24512DFM I2C EEPROM
        readSenParams();

        // Take Sentral out of pass-thru mode and re-start algorithm
        usfs2_setMasterMode();
    } else
    {
        Serial.println("***No Warm Start***");
    }
    // Take user input to choose Warm Start or not...
    // "2" from the keboard is ASCII "1" which gives integer value 50
    // "0" from the keboard is ASCII "0" which gives integer value 48
    Serial.println("Send '2' to apply Accelerometer Cal, '0' to not apply Accelerometer Cal");
    serial_input = Serial.read();
    while(!(serial_input == '2') && !(serial_input == '0'))
    {
        serial_input = Serial.read();
        delay(500);
    }
    if (serial_input == '2')
    {
        accel_cal = true;
    } else
    {
        accel_cal = false;
    }
    if (accel_cal)
    {
        Serial.println("!!!Accel Cal Active!!!");

        // Put the Sentral in pass-thru mode
        usfs2_setPassThroughMode();

        // Fetch the WarmStart data from the M24512DFM I2C EEPROM
        readAccelCal();
        Serial.print("X-acc max: ");
        Serial.println(global_conf.accZero_max[0]);
        Serial.print("Y-acc max: ");
        Serial.println(global_conf.accZero_max[1]);
        Serial.print("Z-acc max: ");
        Serial.println(global_conf.accZero_max[2]);
        Serial.print("X-acc min: ");
        Serial.println(global_conf.accZero_min[0]);
        Serial.print("Y-acc min: ");
        Serial.println(global_conf.accZero_min[1]);
        Serial.print("Z-acc min: ");
        Serial.println(global_conf.accZero_min[2]);

        // Take Sentral out of pass-thru mode and re-start algorithm
        usfs2_setMasterMode();
    } else
    {
        Serial.println("***No Accel Cal***");
    }
    // Give some time to read the screen
    delay(1000);

    // Set SENtral in initialized state to configure registers
    usfs2_setRunDisable();

    // Load Accel Cal
    if (accel_cal)
    {
        USFS_acc_cal_upload();
    }

    // Force initialize
    usfs2_setRunEnable();

    // Load Warm Start parameters
    if (warm_start)
    {
        set_WS_params();
    }

    // Set SENtral in initialized state to configure registers
    usfs2_setRunDisable();

    //Setup LPF bandwidth (BEFORE setting ODR's)
    usfs2_setAccelLpfBandwidth(0x03); // 41Hz
    usfs2_setGyroLpfBandwidth(0x01); // 184Hz

    // Set accel/gyro/mage desired ODR rates
    usfs2_setQRateDivisor(0x02); // 100 Hz
    usfs2_setMagRate(0x64); // 100 Hz
    usfs2_setAccelRate(0x14); // 200/10 Hz
    usfs2_setGyroRate(0x14); // 200/10 Hz
    usfs2_setBaroRate(0x80 | 0x32);  // set enable bit and set Baro rate to 25 Hz

    // Configure operating mode
    usfs2_algorithmControlReset(); // read scale sensor data

    // Enable interrupt to host upon certain events
    // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    usfs2_enableEvents(0x07);

    // Enable USFS run mode
    usfs2_setRunEnable();
    delay(100);

    // USFS parameter adjustments
    Serial.println("Beginning Parameter Adjustments");

    // Read sensor default FS values from parameter space
    uint8_t param[4];
    readParams(0x4A, param);

    uint16_t USFS_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
    uint16_t USFS_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
    Serial.print("Magnetometer Default Full Scale Range: +/-");
    Serial.print(USFS_mag_fs);
    Serial.println("uT");
    Serial.print("Accelerometer Default Full Scale Range: +/-");
    Serial.print(USFS_acc_fs);
    Serial.println("g");
    readParams(0x4B, param); // Request to read  parameter 75
    uint16_t USFS_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
    Serial.print("Gyroscope Default Full Scale Range: +/-");
    Serial.print(USFS_gyro_fs);
    Serial.println("dps");
    usfs2_requestParamRead(0x00);//End parameter transfer
    usfs2_algorithmControlReset(); // re-enable algorithm

    // Disable stillness mode
    set_integer_param (0x49, 0x00);

    // Write desired sensor full scale ranges to the USFS
    usfs2_setMagAccFs (0x3E8, 0x08); // 1000 uT, 8 g
    usfs2_setGyroFs(0x7D0); // 2000 dps

    // Read sensor new FS values from parameter space
    readParams(0x4A, param);// Request to read  parameter 74
    USFS_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
    USFS_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
    Serial.print("Magnetometer New Full Scale Range: +/-");
    Serial.print(USFS_mag_fs);
    Serial.println("uT");
    Serial.print("Accelerometer New Full Scale Range: +/-");
    Serial.print(USFS_acc_fs);
    Serial.println("g");
    readParams(0x4B, param);// Request to read  parameter 75
    USFS_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
    Serial.print("Gyroscope New Full Scale Range: +/-");
    Serial.print(USFS_gyro_fs);
    Serial.println("dps");
    usfs2_requestParamRead(0x00);//End parameter transfer
    usfs2_algorithmControlReset(); // re-enable algorithm

    // Read USFS status
    if (usfsRunStatusIsNormal(usfsGetRunStatus())) {
        Serial.println(" USFS run status = normal mode");
    }

    uint8_t algoStatus = usfsGetAlgorithmStatus();

    if (usfsAlgorithmStatusIsStandby(algoStatus)) {
        Serial.println(" USFS standby status");
    }
    if (usfsAlgorithmStatusIsAlgorithmSlow(algoStatus)) {
        Serial.println(" USFS algorithm slow");
    }
    if (usfsAlgorithmStatusIsStillnessMode(algoStatus)) {
        Serial.println(" USFS in stillness mode");
    }
    if (usfsAlgorithmStatusIsCalibrationCompleted(algoStatus)) {
        Serial.println(" USFS mag calibration completed");
    }
    if (usfsAlgorithmStatusIsMagneticAnomaly(algoStatus)) {
        Serial.println(" USFS magnetic anomaly detected");
    }
    if (usfsAlgorithmStatusIsSensorUnreliable(algoStatus)) {
        Serial.println(" USFS unreliable sensor data");
    }

    if (usfsIsInPassThroughMode()) Serial.print(" USFS in passthru mode!");

    uint8_t eventStatus = usfsGetEventStatus();

    if (usfsEventStatusIsReset(eventStatus)) {
        Serial.println(" USFS CPU reset");
    }

    if (usfsEventStatusIsError(eventStatus)) {
        Serial.println(" USFS Error");
    }

    // Give some time to read the screen
    delay(1000);

    // Check sensor status
    usfsCheckSensorStatus(usfsGetSensorStatus());

    // Give some time to read the screen
    delay(1000);
}

void loop(void)
{  
    static bool warm_start_saved;

    // Used to control display output rate
    static uint32_t delt_t;
    static uint32_t count;

    static float Quat[4];

    static float ax;
    static float ay;
    static float az;

    uint8_t serial_input = Serial.read();

    if (serial_input == '1') {

        delay(100);
        USFS_get_WS_params();

        // Put the Sentral in pass-thru mode
        usfs2_setPassThroughMode();

        // Store WarmStart data to the M24512DFM I2C EEPROM
        writeSenParams();

        // Take Sentral out of pass-thru mode and re-start algorithm
        usfs2_setMasterMode();
        warm_start_saved = true;
    }

    if (serial_input == '2') {
        calibratingA = 512;
    }

    // Check event status register, way to check data ready by polling rather
    // than interrupt
    uint8_t eventStatus = usfsGetEventStatus(); // reading clears the

    if (usfsEventStatusIsError(eventStatus)) { 

        usfsReportError(eventStatus);
    }

    // if no errors, see if new data is ready
    if (usfsEventStatusIsAccelerometer(eventStatus)) { 

        int16_t accelCount[3];

        usfs2_readAccelerometer(accelCount[0], accelCount[1], accelCount[2]);

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*0.000488;  // get actual g value
        ay = (float)accelCount[1]*0.000488;    
        az = (float)accelCount[2]*0.000488;

        // Manages accelerometer calibration; is active when calibratingA > 0
        Accel_cal_check(accelCount);
    }

    if (usfsEventStatusIsQuaternion(eventStatus)) {
        usfs2_readQuaternion(Quat[0], Quat[1], Quat[2], Quat[3]);
    }

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;

    // update LCD once per half-second independent of read rate
    if (delt_t > 500) { 

        float Yaw  = atan2(2.0f * (Quat[1] * Quat[2] + Quat[0] * Quat[3]),
                Quat[0] * Quat[0] + Quat[1] * Quat[1] - Quat[2] * Quat[2] -
                Quat[3] * Quat[3]);
        Yaw   *= 180.0f / PI; 
        Yaw   += MAGNETIC_DECLINATION;
        if (Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360

        Serial.print("ax = ");
        Serial.print((int)1000*ax);  
        Serial.print(" ay = ");
        Serial.print((int)1000*ay); 
        Serial.print(" az = ");
        Serial.print((int)1000*az);
        Serial.println(" mg");
        Serial.print("Hardware yaw: ");
        Serial.println(Yaw, 2);

        if (warm_start_saved) {
            Serial.println("Warm Start configuration saved!");
        } 
        else {
            Serial.println("Send '1' to store Warm Start configuration");
        }
        if (accel_cal_saved > 0) {
            Serial.print("Accel Cals Complete:");
            Serial.println(accel_cal_saved);
        } 
        else {
            Serial.println("Send '2' to store Accel Cal");
        }

        count = millis(); 
    }
}


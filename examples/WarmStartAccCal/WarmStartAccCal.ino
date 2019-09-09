/* 
   WamStartAccAcall.ino: IMU calibration

   Copyright (C) 2018 Simon D. Levy

   Adapted from

     https://github.com/kriswiner/Teensy_Flight_Controller/blob/master/EM7180_MPU9250_BMP280

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

#include "EM7180.h"

EM7180 em7180;

static const uint8_t M24512DFM_DATA_ADDRESS   = 0x50;   // Address of the 500 page M24512DRC EEPROM data buffer, 1024 bits (128 8-bit bytes) per page

static const float MAGNETIC_DECLINATION =  13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04

/*************************************************************************************************/
/*************                                                                     ***************/
/*************                 Enumerators and Structure Variables                 ***************/
/*************                                                                     ***************/
/*************************************************************************************************/

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

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

//===================================================================================================================
//====== Sentral parameter management functions
//===================================================================================================================

static void EM7180_set_integer_param (uint8_t param, uint32_t param_val)
{
    uint8_t bytes[4];
    bytes[0] = param_val & (0xFF);
    bytes[1] = (param_val >> 8) & (0xFF);
    bytes[2] = (param_val >> 16) & (0xFF);
    bytes[3] = (param_val >> 24) & (0xFF);

    // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    param = param | 0x80;
    em7180.loadParamByte0(bytes[0]); //Param LSB
    em7180.loadParamByte1(bytes[1]);
    em7180.loadParamByte2(bytes[2]);
    em7180.loadParamByte3(bytes[3]); //Param MSB
    em7180.requestParamRead(param);

    // Request parameter transfer procedure
    em7180.algorithmControlRequestParameterTransfer();

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    while (true) {
        if (em7180.getParamAcknowledge() == param) break;
    }

    // Parameter request = 0 to end parameter transfer process
    em7180.requestParamRead(0x00);
    em7180.algorithmControlReset(); // Re-start algorithm
}

static void EM7180_set_WS_params()
{
    uint8_t param = 1;
    uint8_t stat;

    // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    param = param | 0x80;
    em7180.loadParamByte0(WS_params.Sen_param[0][0]);
    em7180.loadParamByte1(WS_params.Sen_param[0][1]);
    em7180.loadParamByte2(WS_params.Sen_param[0][2]);
    em7180.loadParamByte3(WS_params.Sen_param[0][3]);
    em7180.requestParamRead(param);

    // Request parameter transfer procedure
    em7180.algorithmControlRequestParameterTransfer();

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    stat = em7180.getParamAcknowledge();
    while(!(stat==param)) {
        stat = em7180.getParamAcknowledge();
    }
    for(uint8_t i=1; i<35; i++) {
        param = (i+1) | 0x80;
        em7180.loadParamByte0(WS_params.Sen_param[i][0]);
        em7180.loadParamByte1(WS_params.Sen_param[i][1]);
        em7180.loadParamByte2(WS_params.Sen_param[i][2]);
        em7180.loadParamByte3(WS_params.Sen_param[i][3]);
        em7180.requestParamRead(param);

        // Check the parameter acknowledge register and loop until the result matches parameter request byte
        stat = em7180.getParamAcknowledge();
        while(!(stat==param))
        {
            stat = em7180.getParamAcknowledge();
        }
    }
    // Parameter request = 0 to end parameter transfer process
    em7180.requestParamRead(0x00);
}

static void EM7180_get_WS_params()
{
    uint8_t param = 1;
    uint8_t stat;

    em7180.requestParamRead(param);
    delay(10);

    // Request parameter transfer procedure
    em7180.algorithmControlRequestParameterTransfer();
    delay(10);

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    stat = em7180.getParamAcknowledge();
    while(!(stat==param))
    {
        stat = em7180.getParamAcknowledge();
    }

    // Parameter is the decimal value with the MSB set low (default) to indicate a paramter read processs
    WS_params.Sen_param[0][0] = em7180.readSavedParamByte0();
    WS_params.Sen_param[0][1] = em7180.readSavedParamByte1();
    WS_params.Sen_param[0][2] = em7180.readSavedParamByte2();
    WS_params.Sen_param[0][3] = em7180.readSavedParamByte3();

    for(uint8_t i=1; i<35; i++)
    {
        param = (i+1);
        em7180.requestParamRead(param);
        delay(10);

        // Check the parameter acknowledge register and loop until the result matches parameter request byte
        stat = em7180.getParamAcknowledge();
        while(!(stat==param))
        {
            stat = em7180.getParamAcknowledge();
        }
        WS_params.Sen_param[i][0] = em7180.readSavedParamByte0();
        WS_params.Sen_param[i][1] = em7180.readSavedParamByte1();
        WS_params.Sen_param[i][2] = em7180.readSavedParamByte2();
        WS_params.Sen_param[i][3] = em7180.readSavedParamByte3();
    }
    // Parameter request = 0 to end parameter transfer process
    em7180.requestParamRead(0x00);

    // Re-start algorithm
    em7180.algorithmControlReset();
}

static void EM7180_acc_cal_upload()
{
    int64_t big_cal_num;
    union
    {
        int16_t cal_num;
        unsigned char cal_num_byte[2];
    };

    if (!accel_cal)
    {
        cal_num_byte[0] = 0;
        cal_num_byte[1] = 0;
    } else
    {
        big_cal_num = (4096000000/(global_conf.accZero_max[0] - global_conf.accZero_min[0])) - 1000000;
        cal_num = (int16_t)big_cal_num;
    }
    em7180.writeGp36(cal_num_byte[0]);
    em7180.writeGp37(cal_num_byte[1]);

    if (!accel_cal)
    {
        cal_num_byte[0] = 0;
        cal_num_byte[1] = 0;
    } else
    {
        big_cal_num = (4096000000/(global_conf.accZero_max[1] - global_conf.accZero_min[1])) - 1000000;
        cal_num = (int16_t)big_cal_num;
    }
    em7180.writeGp38(cal_num_byte[0]);
    em7180.writeGp39(cal_num_byte[1]);  

    if (!accel_cal)
    {
        cal_num_byte[0] = 0;
        cal_num_byte[1] = 0;
    } else
    {
        big_cal_num = (4096000000/(global_conf.accZero_max[2] - global_conf.accZero_min[2])) - 1000000;
        cal_num = (int16_t)big_cal_num;
    }
    em7180.writeGp40(cal_num_byte[0]);
    em7180.writeGp50(cal_num_byte[1]);

    if (!accel_cal)
    {
        cal_num_byte[0] = 0;
        cal_num_byte[1] = 0;
    } else
    {
        big_cal_num = (((2048 - global_conf.accZero_max[0]) + (-2048 - global_conf.accZero_min[0]))*100000)/4096;
        cal_num = (int16_t)big_cal_num;
    }
    em7180.writeGp51(cal_num_byte[0]);
    em7180.writeGp52(cal_num_byte[1]);

    if (!accel_cal)
    {
        cal_num_byte[0] = 0;
        cal_num_byte[1] = 0;
    } else
    {
        big_cal_num = (((2048 - global_conf.accZero_max[1]) + (-2048 - global_conf.accZero_min[1]))*100000)/4096;
        cal_num = (int16_t)big_cal_num;
    }
    em7180.writeGp53(cal_num_byte[0]);
    em7180.writeGp54(cal_num_byte[1]);

    if (!accel_cal)
    {
        cal_num_byte[0] = 0;
        cal_num_byte[1] = 0;
    } else
    {
        big_cal_num = (((2048 - global_conf.accZero_max[2]) + (-2048 - global_conf.accZero_min[2]))*100000)/4096;
        cal_num = -(int16_t)big_cal_num;
    }
    em7180.writeGp55(cal_num_byte[0]);
    em7180.writeGp56(cal_num_byte[1]);
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
            em7180.setPassThroughMode();

            // Store accelerometer calibration data to the M24512DFM I2C EEPROM
            writeAccCal();


            // Take Sentral out of pass-thru mode and re-start algorithm
            em7180.setMasterMode();
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

static void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(device_address);            // Initialize the Tx buffer
    Wire.write(data_address1);                         // Put slave register address in Tx buffer
    Wire.write(data_address2);                         // Put slave register address in Tx buffer
    Wire.endTransmission(NOSTOP);                  // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(device_address, (size_t)count);  // Read bytes from slave register address 
    while (Wire.available())
    {
        dest[i++] = Wire.read();
    }                                                   // Put read results in the Rx buffer
}

// simple function to scan for I2C devices on the bus
static void I2Cscan() 
{
    // scan for i2c devices
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for(address = 1; address < 127; address++ ) 
    {
        // Force serial output
        Serial.flush();

        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address<16) 
                Serial.print("0");
            Serial.print(address,HEX);
            Serial.println("  !");
            nDevices++;
        }
        else if (error==4) 
        {
            Serial.print("Unknow error at address 0x");
            if (address<16) 
                Serial.print("0");
            Serial.println(address,HEX);
        }    
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}

static void sensorError(const char * errmsg)
{
    Serial.println(errmsg);
    while (true)
        ;
}

static void readParams(uint8_t paramId, uint8_t param[4])
{
    em7180.requestParamRead(paramId); // Request to read parameter 74
    em7180.algorithmControlRequestParameterTransfer(); // Request parameter transfer process
    while (true) {
        if (em7180.getParamAcknowledge() == paramId) break;
    }
    param[0] = em7180.readSavedParamByte0();
    param[1] = em7180.readSavedParamByte1();
    param[2] = em7180.readSavedParamByte2();
    param[3] = em7180.readSavedParamByte3();
}

// ======================================================================================

void setup()
{  
    // Support both Teensy and traditional Arduino
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#elif defined(ESP8266)
    Wire.begin(0,2); // SDA (0), SCL (2) on ESP8266 
#else
    Wire.begin();
#endif

    delay(100);
    //Serial.begin(115200);
    Serial.begin(9600);
    delay(1000);

    // Should detect SENtral at 0x28
    I2Cscan();

    // Start EM7180 interaction
    em7180.begin();

    // Read SENtral device information
    Serial.print("EM7180 ROM Version: 0x");
    Serial.print(em7180.getRomVersion(), HEX);
    Serial.println(" (should be: 0xE609)");
    Serial.print("EM7180 RAM Version: 0x"); 
    Serial.println(em7180.getRamVersion());

    Serial.print("EM7180 ProductID: 0x"); 
    Serial.print(em7180.getProductId(), HEX);
    Serial.println(" Should be: 0x80");
    Serial.print("EM7180 RevisionID: 0x"); 
    Serial.print(em7180.getRevisionId(), HEX); 
    Serial.println(" Should be: 0x02");

    Serial.flush();

    // Give some time to read the screen
    delay(4000);

    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    for (uint8_t k=0; k<10; ++k) {

        uint8_t stat = em7180.getSentralStatus() & 0x01;

        if (stat & 0x01) Serial.println("EEPROM detected on the sensor bus!");
        if (stat & 0x02) Serial.println("EEPROM uploaded config file!");
        if (stat & 0x04) Serial.println("EEPROM CRC incorrect!");
        if (stat & 0x08) Serial.println("EM7180 in initialized state!");
        if (stat & 0x10) Serial.println("No EEPROM detected!");

        if (stat) break;

        em7180.requestReset();

        delay(500);  
    }

    if (!(em7180.getSentralStatus() & 0x04))  Serial.println("EEPROM upload successful!");

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
        em7180.setPassThroughMode();

        // Fetch the WarmStart data from the M24512DFM I2C EEPROM
        readSenParams();

        // Take Sentral out of pass-thru mode and re-start algorithm
        em7180.setMasterMode();
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
        accel_cal = 1;
    } else
    {
        accel_cal = 0;
    }
    if (accel_cal)
    {
        Serial.println("!!!Accel Cal Active!!!");

        // Put the Sentral in pass-thru mode
        em7180.setPassThroughMode();

        // Fetch the WarmStart data from the M24512DFM I2C EEPROM
        readAccelCal();
        Serial.print("X-acc max: "); Serial.println(global_conf.accZero_max[0]);
        Serial.print("Y-acc max: "); Serial.println(global_conf.accZero_max[1]);
        Serial.print("Z-acc max: "); Serial.println(global_conf.accZero_max[2]);
        Serial.print("X-acc min: "); Serial.println(global_conf.accZero_min[0]);
        Serial.print("Y-acc min: "); Serial.println(global_conf.accZero_min[1]);
        Serial.print("Z-acc min: "); Serial.println(global_conf.accZero_min[2]);

        // Take Sentral out of pass-thru mode and re-start algorithm
        em7180.setMasterMode();
    } else
    {
        Serial.println("***No Accel Cal***");
    }
    // Give some time to read the screen
    delay(1000);

    // Set SENtral in initialized state to configure registers
    em7180.setRunDisable();

    // Load Accel Cal
    if (accel_cal)
    {
        EM7180_acc_cal_upload();
    }

    // Force initialize
    em7180.setRunEnable();

    // Load Warm Start parameters
    if (warm_start)
    {
        EM7180_set_WS_params();
    }

    // Set SENtral in initialized state to configure registers
    em7180.setRunDisable();

    //Setup LPF bandwidth (BEFORE setting ODR's)
    em7180.setAccelLpfBandwidth(0x03); // 41Hz
    em7180.setGyroLpfBandwidth(0x01); // 184Hz

    // Set accel/gyro/mage desired ODR rates
    em7180.setQRateDivisor(0x02); // 100 Hz
    em7180.setMagRate(0x64); // 100 Hz
    em7180.setAccelRate(0x14); // 200/10 Hz
    em7180.setGyroRate(0x14); // 200/10 Hz
    em7180.setBaroRate(0x80 | 0x32);  // set enable bit and set Baro rate to 25 Hz

    // Configure operating mode
    em7180.algorithmControlReset(); // read scale sensor data

    // Enable interrupt to host upon certain events
    // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    em7180.enableEvents(0x07);

    // Enable EM7180 run mode
    em7180.setRunEnable();
    delay(100);

    // EM7180 parameter adjustments
    Serial.println("Beginning Parameter Adjustments");

    // Read sensor default FS values from parameter space
    uint8_t param[4];
    readParams(0x4A, param);

    uint16_t EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
    uint16_t EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
    Serial.print("Magnetometer Default Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
    Serial.print("Accelerometer Default Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
    readParams(0x4B, param); // Request to read  parameter 75
    uint16_t EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
    Serial.print("Gyroscope Default Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
    em7180.requestParamRead(0x00);//End parameter transfer
    em7180.algorithmControlReset(); // re-enable algorithm

    // Disable stillness mode
    EM7180_set_integer_param (0x49, 0x00);

    // Write desired sensor full scale ranges to the EM7180
    em7180.setMagAccFs (0x3E8, 0x08); // 1000 uT, 8 g
    em7180.setGyroFs(0x7D0); // 2000 dps

    // Read sensor new FS values from parameter space
    readParams(0x4A, param);// Request to read  parameter 74
    EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
    EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
    Serial.print("Magnetometer New Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
    Serial.print("Accelerometer New Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
    readParams(0x4B, param);// Request to read  parameter 75
    EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
    Serial.print("Gyroscope New Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
    em7180.requestParamRead(0x00);//End parameter transfer
    em7180.algorithmControlReset(); // re-enable algorithm

    // Read EM7180 status
    if (em7180.getRunStatus() & 0x01) Serial.println(" EM7180 run status = normal mode");
    uint8_t algoStatus = em7180.getAlgorithmStatus();
    if (algoStatus & 0x01) Serial.println(" EM7180 standby status");
    if (algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
    if (algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
    if (algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
    if (algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
    if (algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");
    if (em7180.getPassThruStatus() & 0x01) Serial.print(" EM7180 in passthru mode!");
    uint8_t eventStatus = em7180.getEventStatus();
    if (eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
    if (eventStatus & 0x02) Serial.println(" EM7180 Error");

    // Give some time to read the screen
    delay(1000);

    // Check sensor status
    uint8_t sensorStatus = em7180.getSensorStatus();
    if (sensorStatus & 0x01) sensorError("Magnetometer not acknowledging!");
    if (sensorStatus & 0x02) sensorError("Accelerometer not acknowledging!");
    if (sensorStatus & 0x04) sensorError("Gyro not acknowledging!");
    if (sensorStatus & 0x10) sensorError("Magnetometer ID not recognized!");
    if (sensorStatus & 0x20) sensorError("Accelerometer ID not recognized!");
    if (sensorStatus & 0x40) sensorError("Gyro ID not recognized!");

    // Give some time to read the screen
    delay(1000);
}

void loop()
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
        EM7180_get_WS_params();

        // Put the Sentral in pass-thru mode
        em7180.setPassThroughMode();

        // Store WarmStart data to the M24512DFM I2C EEPROM
        writeSenParams();

        // Take Sentral out of pass-thru mode and re-start algorithm
        em7180.setMasterMode();
        warm_start_saved = true;
    }

    if (serial_input == '2') {
        calibratingA = 512;
    }

    // Check event status register, way to check data ready by polling rather than interrupt
    uint8_t eventStatus = em7180.getEventStatus(); // reading clears the register

    // Check for errors
    // Error detected, what is it?
    if (eventStatus & 0x02) { 

        uint8_t errorStatus = em7180.getErrorStatus();
        if (!errorStatus)
        {
            Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
            if (errorStatus == 0x11) Serial.print("Magnetometer failure!");
            if (errorStatus == 0x12) Serial.print("Accelerometer failure!");
            if (errorStatus == 0x14) Serial.print("Gyro failure!");
            if (errorStatus == 0x21) Serial.print("Magnetometer initialization failure!");
            if (errorStatus == 0x22) Serial.print("Accelerometer initialization failure!");
            if (errorStatus == 0x24) Serial.print("Gyro initialization failure!");
            if (errorStatus == 0x30) Serial.print("Math error!");
            if (errorStatus == 0x80) Serial.print("Invalid sample rate!");
        }
        // Handle errors ToDo
    }

    // if no errors, see if new data is ready
    // new acceleration data available
    if (eventStatus & 0x10) { 

        int16_t accelCount[3];

        em7180.readAccelerometer(accelCount[0], accelCount[1], accelCount[2]);

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*0.000488;  // get actual g value
        ay = (float)accelCount[1]*0.000488;    
        az = (float)accelCount[2]*0.000488;

        // Manages accelerometer calibration; is active when calibratingA > 0
        Accel_cal_check(accelCount);
    }

    if (eventStatus & 0x04) {
        em7180.readQuaternion(Quat[0], Quat[1], Quat[2], Quat[3]);
    }

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;

    // update LCD once per half-second independent of read rate
    if (delt_t > 500) { 

        float Yaw  = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);   
        Yaw   *= 180.0f / PI; 
        Yaw   += MAGNETIC_DECLINATION;
        if (Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360

        Serial.print("ax = "); Serial.print((int)1000*ax);  
        Serial.print(" ay = "); Serial.print((int)1000*ay); 
        Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
        Serial.print("Hardware yaw: ");
        Serial.println(Yaw, 2);

        if (warm_start_saved) {
            Serial.println("Warm Start configuration saved!");
        } 
        else {
            Serial.println("Send '1' to store Warm Start configuration");
        }
        if (accel_cal_saved > 0) {
            Serial.print("Accel Cals Complete:"); Serial.println(accel_cal_saved);
        } 
        else {
            Serial.println("Send '2' to store Accel Cal");
        }

        count = millis(); 
    }
}


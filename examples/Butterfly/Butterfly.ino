#include "LSM6DSM.h"
#include "LIS2MDL.h"
#include "LPS22HB.h"
#include "USFS.h"

#include <RTC.h>

#define myLed 13

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
uint32_t delt_t = 0;                      // used to control display output rate
uint32_t sumCount = 0;                    // used to control display output rate
float pitch, yaw, roll, Yaw, Pitch, Roll;
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float A12, A22, A31, A32, A33;            // rotation matrix coefficients for Hardware Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float lin_Ax, lin_Ay, lin_Az;             // Hardware linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // hardware quaternion data register
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


//LSM6DSM definitions
#define LSM6DSM_intPin1 10  // interrupt1 pin definitions, significant motion
#define LSM6DSM_intPin2 9   // interrupt2 pin definitions, data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
 AFS_2G, AFS_4G, AFS_8G, AFS_16G  
 GFS_245DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
 AODR_12_5Hz, AODR_26Hz, AODR_52Hz, AODR_104Hz, AODR_208Hz, AODR_416Hz, AODR_833Hz, AODR_1660Hz, AODR_3330Hz, AODR_6660Hz
 GODR_12_5Hz, GODR_26Hz, GODR_52Hz, GODR_104Hz, GODR_208Hz, GODR_416Hz, GODR_833Hz, GODR_1660Hz, GODR_3330Hz, GODR_6660Hz
 */ 
uint8_t Ascale = AFS_2G, Gscale = GFS_245DPS, AODR = AODR_208Hz, GODR = GODR_416Hz;

float aRes, gRes;              // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {-0.00499, 0.01540, 0.02902}, gyroBias[3] = {-0.50, 0.14, 0.28}; // offset biases for the accel and gyro
int16_t LSM6DSMData[7];        // Stores the 16-bit signed sensor output
float   Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;  // variables to hold latest accel/gyro data values 

bool newLSM6DSMData = false;
bool newLSM6DSMTap  = false;

LSM6DSM LSM6DSM(LSM6DSM_intPin1, LSM6DSM_intPin2); // instantiate LSM6DSM class


//LIS2MDL definitions
#define LIS2MDL_intPin  8 // interrupt for magnetometer data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
 */ 
uint8_t MODR = MODR_100Hz;

float mRes = 0.0015f;            // mag sensitivity
float magBias[3] = {0,0,0}, magScale[3]  = {0,0,0}; // Bias corrections for magnetometer
int16_t LIS2MDLData[4];          // Stores the 16-bit signed sensor output
float Mtemperature;              // Stores the real internal chip temperature in degrees Celsius
float mx, my, mz;                // variables to hold latest mag data values 
uint8_t LIS2MDLstatus;

bool newLIS2MDLData = false;

LIS2MDL LIS2MDL(LIS2MDL_intPin); // instantiate LIS2MDL class


// LPS22H definitions
uint8_t LPS22H_intPin = 5;

/* Specify sensor parameters (sample rate is twice the bandwidth) 
   Choices are P_1Hz, P_10Hz P_25 Hz, P_50Hz, and P_75Hz
 */
uint8_t PODR = P_25Hz;     // set pressure amd temperature output data rate
uint8_t LPS22Hstatus;
float temperature, pressure, altitude;

bool newLPS22HData = false;

LPS22H LPS22H(LPS22H_intPin);


// RTC set time using STM32L4 natve RTC class
/* Change these values to set the current initial time */
uint8_t seconds = 0;
uint8_t minutes = 33;
uint8_t hours = 12;

/* Change these values to set the current initial date */
uint8_t day = 2;
uint8_t month = 12;
uint8_t year = 17;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = false; // for RTC alarm interrupt


const uint8_t USFS_intPin = 31;
bool newEM7180Data = false;
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
int16_t tempCount, rawPressure, rawTemperature;            // temperature raw count output
float   Temperature, Pressure, Altitude; //  temperature in degrees Celsius, pressure in mbar
float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz; // variables to hold latest sensor data values



/* Choose EM7180, MPU9250 and MS5637 sample rates and bandwidths
   Choices are:
   accBW, gyroBW 0x00 = 250 Hz, 0x01 = 184 Hz, 0x02 = 92 Hz, 0x03 = 41 Hz, 0x04 = 20 Hz, 0x05 = 10 Hz, 0x06 = 5 Hz, 0x07 = no filter (3600 Hz)
   QRtDiv 0x00, 0x01, 0x02, etc quat rate = gyroRt/(1 + QRtDiv)
   magRt 8 Hz = 0x08 or 100 Hz 0x64
   accRt, gyroRt 1000, 500, 250, 200, 125, 100, 50 Hz enter by choosing desired rate
   and dividing by 10, so 200 Hz would be 200/10 = 20 = 0x14
   sample rate of barometer is baroRt/2 so for 25 Hz enter 50 = 0x32
 */
uint8_t accBW = 0x03, gyroBW = 0x03, QRtDiv = 0x01, magRt = 0x64, accRt = 0x14, gyroRt = 0x14, baroRt = 0x32;
/*
   Choose MPU9250 sensor full ranges
   Choices are 2, 4, 8, 16 g for accFS, 250, 500, 1000, and 2000 dps for gyro FS and 1000 uT for magFS expressed as HEX values
 */
uint16_t accFS = 0x08, gyroFS = 0x7D0, magFS = 0x3E8;

USFS USFS(USFS_intPin, false);


void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    delay(4000);

    // Configure led
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH); // start with led off

    Wire.begin(TWI_PINS_20_21); // set master mode 
    Wire.setClock(400000); // I2C frequency at 400 kHz  
    delay(1000);

    LSM6DSM.I2Cscan(); // which I2C device are on the bus?

    // Initialize the USFS
    USFS.getChipID();        // check ROM/RAM version of EM7180
    USFS.loadfwfromEEPROM(); // load EM7180 firmware from EEPROM
    USFS.initEM7180(accBW, gyroBW, accFS, gyroFS, magFS, QRtDiv, magRt, accRt, gyroRt, baroRt); // set MPU and MS5637 sensor parameters

    // Set the time
    SetDefaultRTC();

    /* Set up the RTC alarm interrupt */
    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

    RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

    attachInterrupt(USFS_intPin, EM7180intHandler, RISING);  // define interrupt for INT pin output of EM7180

    USFS.checkEM7180Status();
}

/* End of setup */

void loop() {

    static uint32_t _interruptCount;

    // If intPin goes high, either all data registers have new data
    if(newLIS2MDLData == true) {   // On interrupt, read data
        newLIS2MDLData = false;     // reset newData flag

        LIS2MDLstatus = LIS2MDL.status();

        if(LIS2MDLstatus & 0x08) // if all axes have new data ready
        {
            LIS2MDL.readData(LIS2MDLData);  

            // Now we'll calculate the accleration value into actual G's
            mx = (float)LIS2MDLData[0]*mRes - magBias[0];  // get actual G value 
            my = (float)LIS2MDLData[1]*mRes - magBias[1];   
            mz = (float)LIS2MDLData[2]*mRes - magBias[2]; 
            mx *= magScale[0];
            my *= magScale[1];
            mz *= magScale[2];  
        }
    }

    /*EM7180*/
    // If intpin goes high, all data registers have new data
    if (newEM7180Data == true) { // On interrupt, read data
        newEM7180Data = false;  // reset newData flag
        _interruptCount++;

        // Check event status register, way to chech data ready by polling rather than interrupt
        uint8_t eventStatus = USFS.checkEM7180Status(); // reading clears the register

        // Check for errors
        if (eventStatus & 0x02) { // error detected, what is it?

            uint8_t errorStatus = USFS.checkEM7180Errors();
            if (errorStatus != 0x00) { // is there an error?
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
        if (eventStatus & 0x10) { // new acceleration data available
            USFS.readSENtralAccelData(accelCount);

            // Now we'll calculate the accleration value into actual g's
            Ax = (float)accelCount[0] * 0.000488f; // get actual g value
            Ay = (float)accelCount[1] * 0.000488f;
            Az = (float)accelCount[2] * 0.000488f;
        }

        if (eventStatus & 0x20) { // new gyro data available
            USFS.readSENtralGyroData(gyroCount);

            // Now we'll calculate the gyro value into actual dps's
            Gx = (float)gyroCount[0] * 0.153f; // get actual dps value
            Gy = (float)gyroCount[1] * 0.153f;
            Gz = (float)gyroCount[2] * 0.153f;
        }

        if (eventStatus & 0x08) { // new mag data available
            USFS.readSENtralMagData(magCount);

            // Now we'll calculate the mag value into actual G's
            Mx = (float)magCount[0] * 0.305176f; // get actual G value
            My = (float)magCount[1] * 0.305176f;
            Mz = (float)magCount[2] * 0.305176f;
        }

        if (eventStatus & 0x04) { // new quaternion data available
            USFS.readSENtralQuatData(Q);
        }

        // get MS5637 pressure
        if (eventStatus & 0x40) { // new baro data available
            rawPressure = USFS.readSENtralBaroData();
            Pressure = (float)rawPressure * 0.01f + 1013.25f; // pressure in mBar

            // get MS5637 temperature
            rawTemperature = USFS.readSENtralTempData();
            Temperature = (float) rawTemperature * 0.01f; // temperature in degrees C
        }
    } 

    // end sensor interrupt handling

    /*RTC*/
    if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved and the MPU9250 is awake
        alarmFlag = false;

        // Read RTC
        Serial.print("Interrupts: ");
        Serial.println(_interruptCount);
        Serial.println("RTC:");
        Day = RTC.getDay();
        Month = RTC.getMonth();
        Year = RTC.getYear();
        Seconds = RTC.getSeconds();
        Minutes = RTC.getMinutes();
        Hours   = RTC.getHours();     
        if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
        Serial.print(":"); 
        if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
        Serial.print(":"); 
        if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

        Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
        Serial.println(" ");

        Serial.print("Ax = "); Serial.print((int)1000 * Ax);
        Serial.print(" Ay = "); Serial.print((int)1000 * Ay);
        Serial.print(" Az = "); Serial.print((int)1000 * Az); Serial.println(" mg");
        Serial.print("Gx = "); Serial.print( Gx, 2);
        Serial.print(" Gy = "); Serial.print( Gy, 2);
        Serial.print(" Gz = "); Serial.print( Gz, 2); Serial.println(" deg/s");
        Serial.print("Mx = "); Serial.print( (int)Mx);
        Serial.print(" My = "); Serial.print( (int)My);
        Serial.print(" Mz = "); Serial.print( (int)Mz); Serial.println(" mG");

        Serial.println("Hardware quaternions:");
        Serial.print("Q0 = "); Serial.print(Q[0]);
        Serial.print(" Qx = "); Serial.print(Q[1]);
        Serial.print(" Qy = "); Serial.print(Q[2]);
        Serial.print(" Qz = "); Serial.println(Q[3]);

        //Hardware AHRS:
        A12 =   2.0f * (Q[1] * Q[2] + Q[0] * Q[3]);
        A22 =   Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
        A31 =   2.0f * (Q[0] * Q[1] + Q[2] * Q[3]);
        A32 =   2.0f * (Q[1] * Q[3] - Q[0] * Q[2]);
        A33 =   Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];
        Pitch = -asinf(A32);
        Roll  = atan2f(A31, A33);
        Yaw   = atan2f(A12, A22);
        Pitch *= 180.0f / pi;
        Yaw   *= 180.0f / pi;
        Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        if (Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
        Roll  *= 180.0f / pi;
        lin_Ax = ax + a31;
        lin_Ay = ay + a32;
        lin_Az = az - a33;

        Serial.print("Hardware Yaw, pitch, Roll: ");
        Serial.print(Yaw, 2);
        Serial.print(", ");
        Serial.print(Pitch, 2);
        Serial.print(", ");
        Serial.println(Roll, 2);

        Serial.print("Hardware Grav_x, Grav_y, Grav_z: ");
        Serial.print(-A31 * 1000, 2);
        Serial.print(", ");
        Serial.print(-A32 * 1000, 2);
        Serial.print(", ");
        Serial.print(A33 * 1000, 2);  Serial.println(" mg");
        Serial.print("Hardware Lin_ax, Lin_ay, Lin_az: ");
        Serial.print(lin_Ax * 1000, 2);
        Serial.print(", ");
        Serial.print(lin_Ay * 1000, 2);
        Serial.print(", ");
        Serial.print(lin_Az * 1000, 2);  Serial.println(" mg");

        Serial.println("MS5637:");
        Serial.print("Altimeter temperature = ");
        Serial.print(Temperature, 2);
        Serial.println(" C"); // temperature in degrees Celsius
        Serial.print("Altimeter temperature = ");
        Serial.print(9.0f * Temperature / 5.0f + 32.0f, 2);
        Serial.println(" F"); // temperature in degrees Fahrenheit
        Serial.print("Altimeter pressure = ");
        Serial.print(Pressure, 2);
        Serial.println(" mbar");// pressure in millibar
        Altitude = 145366.45f * (1.0f - pow(((Pressure) / 1013.25f), 0.190284f));
        Serial.print("Altitude = ");
        Serial.print(Altitude, 2);
        Serial.println(" feet");
        Serial.println(" ");


    } // end of RTC alarm handling

    digitalWrite(myLed, LOW); delay(10); digitalWrite(myLed, HIGH);  // flash led for 10 milliseconds
    STM32.sleep();

}  //end of loop

/*  End of main loop */


void myinthandler1()
{
    newLSM6DSMData = true;
}

void myinthandler2()
{
    newLIS2MDLData = true;
}

void myinthandler3()
{
    newLPS22HData = true;
}

void EM7180intHandler()
{
    newEM7180Data = true;
}

void alarmMatch()
{
    alarmFlag = true;
}

void SetDefaultRTC()  // Sets the RTC to the FW build date-time...
{
    char Build_mo[3];

    // Convert month string to integer

    Build_mo[0] = build_date[0];
    Build_mo[1] = build_date[1];
    Build_mo[2] = build_date[2];

    String build_mo = Build_mo;

    if(build_mo == "Jan")
    {
        month = 1;
    } else if(build_mo == "Feb")
    {
        month = 2;
    } else if(build_mo == "Mar")
    {
        month = 3;
    } else if(build_mo == "Apr")
    {
        month = 4;
    } else if(build_mo == "May")
    {
        month = 5;
    } else if(build_mo == "Jun")
    {
        month = 6;
    } else if(build_mo == "Jul")
    {
        month = 7;
    } else if(build_mo == "Aug")
    {
        month = 8;
    } else if(build_mo == "Sep")
    {
        month = 9;
    } else if(build_mo == "Oct")
    {
        month = 10;
    } else if(build_mo == "Nov")
    {
        month = 11;
    } else if(build_mo == "Dec")
    {
        month = 12;
    } else
    {
        month = 1;     // Default to January if something goes wrong...
    }

    // Convert ASCII strings to integers
    day     = (build_date[4] - 48)*10 + build_date[5] - 48;  // ASCII "0" = 48
    year    = (build_date[9] - 48)*10 + build_date[10] - 48;
    hours   = (build_time[0] - 48)*10 + build_time[1] - 48;
    minutes = (build_time[3] - 48)*10 + build_time[4] - 48;
    seconds = (build_time[6] - 48)*10 + build_time[7] - 48;

    // Set the date/time

    RTC.setDay(day);
    RTC.setMonth(month);
    RTC.setYear(year);
    RTC.setHours(hours);
    RTC.setMinutes(minutes);
    RTC.setSeconds(seconds);
}

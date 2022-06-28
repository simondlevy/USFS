#include "USFS.h"

#include <RTC.h>

static const uint8_t LED_PIN       = 13;
static const uint8_t INTERRUPT_PIN = 31;

static const char  *build_date = __DATE__;   // 11 characters MMM DD YYYY
static const char  *build_time = __TIME__;   // 8 characters HH:MM:SS

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
static float pi = 3.141592653589793238462643383279502884f;
static float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
static float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
static float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
static float Yaw, Pitch, Roll;
static float a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
static float A12, A22, A31, A32, A33;            // rotation matrix coefficients for Hardware Euler angles and gravity components
static float deltat;
static float lin_Ax, lin_Ay, lin_Az;             // Hardware linear acceleration (acceleration with gravity component subtracted)
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
static float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // hardware quaternion data register
static float ax, ay, az;

// RTC set time using STM32L4 natve RTC class
/* Change these values to set the current initial time */
static uint8_t seconds = 0;
static uint8_t minutes = 33;
static uint8_t hours = 12;

/* Change these values to set the current initial date */
static uint8_t day = 2;
static uint8_t month = 12;
static uint8_t year = 17;

static uint8_t Seconds, Minutes, Hours, Day, Month, Year;

static bool alarmFlag = false; // for RTC alarm interrupt


static bool newEM7180Data = false;
static int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
static int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
static int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
static int16_t rawPressure, rawTemperature;            // temperature raw count output
static float   Temperature, Pressure, Altitude; //  temperature in degrees Celsius, pressure in mbar
static float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz; // variables to hold latest sensor data values


static uint8_t accBW = 0x03, gyroBW = 0x03, QRtDiv = 0x01, magRt = 0x64, accRt = 0x14, gyroRt = 0x14, baroRt = 0x32;

static uint16_t accFS = 0x08, gyroFS = 0x7D0, magFS = 0x3E8;

static USFS USFS(INTERRUPT_PIN, false);

static void EM7180intHandler()
{
    newEM7180Data = true;
}

static void alarmMatch()
{
    alarmFlag = true;
}

static void SetDefaultRTC()  // Sets the RTC to the FW build date-time...
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

void setup()
{

    // put your setup code here, to run once:
    Serial.begin(115200);
    delay(4000);

    // Configure led
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // start with led off

    Wire.begin(TWI_PINS_20_21); // set master mode 
    Wire.setClock(400000); // I2C frequency at 400 kHz  
    delay(1000);


    // Initialize the USFS
    USFS.getChipID();        // check ROM/RAM version of EM7180
    USFS.loadfwfromEEPROM(); // load EM7180 firmware from EEPROM
    USFS.initEM7180(accBW, gyroBW, accFS, gyroFS, magFS, QRtDiv, magRt, accRt, gyroRt, baroRt); // set MPU and MS5637 sensor parameters

    // Set the time
    SetDefaultRTC();

    /* Set up the RTC alarm interrupt */
    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

    RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

    attachInterrupt(INTERRUPT_PIN, EM7180intHandler, RISING);  // define interrupt for INT pin output of EM7180

    USFS.checkEM7180Status();
}

/* End of setup */

void loop() {

    static uint32_t _interruptCount;

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

    digitalWrite(LED_PIN, LOW); delay(10); digitalWrite(LED_PIN, HIGH);  // flash led for 10 milliseconds
    STM32.sleep();

}  //end of loop


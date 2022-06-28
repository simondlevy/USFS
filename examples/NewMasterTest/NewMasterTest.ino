#include <Wire.h>   

#include "USFS.h"
#include "madgwick.h"

// Specify sensor full scale
static const uint8_t Gscale = GFS_250DPS;
static const uint8_t Ascale = AFS_2G;
static const uint8_t Mscale = MFS_16BITS;
static const uint8_t Mmode = MMODE_8HZ;

// Pin definitions
static const uint8_t INTERRUPT_PIN = 12;  
static const uint8_t LED_PIN = 18;  

static volatile bool _gotInterrupt;

static void interruptHandler(void)
{
    _gotInterrupt = true;
}

static void reportEulerAngles(float q[4], const char * label)
{
    // Define output variables from updated quaternion---these are
    // Tait-Bryan angles, commonly used in aircraft orientation.  In this
    // coordinate system, the positive z-axis is down toward Earth.  Yaw is
    // the angle between Sensor x-axis and Earth magnetic North (or true
    // North if corrected for local declination, looking down on the sensor
    // positive yaw is counterclockwise.  Pitch is angle between sensor
    // x-axis and Earth ground plane, toward the Earth is positive, up
    // toward the sky is negative.  Roll is angle between sensor y-axis and
    // Earth ground plane, y-axis up is positive roll.  These arise from
    // the definition of the homogeneous rotation matrix constructed from
    // quaternions.  Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the
    // rotations must be applied in the correct order which for this
    // configuration is yaw, pitch, and then roll.  For more see
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.

    Serial.print(label);
    Serial.println(" quaternions:"); 
    Serial.print("qw = ");
    Serial.print(q[0]);
    Serial.print(" qx = ");
    Serial.print(q[1]); 
    Serial.print(" qy = ");
    Serial.print(q[2]); 
    Serial.print(" qz = ");
    Serial.println(q[3]); 

    float yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] *
            q[1] - q[2] * q[2] - q[3] * q[3]);   

    float pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));

    float roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1]
            * q[1] - q[2] * q[2] + q[3] * q[3]);

    pitch *= 180 / PI;
    yaw   *= 180 / PI; 

    // Declination at Danville, California is 13 degrees 48 minutes and 47
    // seconds on 2014-04-04
    yaw += 13.8f; 

    yaw += (yaw < 0) ? 360 : 0; // Ensure yaw stays between 0 and 360

    roll  *= 180.0f / PI;

    Serial.print(label);
    Serial.print(" yaw, pitch, roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
}

void setup()
{
    Wire.begin();
    Wire.setClock(400000);
    delay(5000);
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);

    delay(1000);

    // Read SENtral device information
    Serial.print("EM7180 ROM Version: 0x");
    Serial.print(usfsReadRom1(), HEX);
    Serial.println(usfsReadRom2(), HEX);
    Serial.println("Should be: 0xE609");
    Serial.print("EM7180 RAM Version: 0x");
    Serial.print(usfsReadRam1());
    Serial.println(usfsReadRam2());
    Serial.print("EM7180 ProductID: 0x");
    Serial.print(usfsReadPid(), HEX);
    Serial.println(" Should be: 0x80");
    Serial.print("EM7180 RevisionID: 0x");
    Serial.print(usfsReadRid(), HEX);
    Serial.println(" Should be: 0x02");

    delay(1000); // give some time to read the screen

    // Check which sensors can be detected by the EM7180
    if (usfsHasBarometer()) {
        Serial.println("A barometer is installed");
    }

    if (usfsHasHumiditySensor()) {
        Serial.println("A humidity sensor is installed");
    }

    if (usfsHasTemperatureSensor()) {
        Serial.println("A temperature sensor is installed");
    }

    if (usfsHasCustomSensor()) {
        Serial.println("A custom sensor is installed");
    }

    if (usfsHasSecondCustomSensor()) {
        Serial.println("A second custom sensor is installed");
    }

    if (usfsHasThirdCustomSensor()) {
        Serial.println("A third custom sensor is installed");
    }

    delay(1000); // give some time to read the screen

    // Make several attempts to check status, each followed by a reset
    for (uint8_t k=0; k<10; ++k) {

        bool eepromDetected = usfsEepromDetected();

        if (eepromDetected) {
            Serial.println("EEPROM detected on the sensor bus!");
        }

        if (usfsEepromUploaded()) {
            Serial.println("EEPROM uploaded config file!");
        }

        if (usfsEepromIncorrect()) {
            Serial.println("EEPROM CRC incorrect!");
        }

        if (usfsEepromInitialized()) {
            Serial.println("EM7180 in initialized state!");
        }

        if (usfsEepromNotDetected()) {
            Serial.println("No EEPROM detected!");
        }

        if (eepromDetected) {
            break;
        }

        usfsReset();

        delay(500);  
    }

    if (usfsEepromUploadSuccessful()) {
        Serial.println("EEPROM upload successful!");
    }

    delay(1000); // give some time to read the screen

    static const uint8_t QUAT_RATE_DIVISOR = 2;
    static const uint8_t MAG_RATE          = 100;
    static const uint8_t ACCEL_RATE        = 20; // 200
    static const uint8_t GYRO_RATE         = 20; // 200
    static const uint8_t BARO_RATE         = 50; // 200
    static const bool    VERBOSE           = true;

    usfsBegin(
            QUAT_RATE_DIVISOR,
            MAG_RATE,
            ACCEL_RATE,
            GYRO_RATE,
            BARO_RATE, VERBOSE,
            INTERRUPT_RESET | INTERRUPT_ERROR | INTERRUPT_QUAT);

    delay(1000); // give some time to read the screen

    Serial.println("Enter '1' to proceed...");

    while (true) {
        if (Serial.read() == '1') {
            break;
        }
        delay(10);
    }
}

/*
   void loop()
   {  
   static uint32_t _interruptCount;

   if (_gotInterrupt) {
   _gotInterrupt = false;
   _interruptCount++;
   }

   uint8_t eventStatus = usfsGetEventStatus();

// Check for errors
if (usfsEventStatusIsError(eventStatus)) {

usfsReportEventError();

// XXX should handle error
}

static float ax, ay, az;

if (usfsEventIsAccelerometer(eventStatus)) { 

int16_t accelCount[3] = {};

usfsReadAccelerometer(accelCount);

// Now we'll calculate the accleration value into actual g's
ax = (float)accelCount[0]*0.000488;  // get actual g value
ay = (float)accelCount[1]*0.000488;    
az = (float)accelCount[2]*0.000488;  
}

static float gx, gy, gz; 

if (usfsEventIsGyrometer(eventStatus)) { 

int16_t gyroCount[3] = {};

usfsReadGyrometer(gyroCount);

// Now we'll calculate the gyro value into actual dps's
gx = (float)gyroCount[0]*0.153;  // get actual dps value
gy = (float)gyroCount[1]*0.153;    
gz = (float)gyroCount[2]*0.153;  
}

static float mx, my, mz; 

if (usfsEventIsMagnetometer(eventStatus)) {

int16_t magCount[3] = {};

usfsReadMagnetometer(magCount);

// Now we'll calculate the mag value into actual G's
mx = (float)magCount[0]*0.305176;  // get actual G value
my = (float)magCount[1]*0.305176;    
mz = (float)magCount[2]*0.305176;  
}

static float hardwareQuat[4];

if (usfsEventIsQuaternion(eventStatus)) { 
usfsReadQuaternion(hardwareQuat); 
}

static int16_t rawPressure, rawTemperature;    
static float  temperature, pressure; 

if (usfsEventIsBarometer(eventStatus)) {

    rawPressure = usfsReadBarometer();
    pressure = (float)rawPressure*0.01f + 1013.25f; // pressure in mBar

    // get MS5637 temperature
    rawTemperature = usfsReadTemperature();  
    temperature = (float) rawTemperature*0.01;  // temperature in degrees C
}

// keep track of rates
uint32_t now = micros();
static uint32_t lastUpdate;
float deltat = ((now - lastUpdate)/1000000.0f); 
lastUpdate = now;

static float sum;
sum += deltat; 

static uint32_t sumCount;
sumCount++;

// Sensors x (y)-axis of the accelerometer/gyro is aligned with the y
// (x)-axis of the magnetometer; the magnetometer z-axis (+ down) is
// misaligned with z-axis (+ up) of accelerometer and gyro!  We have to
// make some allowance for this orientation mismatch in feeding the output
// to the quaternion filter.  We will assume that +y accel/gyro is North,
// then x accel/gyro is East. So if we want te quaternions properly aligned
// we need to feed into the madgwick function Ay, Ax, -Az, Gy, Gx, -Gz, Mx,
// My, and Mz. But because gravity is by convention positive down, we need
// to invert the accel data, so we pass -Ay, -Ax, Az, Gy, Gx, -Gz, Mx, My,
// and Mz into the Madgwick function to get North along the accel +y-axis,
// East along the accel +x-axis, and Down along the accel -z-axis.  This
// orientation choice can be modified to allow any convenient (non-NED)
// orientation convention.  This is ok by aircraft orientation standards!
// Pass gyro rate as rad/s

static float softwareQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};   

MadgwickQuaternionUpdate(deltat, -ay, -ax, az,
        gy*PI/180.0f, gx*PI/180.0f, -gz*PI/180.0f,  mx,  my, mz, softwareQuat);

static uint32_t msec;

if (millis()-msec > 500) { // update LCD once per half-second independent of read rate

    Serial.print("ax = ");
    Serial.print((int)1000*ax);  
    Serial.print(" ay = ");
    Serial.print((int)1000*ay); 
    Serial.print(" az = ");
    Serial.print((int)1000*az);
    Serial.println(" mg");
    Serial.print("gx = ");
    Serial.print( gx, 2); 
    Serial.print(" gy = ");
    Serial.print( gy, 2); 
    Serial.print(" gz = ");
    Serial.print( gz, 2);
    Serial.println(" deg/s");
    Serial.print("mx = ");
    Serial.print( (int)mx); 
    Serial.print(" my = ");
    Serial.print( (int)my); 
    Serial.print(" mz = ");
    Serial.print( (int)mz);
    Serial.println(" mG");

    reportEulerAngles(softwareQuat, "Software");

    reportEulerAngles(hardwareQuat, "Hardware");

    Serial.println("MS5637:");
    Serial.print("Altimeter temperature = "); 
    Serial.print( temperature, 2); 
    Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Altimeter temperature = "); 
    Serial.print(9.*temperature/5. + 32., 2); 
    Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter pressure = "); 
    Serial.print(pressure, 2);  
    Serial.println(" mbar");// pressure in millibar
    float altitude = 145366.45f*(1.0f - pow(((pressure)/1013.25f), 0.190284f));
    Serial.print("Altitude = "); 
    Serial.print(altitude, 2); 
    Serial.println(" feet");
    Serial.println(" ");

    Serial.print("rate = ");
    Serial.print((float)sumCount/sum, 2);
    Serial.println(" Hz");

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    msec = millis(); 
    sumCount = 0;
    sum = 0;    
}
}
*/

void loop()
{  
    static uint32_t _interruptCount;

    if (_gotInterrupt == true) {  

        _gotInterrupt = false; 

        _interruptCount++;
    }

    Serial.println(_interruptCount);
}

#include <Wire.h>   

#include "USFS.h"
#include "madgwick.h"

// Specify sensor full scale
static const uint8_t Gscale = GFS_250DPS;
static const uint8_t Ascale = AFS_2G;
static const uint8_t Mscale = MFS_16BITS;
static const uint8_t Mmode = MMODE_8HZ;

// Pin definitions
static const uint8_t INT_PIN = 12;  
static const uint8_t LED_PIN = 18;  

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
    delay(5000);
    Serial.begin(115200);

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(INT_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

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

    usfsBegin(QUAT_RATE_DIVISOR, MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE);

    delay(100);

    // EM7180 parameter adjustments
    Serial.println("Beginning Parameter Adjustments");

    // Read sensor default FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); 
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);
    byte param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
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

    Serial.print("Magnetometer Default Full Scale Range: +/-");
    Serial.print(EM7180_mag_fs);
    Serial.println("uT");
    Serial.print("Accelerometer Default Full Scale Range: +/-");
    Serial.print(EM7180_acc_fs);
    Serial.println("g");

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

    Serial.print("Gyroscope Default Full Scale Range: +/-");
    Serial.print(EM7180_gyro_fs);
    Serial.println("dps");

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
    Serial.print("Magnetometer New Full Scale Range: +/-");
    Serial.print(EM7180_mag_fs);
    Serial.println("uT");
    Serial.print("Accelerometer New Full Scale Range: +/-");
    Serial.print(EM7180_acc_fs);
    Serial.println("g");
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
    Serial.print("Gyroscope New Full Scale Range: +/-");
    Serial.print(EM7180_gyro_fs);
    Serial.println("dps");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

    // Read EM7180 status
    uint8_t runStatus = readByte(EM7180_ADDRESS, EM7180_RunStatus);
    if (runStatus & 0x01) Serial.println(" EM7180 run status = normal mode");
    uint8_t algoStatus = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
    if (algoStatus & 0x01) Serial.println(" EM7180 standby status");
    if (algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
    if (algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
    if (algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
    if (algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
    if (algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");
    uint8_t passthruStatus = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
    if (passthruStatus & 0x01) Serial.print(" EM7180 in passthru mode!");
    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus);
    if (eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
    if (eventStatus & 0x02) Serial.println(" EM7180 Error");
    if (eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
    if (eventStatus & 0x08) Serial.println(" EM7180 new mag result");
    if (eventStatus & 0x10) Serial.println(" EM7180 new accel result");
    if (eventStatus & 0x20) Serial.println(" EM7180 new gyro result"); 

    delay(1000); // give some time to read the screen

    // Check sensor status
    uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);
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

    delay(1000); // give some time to read the screen

    Serial.println("Enter '1' to proceed...");

    while (true) {
        if (Serial.read() == '1') {
            break;
        }
        delay(10);
    }
}

void loop()
{  
    static float Quat[4] = {0, 0, 0, 0}; // quaternion data register
    static float ax, ay, az, gx, gy, gz, mx, my, mz; 
    static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};   
    static int16_t rawPressure, rawTemperature;    
    static float  temperature, pressure, altitude; 
    static uint32_t count, sumCount;  // used to control  output rate
    static float sum;          // integration interval
    static uint32_t lastUpdate; // used to calculate integration interval

    // Check event status register, way to chech data ready by polling rather than interrupt
    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); 

    // Check for errors
    if (eventStatus & 0x02) { // error detected, what is it?

        uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
        if (!errorStatus) {
            Serial.print(" EM7180 sensor status = ");
            Serial.println(errorStatus);
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

        int16_t accelCount[3] = {};

        usfsReadAccelerometer(accelCount);

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*0.000488;  // get actual g value
        ay = (float)accelCount[1]*0.000488;    
        az = (float)accelCount[2]*0.000488;  
    }

    if (eventStatus & 0x20) { // new gyro data available

        int16_t gyroCount[3] = {};

        usfsReadGyrometer(gyroCount);

        // Now we'll calculate the gyro value into actual dps's
        gx = (float)gyroCount[0]*0.153;  // get actual dps value
        gy = (float)gyroCount[1]*0.153;    
        gz = (float)gyroCount[2]*0.153;  
    }

    if (eventStatus & 0x08) { // new mag data available

        int16_t magCount[3] = {};

        usfsReadMagnetometer(magCount);

        // Now we'll calculate the mag value into actual G's
        mx = (float)magCount[0]*0.305176;  // get actual G value
        my = (float)magCount[1]*0.305176;    
        mz = (float)magCount[2]*0.305176;  
    }

    if (eventStatus & 0x04) { // new quaternion data available
        usfsReadQuaternion(Quat); 
    }

    // get MS5637 pressure
    if (eventStatus & 0x40) { // new baro data available
        //   Serial.println("new Baro data!");
        rawPressure = usfsReadBarometer();
        pressure = (float)rawPressure*0.01f + 1013.25f; // pressure in mBar

        // get MS5637 temperature
        rawTemperature = usfsReadTemperature();  
        temperature = (float) rawTemperature*0.01;  // temperature in degrees C
    }

    // keep track of rates
    uint32_t now = micros();
    float deltat = ((now - lastUpdate)/1000000.0f); 
    lastUpdate = now;

    sum += deltat; // sum for averaging filter update rate
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
    MadgwickQuaternionUpdate(
            deltat, -ay, -ax, az, gy*PI/180.0f, gx*PI/180.0f, -gz*PI/180.0f,  mx,  my, mz, q);


    if (millis()-count > 500) { // update LCD once per half-second independent of read rate

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

        Serial.println("Software quaternions:"); 
        Serial.print("qw = ");
        Serial.print(q[0]);
        Serial.print(" qx = ");
        Serial.print(q[1]); 
        Serial.print(" qy = ");
        Serial.print(q[2]); 
        Serial.print(" qz = ");
        Serial.println(q[3]); 
        Serial.println("Hardware quaternions:"); 
        Serial.print("Qw = ");
        Serial.print(Quat[0]);
        Serial.print(" Qx = ");
        Serial.print(Quat[1]); 
        Serial.print(" Qy = ");
        Serial.print(Quat[2]); 
        Serial.print(" Qz = ");
        Serial.println(Quat[3]); 

        reportEulerAngles(q, "Software");

        reportEulerAngles(Quat, "Hardware");

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
        altitude = 145366.45f*(1.0f - pow(((pressure)/1013.25f), 0.190284f));
        Serial.print("Altitude = "); 
        Serial.print(altitude, 2); 
        Serial.println(" feet");
        Serial.println(" ");

        Serial.print("rate = ");
        Serial.print((float)sumCount/sum, 2);
        Serial.println(" Hz");

        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        count = millis(); 
        sumCount = 0;
        sum = 0;    
    }
}

#include "USFS.h"

static const uint8_t LED_PIN       = 18; 
static const uint8_t INTERRUPT_PIN = 12; 


static float pi = 3.141592653589793238462643383279502884f;
static float GyroMeasError = pi * (40.0f / 180.0f);   
static float GyroMeasDrift = pi * (0.0f  / 180.0f);   
static float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   
static float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   
static float Yaw, Pitch, Roll;
static float a31, a32, a33;            
static float A12, A22, A31, A32, A33;            
static float deltat;
static float lin_Ax, lin_Ay, lin_Az;             
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    
static float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    
static float ax, ay, az;

static bool newEM7180Data = false;
static int16_t accelCount[3];  
static int16_t gyroCount[3];   
static int16_t magCount[3];    
static int16_t rawPressure, rawTemperature;            
static float   Temperature, Pressure, Altitude; 
static float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz; 


static uint8_t accBW = 0x03, gyroBW = 0x03, QRtDiv = 0x01, magRt = 0x64, accRt = 0x14, gyroRt = 0x14, baroRt = 0x32;

static uint16_t accFS = 0x08, gyroFS = 0x7D0, magFS = 0x3E8;

static USFS USFS(INTERRUPT_PIN, false);

static void EM7180intHandler()
{
    newEM7180Data = true;
}

void setup()
{

    Serial.begin(115200);
    delay(4000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); 

    Wire.begin(TWI_PINS_20_21); 
    Wire.setClock(400000); 
    delay(1000);
    
    USFS.getChipID();        
    USFS.loadfwfromEEPROM(); 
    USFS.initEM7180(accBW, gyroBW, accFS, gyroFS, magFS, QRtDiv, magRt, accRt, gyroRt, baroRt); 

    attachInterrupt(INTERRUPT_PIN, EM7180intHandler, RISING);  

    USFS.checkEM7180Status();
}

/* End of setup */

void loop() {

    static uint32_t _interruptCount;

    
    if (newEM7180Data == true) { 
        newEM7180Data = false;  
        _interruptCount++;

        
        uint8_t eventStatus = USFS.checkEM7180Status(); 

        
        if (eventStatus & 0x02) { 

            uint8_t errorStatus = USFS.checkEM7180Errors();
            if (errorStatus != 0x00) { 
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

            

        }

        
        if (eventStatus & 0x10) { 
            USFS.readSENtralAccelData(accelCount);

            
            Ax = (float)accelCount[0] * 0.000488f; 
            Ay = (float)accelCount[1] * 0.000488f;
            Az = (float)accelCount[2] * 0.000488f;
        }

        if (eventStatus & 0x20) { 
            USFS.readSENtralGyroData(gyroCount);

            
            Gx = (float)gyroCount[0] * 0.153f; 
            Gy = (float)gyroCount[1] * 0.153f;
            Gz = (float)gyroCount[2] * 0.153f;
        }

        if (eventStatus & 0x08) { 
            USFS.readSENtralMagData(magCount);

            
            Mx = (float)magCount[0] * 0.305176f; 
            My = (float)magCount[1] * 0.305176f;
            Mz = (float)magCount[2] * 0.305176f;
        }

        if (eventStatus & 0x04) { 
            USFS.readSENtralQuatData(Q);
        }

        
        if (eventStatus & 0x40) { 
            rawPressure = USFS.readSENtralBaroData();
            Pressure = (float)rawPressure * 0.01f + 1013.25f; 

            
            rawTemperature = USFS.readSENtralTempData();
            Temperature = (float) rawTemperature * 0.01f; 
        }
    } 

    static uint32_t _msec;

    uint32_t msec = millis();

    if (msec-_msec > 500) { 

        Serial.print("Interrupts: ");
        Serial.println(_interruptCount);

        _msec = msec;

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
        Yaw   += 13.8f; 
        if (Yaw < 0) Yaw   += 360.0f ; 
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
        Serial.println(" C"); 
        Serial.print("Altimeter temperature = ");
        Serial.print(9.0f * Temperature / 5.0f + 32.0f, 2);
        Serial.println(" F"); 
        Serial.print("Altimeter pressure = ");
        Serial.print(Pressure, 2);
        Serial.println(" mbar");
        Altitude = 145366.45f * (1.0f - pow(((Pressure) / 1013.25f), 0.190284f));
        Serial.print("Altitude = ");
        Serial.print(Altitude, 2);
        Serial.println(" feet");
        Serial.println(" ");


    } 

    digitalWrite(LED_PIN, LOW); delay(10); digitalWrite(LED_PIN, HIGH);  
    STM32.sleep();

}  


// ----------- //
/* Import Libs */
// ----------- //

#include <Adafruit_NeoPixel.h> // NeoPixel v1.10.6
#include <STM32SD.h> // MicroSD card v1.2.3
#include <Adafruit_LSM9DS1.h> // IMU v2.1.1
#include <Adafruit_Sensor.h> // Required for the IMU v1.1.6
#include <Adafruit_BMP3XX.h> // BARO v2.1.2
#include <Adafruit_PWMServoDriver.h> // PWM v2.4.1

// --------- //
/* Variables */
// --------- //

/** DO NOT, FOR THE LOVE OF ALL THAT IS HOLY, ATTEMPT TO MOVE THIS INTO ANY STRUCT OR CLASS!
 * This object has been a massive pain in the butt and all attempts at treating it like a normal variable have failed.
 * Only continue if you wish to loose your sanity.
 *
 * Regardless of what the function docs say, give colors in GRB
 */
Adafruit_NeoPixel stateMachineLED(1, 8, NEO_RGB);

Adafruit_PWMServoDriver PWM = Adafruit_PWMServoDriver();

// ------- //
/* Structs */
// ------- //

typedef struct {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
} XYZ;

typedef struct {
    float x = 0.0;
    float y = 0.0;
} XY;

// ---------- //
/* Subsystems */
// ---------- //

// **REMINDER**: When you add/remove variables from subsystems, remember to modify the "variables" and "post-flight" command.

/** General variables and methods for the rocket.
 * Needed because global variables and functions aren't accessible in other subsystems.
 */
class MetaSubsystem {
    public:
        char buffer[256]; // A standard char[] buffer
        int floatBuffer[2]; // A buffer for use with splitFloat()

        float targetApogee = 83.82; // Target apogee (in m)

        /** Splits a float into two integers (The whole number, and the decimal with 3 digits of precision)
         * @param buffer A buffer array capable of holding at least 2 ints.
         * @param in
         */
        void splitFloat(int buffer[], float in) {
            buffer[0] = in;
            buffer[1] = abs(trunc((in - buffer[0]) * 1000));
        }

        /** Gets a variable's type and stores it in buffer
         * @param in
         */
        template <typename T>
        void getVariableType(T in) {
            String inType = __PRETTY_FUNCTION__;
            int start = inType.indexOf("[with T = ") + 10;
            int stop = inType.lastIndexOf(']');
            inType.substring(start, stop).toCharArray(buffer, sizeof(buffer));
        }

        /** Compares two char arrays
         * @param firstArray
         * @param secondArray
         * @returns bool
         */
        bool compareCharArrays(char firstArray[], char secondArray[]) {
            char terminatingChar = '\0';

            int i = -1;
            do {
                i++;

                if (firstArray[i] != secondArray[i]) {
                    return false;
                }
            } while ((firstArray[i] != terminatingChar) && (secondArray[i] != terminatingChar));

            return true;
        }

        /** Converts a given object and stores it in buffer
         * @param in The object
         */
        void toCharArray(String in) {
            in.toCharArray(buffer, sizeof(buffer));
        }
        void toCharArray(char* in) {
            sprintf(buffer, in);
        }
        void toCharArray(const char* in) {
            sprintf(buffer, in);
        }
        void toCharArray(int in) {
            sprintf(buffer, "%i", in);
        }
        void toCharArray(float in) {
            splitFloat(floatBuffer, in);
            sprintf(buffer, "%i.%i", floatBuffer[0], floatBuffer[1]);
        }
        void toCharArray(bool in) {
            int convertedIn = in;
            sprintf(buffer, (in ? "true" : "false"));
        }
        void toCharArray(XYZ in) {
            int xFloatBuffer[2];
            int yFloatBuffer[2];
            int zFloatBuffer[2];

            splitFloat(xFloatBuffer, in.x);
            splitFloat(yFloatBuffer, in.y);
            splitFloat(zFloatBuffer, in.z);

            sprintf(buffer, "%i.%i, %i.%i, %i.%i", xFloatBuffer[0], xFloatBuffer[1], yFloatBuffer[0], yFloatBuffer[1], zFloatBuffer[0], zFloatBuffer[1]);
        }
        void toCharArray(XY in) {
            int xFloatBuffer[2];
            int yFloatBuffer[2];

            splitFloat(xFloatBuffer, in.x);
            splitFloat(yFloatBuffer, in.y);

            sprintf(buffer, "%i.%i, %i.%i", xFloatBuffer[0], xFloatBuffer[1], yFloatBuffer[0], yFloatBuffer[1]);
        }
};
MetaSubsystem meta;

class DebuggingSubsystem {
    private:
        MetaSubsystem* metaSub;
    public:
        String command = ""; // The debugging command passed in via Serial
        char commandArr[128] = ""; // debugging.command as a char[]
        char commandFirstWord[32] = ""; // All characters up to the first space (exclusive) of debugging.commandArr

        bool watchEnabled = false; // Whether the command "watch" feature is enabled
        char watchCommand[32] = ""; // The command to run every cycle when the "watch" feature is enabled
        const int watchDelay = 100; // The delay in ms to use with commands that require sensor data to be read. Lower values give more data, but aren't human readable alone

        DebuggingSubsystem(MetaSubsystem* metaSub) {
            this->metaSub = metaSub;
        }

        /** Prints an object to Serial
         * @param in
         */
        template <typename T>
        void print(T in) {
            metaSub->toCharArray(in);
            Serial.print(metaSub->buffer);
        }

        /** Prints an object to Serial with a linebreak
         * @param in
         */
        template <typename T>
        void println(T in) {
            metaSub->toCharArray(in);
            Serial.println(metaSub->buffer);
        }

        /** Prints an object to Serial with a label
         * @param in
         */
        template <typename T>
        void print(char label[], T in) {
            Serial.print(label);
            metaSub->toCharArray(in);
            Serial.print(metaSub->buffer);
        }

        /** Prints an object to Serial with a label and a linebreak
         * @param in
         */
        template <typename T>
        void println(char label[], T in) {
            Serial.print(label);
            metaSub->toCharArray(in);
            Serial.println(metaSub->buffer);
        }
};
DebuggingSubsystem debugging(&meta);

class StateMachineSubsystem {
    public:
        // States
        int state = 1; // The current state of the state machine

        const int sBootUp = 1;
        const int sAwaitingArming = 2;
        const int sAwaitingLaunch = 3;
        const int sPoweredAccent = 4;
        const int sUnpoweredAccent = 5;
        const int sParachuteHold = 6;
        const int sDecent = 7;
        const int sLandingHold = 8;
        const int sLanded = 9;

        const int sGeneralError = 10;
        const int sSDError = 11;
        const int sIMUError = 12;
        const int sBAROError = 13;
        const int sLowBatteryVoltage = 14;

        // LED
        const int updateFreq = 1000; // The number of ms the LED is on/off
        int lastUpdate = 0; // The last time we sent an update to the LED
        bool offThisUpdate = false; // Whether this update the LEDs should be off (Meaning next update they should be on)

        // Flight milestones
        int launchTime = 0; // Time of launch
        int burnoutTime = 0; // Time of burnout
        int apogeeTime = 0; // Time of apogee
        int parachuteTime = 0; // Time of parachute deployment
        int touchdownTime = 0; // Time of touchdown

        /** Returns whether the state machine is in an error state
         * @returns bool
         */
        bool isErroring() {
            return (state >= 10) && (state < 20);
        }

        /** Returns whether the state machine is in a flight state
         * @returns bool
         */
        bool inFlight() {
            return (state >= 4) && (state <= 8);
        }
};
StateMachineSubsystem stateMachine;

class LoggingSubsystem {
    private:
        MetaSubsystem* metaSub;
        char buffer[256];
    public:
        File file;
        char filename[19] = "Flight-log-0.csv"; // The filename for the flight log (Supports up to 999 flight files)
        char postFilename[26] = "Flight-log-0.postflight"; // The filename for the postflight log (Supports up to 999 flight files)
        int flightNum = 0; // The flight number for use in filenames

        const int flushFreq = 500; // The minimum number of ms between log flushes
        int lastFlush = 0; // The last time we flushed the log
        int lastLog = 0; // The last time we logged a single record

        LoggingSubsystem(MetaSubsystem* metaSub) {
            this->metaSub = metaSub;
        }

        /** Fast log. For use when logging during flight, when logging must be done as fast as possible
         * @param in
         * @param trailingComma Whether to add a trailing comma to in (Default: true)
         */
        void flog(char in[], bool trailingComma = true) {
            this->file.print(in);

            if (trailingComma) {
                this->file.print(",");
            }
        }
        void flog(int in, bool trailingComma = true) {
            sprintf(metaSub->buffer, ((trailingComma) ? "%i," : "%i"), in);
            this->file.print(metaSub->buffer);
        }
        void flog(float in, bool trailingComma = true) {
            metaSub->splitFloat(metaSub->floatBuffer, in);

            sprintf(metaSub->buffer, ((trailingComma) ? "%i.%i," : "%i.%i"), metaSub->floatBuffer[0], metaSub->floatBuffer[1]);
            this->file.print(metaSub->buffer);
        }
        void flog(bool in, bool trailingComma = true) {
            int boolAsInt = in;

            sprintf(metaSub->buffer, ((trailingComma) ? "%i," : "%i"), boolAsInt);
            this->file.print(metaSub->buffer);
        }

        /** Logs an object to the log file
         * @param in
         */
        template <typename T>
        void log(T in) {
            metaSub->toCharArray(in);
            file.print(metaSub->buffer);
        }

        /** Logs an object to the log file with a linebreak
         * @param in
         */
        template <typename T>
        void logln(T in) {
            metaSub->toCharArray(in);
            file.println(metaSub->buffer);
        }

        /** Logs an object to the log file with a label
         * @param in
         */
        template <typename T>
        void log(char label[], T in) {
            file.print(label);
            metaSub->toCharArray(in);
            file.print(metaSub->buffer);
        }

        /** Logs an object to the log file with a label and a linebreak
         * @param in
         */
        template <typename T>
        void logln(char label[], T in) {
            file.print(label);
            metaSub->toCharArray(in);
            file.println(metaSub->buffer);
        }
};
LoggingSubsystem logging(&meta);

class IMUSubsystem {
    public:
        Adafruit_LSM9DS1 a = Adafruit_LSM9DS1();

        sensors_event_t accel;
        sensors_event_t mag;
        sensors_event_t gyro;
        sensors_event_t temp;

        XYZ lastAccelData;
        XYZ lastMagData;
        XYZ lastGyroData;
        float lastTempData;

        /** Gets sensor data and stores it in the "last" variables.
         * Accel, mag, and gyro data are converted into our standard axes (Z: up | X: left/right | Y: forward/back)
         * Temp is just stored directly in lastTempData
         */
        void getData() {
            this->a.getEvent(&this->accel, &this->mag, &this->gyro, &this->temp);

            lastAccelData = {
                (this->accel.acceleration.z * -1),
                this->accel.acceleration.y,
                this->accel.acceleration.x - 9.81
            };
            lastMagData = {
               (this->mag.magnetic.z * -1),
                this->mag.magnetic.y,
                this->mag.magnetic.x
            };
            lastGyroData = {
               (this->gyro.gyro.z * -1),
                this->gyro.gyro.y,
                this->gyro.gyro.x
            };
            lastTempData = temp.temperature;
        }
};
IMUSubsystem IMU;

typedef struct {
    Adafruit_BMP3XX a;
    const float seaLevelPress = 1013.25; // Air pressure at sea level in hPa
    float initAlt = 0.0; // The initial altitude of the rocket on startup
    float maxAlt = 0.0; // The maximum recorded altitude during flight
} BAROStruct;
BAROStruct BARO;

typedef struct {
    int lastUpdate; // The last time we estimated velocity
    float lastZAccel; // Z accel from the last cycle
    float lastAlt; // Altitude from the last cycle

    float inte; // Velocity from integrated z accel data (in m/s)
    float diff; // Velocity from differentiated altitude data (in m/s)
    float velocity; // Velocity from this.inte and this.diff passed through a complementary filter (in m/s)

    bool lastVarsSetup = false; // Whether the "last" variables have been set
} VelocityEstStruct;
VelocityEstStruct velocityEst;

class ApogeeEstSubsystem {
    public:
        const float rocketWetMass = 0.6204; // The mass of the entire rocket before flight (in kg)
        const float burnoutSubtrahend = 0.018; // The constant (in kg) that will be subtracted from rocketWetMass resulting in rocket's burnout mass (F52: 0.0366, F50: 0.038, F67: 0.03)

        float lastEstApogee = 0.0; // The last result of calling this.estApogee(), which is our last estimated apogee (in m)

        /** Estimates apogee using math
         * @param alt The current altitude of the rocket (in m)
         * @param initAlt The initial/ground altitude of the rocket before launch (in m)
         * @param velocity The current upwards (Unified axis Z) velocity of the rocket (in m/s)
         * @param abAngle The current angle of the air brakes (in deg)
         * @param minAbAngle The minimum air brake angle (in deg)
         * @param maxAbAngle The maximum air brake angle (in deg)
         * @param press The air pressure (in Pa) (Temporarily not used)
         * @param temp The air temperature (in C) (Temporarily not used)
         * @returns Estimated apogee (in m)
         */
        float estApogee(float alt, float initAlt, float velocity, float abAngle, float minAbAngle, float maxAbAngle, float press, float temp) {
            // Constrain abAngle
            if (abAngle < minAbAngle) {
                // Error
                abAngle = minAbAngle;
            } else if (abAngle > maxAbAngle) {
                // Error
                abAngle = maxAbAngle;
            }

            // Est apogee
            float CdA = 0.0;
            if (abAngle < 0.25) { // This avoids log(0) error
                CdA = 0.004882;
            } else {
                CdA = (log(abAngle) / 2450) + 0.00548;
            }

            float atmosphericDensity = 1.1; // press / (287.05 * (temp + 273.1)); // 287.05 is the specific gas constant for air
            float k = .5 * atmosphericDensity * CdA;
            float burnoutMass = rocketWetMass - burnoutSubtrahend;

            lastEstApogee = (burnoutMass / (2 * k)) * log(((k * velocity * velocity) / (burnoutMass * 9.81)) + 1); // Calculate how much further the rocket will go
            lastEstApogee += (alt - initAlt); // Add how far the rocket has gone since launch to get the est apogee

            return lastEstApogee;
        }
};
ApogeeEstSubsystem apogeeEst;

class CycleSubsystem {
    private:
        int avgDataXYZParentSize = 3;
        int avgDataXYZChildSize = 5;
        XYZ avgDataXYZ[3][5] = {
            {}, // accel
            {}, // mag
            {} // gyro
        };

        int avgDataFloatParentSize = 4;
        int avgDataFloatChildSize = 5;
        float avgDataFloat[5][5] = {
            {}, // IMU temp
            {}, // BARO temp
            {}, // Press
            {}, // Alt
            {0.0, 0.0, 0.0, 0.0, 0.0} // Diff velocity
        };
    public:
        int millis = 0; // Time (in ms) since power on

        // IMU
        XYZ accel = {0.0, 0.0, 0.0}; // Running avg of accel data (in m/s^2)
        XYZ mag = {0.0, 0.0, 0.0}; // Running avg of mag data (in uT)
        XYZ gyro = {0.0, 0.0, 0.0}; // Running avg of gyro data (in rad/s)
        float IMUTemp = 0.0; // Running avg of IMU temperature (in C)

        XYZ rawAccel = {0.0, 0.0, 0.0}; // Raw accel data (in m/s^2)
        XYZ rawMag = {0.0, 0.0, 0.0}; // Raw mag data (in uT)
        XYZ rawGyro = {0.0, 0.0, 0.0}; // Raw gyro data (in rad/s)
        float rawIMUTemp = 0.0; // Raw IMU temperature (in C)

        // BARO
        float BAROTemp = 0.0; // Running avg of BARO temperature (in C)
        float press = 0.0; // Running avg of pressure (in Pa)
        float alt = 0.0; // Running avg of altitude (in m)

        float rawBAROTemp = 0.0; // Raw BARO temperature (in C)
        float rawPress = 0.0; // Raw pressure (in Pa)
        float rawAlt = 0.0; // Raw altitude (in m)

        // Velocity
        float diffVelocity = 0.0;

        /** Calculates new running averages given more data and stores them in corresponding class variables
         * @param accel
         * @param mag
         * @param gyro
         * @param IMUTemp
         * @param BAROTemp
         * @param press
         * @param alt
         */
        void calculateRunningAvgs(XYZ accel, XYZ mag, XYZ gyro, float IMUTemp, float BAROTemp, float press, float alt) {
            // Shift data
            for (int i = 0; i < avgDataXYZParentSize; i++) {
                for (int j = (avgDataXYZChildSize - 2); j >= 0; j--) {
                    avgDataXYZ[i][j + 1] = avgDataXYZ[i][j]; // Copy j to (j + 1)
                }
            }

            for (int i = 0; i < avgDataFloatParentSize; i++) {
                for (int j = (avgDataFloatChildSize - 2); j >= 0; j--) {
                    avgDataFloat[i][j + 1] = avgDataFloat[i][j]; // Copy j to (j + 1)
                }
            }

            // Insert new data
            avgDataXYZ[0][0] = accel;
            avgDataXYZ[1][0] = mag;
            avgDataXYZ[2][0] = gyro;

            avgDataFloat[0][0] = IMUTemp;
            avgDataFloat[1][0] = BAROTemp;
            avgDataFloat[2][0] = press;
            avgDataFloat[3][0] = alt;

            // Calculate new averages
            for (int i = 0; i < avgDataXYZParentSize; i++) {
                XYZ averagedData = {
                    0.0,
                    0.0,
                    0.0
                };

                for (int j = 0; j < avgDataXYZChildSize; j++) {
                    averagedData.x += avgDataXYZ[i][j].x;
                    averagedData.y += avgDataXYZ[i][j].y;
                    averagedData.z += avgDataXYZ[i][j].z;
                }

                averagedData.x /= avgDataXYZChildSize;
                averagedData.y /= avgDataXYZChildSize;
                averagedData.z /= avgDataXYZChildSize;

                if (i == 0) {
                    this->accel = averagedData;
                } else if (i == 1) {
                    this->mag = averagedData;
                } else if (i == 2) {
                    this->gyro = averagedData;
                }
            }

            for (int i = 0; i < avgDataFloatParentSize; i++) {
                float averagedData = 0.0;

                for (int j = 0; j < avgDataFloatChildSize; j++) {
                    averagedData += avgDataFloat[i][j];
                }

                averagedData /= avgDataFloatChildSize;

                if (i == 0) {
                    this->IMUTemp = averagedData;
                } else if (i == 1) {
                    this->BAROTemp = averagedData;
                } else if (i == 2) {
                    this->press = averagedData;
                } else if (i == 3) {
                    this->alt = averagedData;
                }
            }
        }

        /** Calculates the running average of differentiated velocity and stores it diffVelocity
         * @param velocity
         */
        void calculateRunningAvgOfDiffVelocity(float velocity) {
            avgDataFloat[4][4] = avgDataFloat[4][3];
            avgDataFloat[4][3] = avgDataFloat[4][2];
            avgDataFloat[4][2] = avgDataFloat[4][1];
            avgDataFloat[4][1] = avgDataFloat[4][0];
            avgDataFloat[4][0] = velocity;

            diffVelocity = 0.0;

            for (int i = 0; i < 5; i++) {
                diffVelocity += avgDataFloat[4][i];
            }

            diffVelocity /= 5;
        }
};
CycleSubsystem cycle;

class AirBrakesSubsystem {
    private:
        Adafruit_PWMServoDriver* pwmController;
    public:
        const int min = 150; // Minimum value for the air brakes (in PWM)
        const int max = 400; // Maximum value for the air brakes (in PWM)
        const int pins[4] = {4, 5, 6, 7}; // Pins that the air brakes use (Front, left, back, right)
        const int offsets[4] = {0, +5, +20, +20}; // Offsets for each air brake to make them sync up. Offsets are added to the base (in PWM)

        const float maxAngle = 51.0; // Angle of the air brakes when set to this.max (in deg)
        float currentAngle = 0.0; // The current angle of the air brakes (in deg)
        float targetAngle = 0.0; // Target air brake angle to reach the target apogee calculated in flight (in deg)
        float startingAngle = 30.5; // Starting angle of the air brakes on accent (in deg)

        const float Kp = 0.0; // Proportional multiplier
        const float Ki = 3.0; // Integral multiplier
        const float Kd = 0.0; // Derivative multiplier

        int lastUpdate = 0; // The last time we did PID
        float lastError = 0.0; // The last error (Used in calculating I)
        float cumulativeI = 0.0; // The cumulative I value
        float error = 0.0; // Current error. Not used in the PID function, but exists so we can log the error

        const bool useStaticAngle = false; // Whether to use a static PWM value during accent instead of the active control system. (Note: Disabling active control via setting this to true, still requires a lot of the variables and equations in this section to work, so set those accordingly)
        const int pwmTarget = 0; // The value to set the air brakes to during accent, if this.useStaticAngle is true (in PWM)

        AirBrakesSubsystem(Adafruit_PWMServoDriver* pwmController) {
            this->pwmController = pwmController;
        }

        /** Calculates the output of the PID controller, given some error
         * @param error The PID input
         * @param currentTime The current time (in ms)
         * @returns PID output
         */
        float calculatePID(float error, float currentTime) {
            float deltaTime = (currentTime - lastUpdate) / 1000; // Time since last calculation (in s)

            cumulativeI += 0.5 * deltaTime * (error + lastError);

            // Windup protection (Prevents cumulativeI from getting too big)
            if (cumulativeI < 0) {
                cumulativeI = 0;
            } else if (cumulativeI > (maxAngle / Ki)) {
                cumulativeI = maxAngle / Ki;
            }

            // Factor in multipliers
            float p = Kp * error;
            float i = Ki * cumulativeI;
            float d = Kd * ((error - lastError) / deltaTime);

            // Save "last" variables
            lastUpdate = currentTime;
            lastError = error;

            return p + i + d;
        }

        /** Sets the air brakes to a value (in PWM)
         * @param PWM
         */
        void setPWM(int PWM) {
            currentAngle = PWMToAngle(PWM);

            for (int i = 0; i < 5; i++) {
                pwmController->setPWM(pins[i], 0, (PWM + offsets[i]));
            }
        }

        /** Sets the air brakes to an angle (in deg)
         * @param angle
         */
        void setAngle(int angle) {
            currentAngle = angle;
            int PWM = angleToPWM(angle);

            for (int i = 0; i < 5; i++) {
                pwmController->setPWM(pins[i], 0, (PWM + offsets[i]));
            }
        }

        /** Converts an angle (in deg) to a PWM value
         * @param angle
         * @returns PWM value
         */
        float angleToPWM(float angle) {
            return (4.9 * angle) + 150;
        }

        /** Converts a PWM value to an angle (in deg)
         * @param PWM
         * @returns Angle
         */
        float PWMToAngle(float PWM) {
            return (PWM - 150) / 4.9;
        }
 };
AirBrakesSubsystem airBrakes(&PWM);

typedef struct {
    int updateFreq = 1000; // Time between heartbeats
    int heartbeatLength = 100; // Minumum time of one heartbeat
    int heartbeatPWM = 4096; // The PWM value of the heartbeat

    int pin = 15; // The pin that the buzzer uses
    int lastUpdate = 0; // The last time we played a heartbeat
    bool heartbeatPlaying = false; // Whether the heartbeat is currently playing
} BuzzerStruct;
BuzzerStruct buzzer;

typedef struct {
    char buffer[256];
    char letter;
    float counter;
} TempStruct;
TempStruct temp;

// ---- //
/* Main */
// ---- //

/** Code put here is run once at startup */
void setup() {
    /*
     * Variable setup
     */
    Serial.begin(9600);
    Serial.setTimeout(0);

    stateMachineLED.begin();
    stateMachineLED.setBrightness(1); // Set the brightness (1-255) (Helpful for testing)
    stateMachineLED.setPixelColor(0, 255, 255, 255);
    stateMachineLED.show();

    PWM.begin();
    PWM.setPWMFreq(60);

    /*
     * Subsystem setup
     */

    // Logging
    if (SD.begin(SD_DETECT_PIN)) {
        delay(2000); // Prevent a bad plugin from causing fs problems

        while (SD.exists(logging.filename)) { // Find the first filename that isn't used
            logging.flightNum += 1;
            sprintf(logging.filename, "Flight-log-%i.csv", logging.flightNum);
            sprintf(logging.postFilename, "Flight-log-%i.postflight", logging.flightNum);
        }

        logging.file = SD.open(logging.filename, FILE_WRITE);

        if (logging.file) {
            // **REMINDER**: Use unified log headers

            logging.flog("millis(),Current_state,Calculated_flight_time,Time_since_last_log");

            // IMU
            logging.flog("IMU_accel_X,IMU_accel_Y,IMU_accel_Z");
            logging.flog("IMU_mag_X,IMU_mag_Y,IMU_mag_Z");
            logging.flog("IMU_gyro_X,IMU_gyro_Y,IMU_gyro_Z");
            logging.flog("IMU_temp");

            // Raw IMU
            logging.flog("Raw_IMU_accel_X,Raw_IMU_accel_Y,Raw_IMU_accel_Z");
            logging.flog("Raw_IMU_mag_X,Raw_IMU_mag_Y,Raw_IMU_mag_Z");
            logging.flog("Raw_IMU_gyro_X,Raw_IMU_gyro_Y,Raw_IMU_gyro_Z");
            logging.flog("Raw_IMU_temp");

            // BARO
            logging.flog("BARO_temp,BARO_press,BARO_alt");
            logging.flog("Raw_BARO_temp,Raw_BARO_press,Raw_BARO_alt");

            // Velocity
            logging.flog("Velocity_lastUpdate,Velocity_lastZAccel,Velocity_lastAlt");
            logging.flog("Velocity_inte,Velocity_diff,Velocity_est");

            // ACS
            logging.flog("Est_apogee");
            logging.flog("Target_air_brake_angle");
            logging.flog("Air_brake_angle");
            logging.flog("Air_brake_error");

            // Other
            logging.flog("Battery_voltage");
            logging.flog("Z_accel_times_burnoutMass", false);

            logging.flog("\n", false);
            logging.file.flush();
        } else {
            stateMachine.state = stateMachine.sSDError;
        }
    } else {
        stateMachine.state = stateMachine.sSDError;
    }

    // IMU
    if (IMU.a.begin()) {
        IMU.a.setupAccel(IMU.a.LSM9DS1_ACCELRANGE_16G);
        IMU.a.setupMag(IMU.a.LSM9DS1_MAGGAIN_4GAUSS);
        IMU.a.setupGyro(IMU.a.LSM9DS1_GYROSCALE_500DPS);
    } else {
        stateMachine.state = stateMachine.sIMUError;
    }

    // BARO
    if (BARO.a.begin_I2C()) {
        bool BAROSetupResults[4];
        BAROSetupResults[0] = BARO.a.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
        BAROSetupResults[1] = BARO.a.setPressureOversampling(BMP3_NO_OVERSAMPLING);
        BAROSetupResults[2] = BARO.a.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
        BAROSetupResults[3] = BARO.a.setOutputDataRate(BMP3_ODR_200_HZ);

        if (!BAROSetupResults[0] || !BAROSetupResults[1] || !BAROSetupResults[2] || !BAROSetupResults[3]) { // One of our sets didn't work
            stateMachine.state = stateMachine.sBAROError;
        } else {
            // Get the first few altitude readings because they seem to be crazy inaccurate
            for (int i = 1; i <= 50; i++) {
                BARO.a.readAltitude(BARO.seaLevelPress);
            }
        }
    } else {
        stateMachine.state = stateMachine.sBAROError;
    }

    /*
     * Final setup
     */
    if (getBatteryVoltage() <= 3.5) { // Ensure there is enough battery for the flight
        stateMachine.state = stateMachine.sLowBatteryVoltage;
    }

    calibrationAndCycleSetup();

    if (stateMachine.state == stateMachine.sBootUp) {
        stateMachine.state = stateMachine.sAwaitingArming;
    }
}

/** Code put here runs once every cycle */
void loop() {
    // Setup cycle variables
    cycle.millis = millis();

    IMU.getData();
    cycle.rawAccel = IMU.lastAccelData;
    cycle.rawMag = IMU.lastMagData;
    cycle.rawGyro = IMU.lastGyroData;
    cycle.rawIMUTemp = IMU.lastTempData;

    cycle.rawBAROTemp = BARO.a.readTemperature();
    cycle.rawPress = BARO.a.readPressure();
    cycle.rawAlt = BARO.a.readAltitude(BARO.seaLevelPress);

    cycle.calculateRunningAvgs(
        IMU.lastAccelData,
        IMU.lastMagData,
        IMU.lastGyroData,
        IMU.lastTempData,
        cycle.rawBAROTemp,
        cycle.rawPress,
        cycle.rawAlt
    );

    // Handle debug commands sent via Serial
    if ((Serial && Serial.available() > 0) || debugging.watchEnabled) {
        debugging.command = Serial.readString();
        debugging.command.trim();

        if (debugging.watchEnabled && debugging.command == "") { // Handle setting debugging.command to whatever debugging.watchCommand is, if needed
            debugging.command = debugging.watchCommand;
        }

        // Create debugging.commandArr and debugging.commandFirstWord
        debugging.command.toCharArray(debugging.commandArr, sizeof(debugging.commandArr));
        extractFirstWord(debugging.commandFirstWord, debugging.commandArr, sizeof(debugging.commandFirstWord), sizeof(debugging.commandArr));

        // Print out what the user gave us
        Serial.print("\n\n");
        Serial.println("> " + debugging.command);

        // Handle commands
        if (debugging.command == "HELP" || debugging.command == "help") {
            Serial.println("All available commands are listed below.\n");
            Serial.println("air-brakes [a]              Manually set all air brakes to air brake angle [a]");
            Serial.println("                                This includes offsets. Max 999 for [a]");
            Serial.println("battery-voltage             Check and report battery voltage");
            Serial.println("pre-flight                  Run pre-flight checks for state and log file");
            Serial.println("post-flight                 Create .postflight report and force touchdown state");
            Serial.println("PWM-set [a] [b]             Set PWM on pin [a] to [b]. Max 99 for [a] and 999 for [b]");
            Serial.println("sensor-data [a]             Get data from sensor [a]");
            Serial.println("    IMU");
            Serial.println("    BARO");
            Serial.println("setup-delay [a]             Pause all code execution for [a] minutes. Max 999 for [a]");
            Serial.println("test                        A temporary command for debug and other testing");
            Serial.println("variables [a]               Print all variables belonging to subsystem [a]");
            Serial.println("    meta");
            Serial.println("    debugging");
            Serial.println("    state-machine");
            Serial.println("    logging");
            Serial.println("    IMU");
            Serial.println("    BARO");
            Serial.println("    velocity-est");
            Serial.println("    apogee-est");
            Serial.println("    cycle");
            Serial.println("    air-brakes");
            Serial.println("    buzzer");
            Serial.println("    temp");
            Serial.println("watch [a]                   Run command [a] every cycle");
            Serial.println("    --stop                      Stop running [a] every cycle");
        } else if (compareCharArrays("air-brakes", debugging.commandFirstWord)) {
            int value = atoi(spliceCharArray(temp.buffer, debugging.commandArr, 11, 14));

            airBrakes.setPWM(value);
            debugging.println("Set air brakes to ", value);
        } else if (debugging.command == "battery-voltage") {
            debugging.println("Battery voltage at ", getBatteryVoltage());
        } else if (debugging.command == "pre-flight") {
            if (logging.file && stateMachine.state == stateMachine.sAwaitingArming) {
                Serial.println("GO for launch!");
            } else {
                Serial.println("NO GO for launch!");
                debugging.println("Current state: ", stateMachine.state);
            }
        } else if (debugging.command == "post-flight") {
            debugging.println("Initial alt (ft):   ", BARO.initAlt * 3.2808f);
            debugging.println("Max alt (ft):       ", BARO.maxAlt * 3.2808f);
            debugging.println("Apogee (ft):        ", (BARO.maxAlt - BARO.initAlt) * 3.2808f);

            debugging.println("Launch time (s):    ", stateMachine.launchTime / 1000);
            debugging.println("Touchdown time (s): ", stateMachine.touchdownTime / 1000);
            debugging.println("Flight time (s):    ", (stateMachine.touchdownTime - stateMachine.launchTime) / 1000);

            logging.file.flush(); // DO NOT REMOVE. Log files aren't flushed with close()
            logging.file.close(); // Close the existing log file
            delay(2000); // Delay to prevent SD errors with the next open()
            logging.file = SD.open(logging.postFilename, FILE_WRITE);

            if (logging.file) {
                logging.log("=== Post Flight Report for F ===\n\n== Flight Info ==\n\nWhether: Include temp and wind\nApogee: Include PNUT and BARO information\nAnomalies: Include any anomalies that occurred with this flight\n\n== Flight Goals ==\n\nList flight goals and outcomes\n\n");
                logging.log("== Variables ==\n\nNote: Variables are printed at the end of the flight.\n");

                logging.logln("\nMeta");
                logging.logln("buffer:       ", meta.buffer);
                logging.logln("floatBuffer:  ", meta.floatBuffer);
                logging.logln("---");
                logging.logln("targetApogee: ", meta.targetApogee);

                logging.logln("\nSerial debugging");
                logging.logln("command:          ", debugging.command);
                logging.logln("commandArr:       ", debugging.commandArr);
                logging.logln("commandFirstWord: ", debugging.commandFirstWord);
                logging.logln("---");
                logging.logln("watchEnabled: ", debugging.watchEnabled);
                logging.logln("watchCommand: ", debugging.watchCommand);
                logging.logln("watchDelay:   ", debugging.watchDelay);

                logging.logln("\nState machine");
                logging.logln("state: ", stateMachine.state);
                logging.logln("---");
                logging.logln("sBootUp:            ", stateMachine.sBootUp);
                logging.logln("sAwaitingArming:    ", stateMachine.sAwaitingArming);
                logging.logln("sAwaitingLaunch:    ", stateMachine.sAwaitingLaunch);
                logging.logln("sPoweredAccent:     ", stateMachine.sPoweredAccent);
                logging.logln("sUnpoweredAccent:   ", stateMachine.sUnpoweredAccent);
                logging.logln("sParachuteHold:     ", stateMachine.sParachuteHold);
                logging.logln("sDecent:            ", stateMachine.sDecent);
                logging.logln("sLandingHold:       ", stateMachine.sLandingHold);
                logging.logln("sLanded:            ", stateMachine.sLanded);
                logging.logln("sGeneralError:      ", stateMachine.sGeneralError);
                logging.logln("sSDError:           ", stateMachine.sSDError);
                logging.logln("sIMUError:          ", stateMachine.sIMUError);
                logging.logln("sBAROError:         ", stateMachine.sBAROError);
                logging.logln("sLowBatteryVoltage: ", stateMachine.sLowBatteryVoltage);
                logging.logln("---");
                logging.logln("updateFreq:    ", stateMachine.updateFreq);
                logging.logln("lastUpdate:    ", stateMachine.lastUpdate);
                logging.logln("offThisUpdate: ", stateMachine.offThisUpdate);
                logging.logln("---");
                logging.logln("launchTime:    ", stateMachine.launchTime);
                logging.logln("burnoutTime:   ", stateMachine.burnoutTime);
                logging.logln("apogeeTime:    ", stateMachine.apogeeTime);
                logging.logln("parachuteTime: ", stateMachine.parachuteTime);
                logging.logln("touchdownTime: ", stateMachine.touchdownTime);

                logging.logln("\nLogging");
                logging.logln("file:         ", logging.file);
                logging.logln("filename:     ", logging.filename);
                logging.logln("postFilename: ", logging.postFilename);
                logging.logln("flightNum:    ", logging.flightNum);
                logging.logln("---");
                logging.logln("flushFreq: ", logging.flushFreq);
                logging.logln("lastFlush: ", logging.lastFlush);
                logging.logln("lastLog:   ", logging.lastLog);

                logging.logln("\nIMU");
                logging.logln("lastAccelData: ", IMU.lastAccelData);
                logging.logln("lastMagData:   ", IMU.lastMagData);
                logging.logln("lastGyroData:  ", IMU.lastGyroData);
                logging.logln("lastTempData:  ", IMU.lastTempData);

                logging.logln("\nBARO");
                logging.logln("seaLevelPress: ", BARO.seaLevelPress);
                logging.logln("initAlt:       ", BARO.initAlt);
                logging.logln("maxAlt:        ", BARO.maxAlt);

                logging.logln("\nVelocity est");
                logging.logln("lastUpdate: ", velocityEst.lastUpdate);
                logging.logln("lastZAccel: ", velocityEst.lastZAccel);
                logging.logln("lastAlt:    ", velocityEst.lastAlt);
                logging.logln("---");
                logging.logln("inte:     ", velocityEst.inte);
                logging.logln("diff:     ", velocityEst.diff);
                logging.logln("velocity: ", velocityEst.velocity);
                logging.logln("---");
                logging.logln("lastVarsSetup: ", velocityEst.lastVarsSetup);

                logging.logln("\nApogee est");
                logging.logln("rocketWetMass:         ", apogeeEst.rocketWetMass);
                logging.logln("burnoutSubtrahend:     ", apogeeEst.burnoutSubtrahend);
                logging.logln("---");
                logging.logln("lastEstApogee: ", apogeeEst.lastEstApogee);

                logging.logln("\nCycle");
                logging.logln("millis: ", cycle.millis);
                logging.logln("---");
                logging.logln("accel:   ", cycle.accel);
                logging.logln("mag:     ", cycle.mag);
                logging.logln("gyro:    ", cycle.gyro);
                logging.logln("IMUTemp: ", cycle.IMUTemp);
                logging.logln("---");
                logging.logln("rawAccel:   ", cycle.rawAccel);
                logging.logln("rawMag:     ", cycle.rawMag);
                logging.logln("rawGyro:    ", cycle.rawGyro);
                logging.logln("rawIMUTemp: ", cycle.rawIMUTemp);
                logging.logln("---");
                logging.logln("BAROTemp: ", cycle.BAROTemp);
                logging.logln("press:    ", cycle.press);
                logging.logln("alt:      ", cycle.alt);
                logging.logln("---");
                logging.logln("rawBAROTemp: ", cycle.rawBAROTemp);
                logging.logln("rawPress:    ", cycle.rawPress);
                logging.logln("rawAlt:      ", cycle.rawAlt);

                logging.logln("\nAir brakes");
                logging.logln("min:     ", airBrakes.min);
                logging.logln("max:     ", airBrakes.max);
                    sprintf(meta.buffer, "%i, %i, %i, %i", airBrakes.pins[0], airBrakes.pins[1], airBrakes.pins[2], airBrakes.pins[3]);
                logging.logln("pins:    ", meta.buffer);
                    sprintf(meta.buffer, "%i, %i, %i, %i", airBrakes.offsets[0], airBrakes.offsets[1], airBrakes.offsets[2], airBrakes.offsets[3]);
                logging.logln("offsets: ", meta.buffer);
                logging.logln("---");
                logging.logln("maxAngle:      ", airBrakes.maxAngle);
                logging.logln("currentAngle:  ", airBrakes.currentAngle);
                logging.logln("targetAngle:   ", airBrakes.targetAngle);
                logging.logln("startingAngle: ", airBrakes.startingAngle);
                logging.logln("---");
                logging.logln("Kp: ", airBrakes.Kp);
                logging.logln("Ki: ", airBrakes.Ki);
                logging.logln("Kd: ", airBrakes.Kd);
                logging.logln("---");
                logging.logln("lastUpdate:  ", airBrakes.lastUpdate);
                logging.logln("lastError:   ", airBrakes.lastError);
                logging.logln("cumulativeI: ", airBrakes.cumulativeI);
                logging.logln("error:       ", airBrakes.error);
                logging.logln("---");
                logging.logln("useStaticAngle: ", airBrakes.useStaticAngle);
                logging.logln("pwmTarget:      ", airBrakes.pwmTarget);

                logging.logln("\nBuzzer");
                logging.logln("updateFreq:      ", buzzer.updateFreq);
                logging.logln("heartbeatLength: ", buzzer.heartbeatLength);
                logging.logln("heartbeatLength: ", buzzer.heartbeatPWM);
                logging.logln("---");
                logging.logln("pin:              ", buzzer.pin);
                logging.logln("lastUpdate:       ", buzzer.lastUpdate);
                logging.logln("heartbeatPlaying: ", buzzer.heartbeatPlaying);

                logging.logln("\nThis is the last line.");

                // Close log file and set state
                logging.file.flush(); // DO NOT REMOVE. Log files aren't flushed with close()
                logging.file.close();
                stateMachine.state = stateMachine.sLanded;
            } else {
                stateMachine.state = stateMachine.sSDError;
            }

            Serial.println("Post-flight measures completed!");
        } else if (compareCharArrays("PWM-set", debugging.commandFirstWord)) {
            int pin = atoi(spliceCharArray(temp.buffer, debugging.commandArr, 8, 10));
            int value = atoi(spliceCharArray(temp.buffer, debugging.commandArr, 11, 14));

            PWM.setPWM(pin, 0, value);
            sprintf(temp.buffer, "Set PWM on pin %i to %i", pin, value); Serial.println(temp.buffer);
        } else if (compareCharArrays("sensor-data", debugging.commandFirstWord)) {
            char token[4];
            spliceCharArray(token, debugging.commandArr, 12, 16);

            if (compareCharArrays(token, "IMU")) {
                debugging.println("Accel (in m/s^2): ", cycle.accel);
                debugging.println("Mag (in uT):      ", cycle.mag);
                debugging.println("Gyro (in rad/s):  ", cycle.gyro);
                debugging.println("Temp (in C):      ", cycle.IMUTemp);
            } else if (compareCharArrays(token, "BARO")) {
                debugging.println("Temp (in C):   ", cycle.BAROTemp);
                debugging.println("Press (in Pa): ", cycle.press);
                debugging.println("Alt (in m):    ", cycle.alt);
            } else {
                Serial.println("Incorrect or missing token! Use HELP for help");
            }

            delay(debugging.watchDelay);
        } else if (compareCharArrays("setup-delay", debugging.commandFirstWord)) {
            char token[3];
            spliceCharArray(token, debugging.commandArr, 12, 15);

            if (token == "") {
                Serial.println("No amount of time specified! Use HELP for help.");
            } else {
                int delayTime = atoi(token); // Minutes to delay

                // Change the LED color as an indicator
                stateMachineLED.setPixelColor(0, 255, 255, 0);
                stateMachineLED.show();

                // Print a warning message
                sprintf(temp.buffer, "Activating setup delay for %i minute(s)!", delayTime); Serial.println(temp.buffer);
                Serial.println("Serial debugging commands will be unavailable until this delay is over");
                Serial.println("For the last minute, the buzzer will buzz as a warning");
                Serial.println("To remove this delay early, power cycle the flight computer");

                // Pause execution for a minute at a time so we can provide status updates
                for (int i = delayTime; i > 0; i--) {
                    sprintf(temp.buffer, "%i minute(s) remaining...", i); Serial.println(temp.buffer);

                    if (i == 1) {
                        PWM.setPWM(buzzer.pin, 0, buzzer.heartbeatPWM);
                        delay(60000);
                    } else {
                        delay(60000);
                    }
                }

                // Get rocket ready for flight
                calibrationAndCycleSetup();
                if (stateMachine.state == stateMachine.sAwaitingArming) {
                    stateMachine.state = stateMachine.sAwaitingLaunch;
                }

                Serial.println("Setup delay complete! Now resuming normal code execution");
            }
        } else if (debugging.command == "test") {
            Serial.println("Nothing to test");
        } else if (compareCharArrays("variables", debugging.commandFirstWord)) {
            char token[20];
            spliceCharArray(token, debugging.commandArr, 10, 30);

            if (compareCharArrays(token, "meta")) {
                debugging.println("buffer:      ", meta.buffer);
                debugging.println("floatBuffer: ", meta.floatBuffer);

                debugging.println("");
                debugging.println("targetApogee: ", meta.targetApogee);
            } else if (compareCharArrays(token, "debugging")) {
                debugging.println("command:          ", debugging.command);
                debugging.println("commandArr:       ", debugging.commandArr);
                debugging.println("commandFirstWord: ", debugging.commandFirstWord);

                debugging.println("");
                debugging.println("watchEnabled: ", debugging.watchEnabled);
                debugging.println("watchCommand: ", debugging.watchCommand);
                debugging.println("watchDelay:   ", debugging.watchDelay);
            } else if (compareCharArrays(token, "state-machine")) {
                debugging.println("state: ", stateMachine.state);

                debugging.println("");
                debugging.println("sBootUp:          ", stateMachine.sBootUp);
                debugging.println("sAwaitingArming:  ", stateMachine.sAwaitingArming);
                debugging.println("sAwaitingLaunch:  ", stateMachine.sAwaitingLaunch);
                debugging.println("sPoweredAccent:   ", stateMachine.sPoweredAccent);
                debugging.println("sUnpoweredAccent: ", stateMachine.sUnpoweredAccent);
                debugging.println("sParachuteHold:   ", stateMachine.sParachuteHold);
                debugging.println("sDecent:          ", stateMachine.sDecent);
                debugging.println("sLandingHold:     ", stateMachine.sLandingHold);
                debugging.println("sLanded:          ", stateMachine.sLanded);

                debugging.println("");
                debugging.println("sGeneralError:      ", stateMachine.sGeneralError);
                debugging.println("sSDError:           ", stateMachine.sSDError);
                debugging.println("sIMUError:          ", stateMachine.sIMUError);
                debugging.println("sBAROError:         ", stateMachine.sBAROError);
                debugging.println("sLowBatteryVoltage: ", stateMachine.sLowBatteryVoltage);

                debugging.println("");
                debugging.println("updateFreq:    ", stateMachine.updateFreq);
                debugging.println("lastUpdate:    ", stateMachine.lastUpdate);
                debugging.println("offThisUpdate: ", stateMachine.offThisUpdate);

                debugging.println("");
                debugging.println("launchTime:    ", stateMachine.launchTime);
                debugging.println("burnoutTime:   ", stateMachine.burnoutTime);
                debugging.println("apogeeTime:    ", stateMachine.apogeeTime);
                debugging.println("parachuteTime: ", stateMachine.parachuteTime);
                debugging.println("touchdownTime: ", stateMachine.touchdownTime);
            } else if (compareCharArrays(token, "logging")) {
                debugging.println("filename:     ", logging.filename);
                debugging.println("postFilename: ", logging.postFilename);
                debugging.println("flightNum:    ", logging.flightNum);

                debugging.println("");
                debugging.println("flushFreq:    ", logging.flushFreq);
                debugging.println("lastFlush:    ", logging.lastFlush);
                debugging.println("lastLog:      ", logging.lastLog);
            } else if (compareCharArrays(token, "IMU")) {
                debugging.println("lastAccelData: ", IMU.lastAccelData);
                debugging.println("lastMagData:   ", IMU.lastMagData);
                debugging.println("lastGyroData:  ", IMU.lastGyroData);
                debugging.println("lastTempData:  ", IMU.lastTempData);
            } else if (compareCharArrays(token, "BARO")) {
                debugging.println("seaLevelPress: ", BARO.seaLevelPress);
                debugging.println("initAlt:       ", BARO.initAlt);
                debugging.println("maxAlt:        ", BARO.maxAlt);
            } else if (compareCharArrays(token, "velocity-est")) {
                debugging.println("lastUpdate: ", velocityEst.lastUpdate);
                debugging.println("lastZAccel: ", velocityEst.lastZAccel);
                debugging.println("lastAlt:    ", velocityEst.lastAlt);

                debugging.println("");
                debugging.println("inte:     ", velocityEst.inte);
                debugging.println("diff:     ", velocityEst.diff);
                debugging.println("velocity: ", velocityEst.velocity);

                debugging.println("");
                debugging.println("lastVarsSetup: ", velocityEst.lastVarsSetup);
            } else if (compareCharArrays(token, "apogee-est")) {
                debugging.println("rocketWetMass:         ", apogeeEst.rocketWetMass);
                debugging.println("burnoutSubtrahend:     ", apogeeEst.burnoutSubtrahend);

                debugging.println("");
                debugging.println("lastEstApogee: ", apogeeEst.lastEstApogee);
            } else if (compareCharArrays(token, "cycle")) {
                debugging.println("millis: ", cycle.millis);

                debugging.println("");
                debugging.println("accel:   ", cycle.accel);
                debugging.println("mag:     ", cycle.mag);
                debugging.println("gyro:    ", cycle.gyro);
                debugging.println("IMUTemp: ", cycle.IMUTemp);

                debugging.println("");
                debugging.println("rawAccel:   ", cycle.rawAccel);
                debugging.println("rawMag:     ", cycle.rawMag);
                debugging.println("rawGyro:    ", cycle.rawGyro);
                debugging.println("rawIMUTemp: ", cycle.rawIMUTemp);

                debugging.println("");
                debugging.println("BAROTemp: ", cycle.BAROTemp);
                debugging.println("press:    ", cycle.press);
                debugging.println("alt:      ", cycle.alt);

                debugging.println("");
                debugging.println("rawBAROTemp: ", cycle.rawBAROTemp);
                debugging.println("rawPress:    ", cycle.rawPress);
                debugging.println("rawAlt:      ", cycle.rawAlt);
            } else if (compareCharArrays(token, "air-brakes")) {
                debugging.println("min:     ", airBrakes.min);
                debugging.println("max:     ", airBrakes.max);
                    sprintf(meta.buffer, "%i, %i, %i, %i", airBrakes.pins[0], airBrakes.pins[1], airBrakes.pins[2], airBrakes.pins[3]);
                debugging.println("pins:    ", meta.buffer);
                    sprintf(meta.buffer, "%i, %i, %i, %i", airBrakes.offsets[0], airBrakes.offsets[1], airBrakes.offsets[2], airBrakes.offsets[3]);
                debugging.println("offsets: ", meta.buffer);

                debugging.println("");
                debugging.println("maxAngle:      ", airBrakes.maxAngle);
                debugging.println("currentAngle:  ", airBrakes.currentAngle);
                debugging.println("targetAngle:   ", airBrakes.targetAngle);
                debugging.println("startingAngle: ", airBrakes.startingAngle);

                debugging.println("");
                debugging.println("Kp: ", airBrakes.Kp);
                debugging.println("Ki: ", airBrakes.Ki);
                debugging.println("Kd: ", airBrakes.Kd);

                debugging.println("");
                debugging.println("lastUpdate:  ", airBrakes.lastUpdate);
                debugging.println("lastError:   ", airBrakes.lastError);
                debugging.println("cumulativeI: ", airBrakes.cumulativeI);
                debugging.println("error:       ", airBrakes.error);

                debugging.println("");
                debugging.println("useStaticAngle: ", airBrakes.useStaticAngle);
                debugging.println("pwmTarget:      ", airBrakes.pwmTarget);
            } else if (compareCharArrays(token, "buzzer")) {
                debugging.println("updateFreq:      ", buzzer.updateFreq);
                debugging.println("heartbeatLength: ", buzzer.heartbeatLength);
                debugging.println("heartbeatLength: ", buzzer.heartbeatPWM);

                debugging.println("");
                debugging.println("pin:              ", buzzer.pin);
                debugging.println("lastUpdate:       ", buzzer.lastUpdate);
                debugging.println("heartbeatPlaying: ", buzzer.heartbeatPlaying);
            } else if (compareCharArrays(token, "temp")) {
                debugging.println("buffer:  ", temp.buffer);
                debugging.println("letter:  ", temp.letter);
                debugging.println("counter: ", temp.counter);
            } else {
                Serial.println("Incorrect or missing token! Use HELP for help");
            }
        } else if (compareCharArrays("watch", debugging.commandFirstWord)) {
            char token[64];
            spliceCharArray(token, debugging.commandArr, 6, 71);

            if (compareCharArrays(token, "--stop")) {
                debugging.watchEnabled = false;
            } else {
                sprintf(debugging.watchCommand, token);
                debugging.watchEnabled = true;
            }
        } else {
            Serial.println("Not a valid command. Use HELP for help.");
        }
    }

    // Change state if necessary and perform one-time actions
    if (stateMachine.state == stateMachine.sBootUp) {
        // Error. We should never be in this state here
    } else if (stateMachine.state == stateMachine.sAwaitingArming) {
        // Nothing to do
    } else if ((stateMachine.state == stateMachine.sAwaitingLaunch) && (cycle.accel.z >= 5)) {
        stateMachine.state = stateMachine.sPoweredAccent;
        stateMachine.launchTime = cycle.millis;

        BARO.maxAlt = 0.0;
        velocityEst.inte = 0.0;
        PWM.setPWM(buzzer.pin, 0, 0);
    } else if ((stateMachine.state == stateMachine.sPoweredAccent) && ((cycle.millis - stateMachine.launchTime) >= 500) && (cycle.accel.z <= 3)) {
        stateMachine.state = stateMachine.sUnpoweredAccent;
        stateMachine.burnoutTime = cycle.millis;

        if (airBrakes.useStaticAngle) {
            airBrakes.setPWM(airBrakes.pwmTarget);
        } else {
            airBrakes.setAngle(airBrakes.startingAngle);
        }

        airBrakes.lastUpdate = cycle.millis;
        airBrakes.cumulativeI = 0; // IDK if this is totally needed, but the consequences for not having it are too high to not have it
    } else if ((stateMachine.state == stateMachine.sUnpoweredAccent) && ((cycle.millis - stateMachine.burnoutTime) >= 3500)) {
        stateMachine.state = stateMachine.sParachuteHold;
        stateMachine.apogeeTime = cycle.millis;

        airBrakes.setPWM(airBrakes.min);
    } else if ((stateMachine.state == stateMachine.sParachuteHold) && ((cycle.millis - stateMachine.apogeeTime) >= 3000)) {
        stateMachine.state = stateMachine.sDecent;
        stateMachine.parachuteTime = cycle.millis;

        airBrakes.setPWM(airBrakes.max);
    } else if ((stateMachine.state == stateMachine.sDecent) && ((cycle.alt - BARO.initAlt) <= 3.05)) {
        stateMachine.state = stateMachine.sLandingHold;

        airBrakes.setPWM(airBrakes.min);
    } else if ((stateMachine.state == stateMachine.sLandingHold) && ((cycle.alt - BARO.initAlt) <= 0.5)) {
        stateMachine.state = stateMachine.sLanded;
        stateMachine.touchdownTime = cycle.millis;
    } else if (stateMachine.state == stateMachine.sLanded) {
        // Nothing to do
    } else if (stateMachine.state == stateMachine.sSDError) { // Errors
        // Try to re-open the file, or start writing to flash memory
    } else if (stateMachine.state == stateMachine.sIMUError) {
        // Attempt to restart IMU
    } else if (stateMachine.state == stateMachine.sBAROError) {
        // Attempt to restart BARO
    } else if (stateMachine.state == stateMachine.sLowBatteryVoltage) {
        // Nothing to do
    }

    // Estimate velocity
    if (velocityEst.lastVarsSetup) { // We have lastUpdate and lastZAccel and can estimate our velocity
        float deltaTime = cycle.millis - velocityEst.lastUpdate; // Change in time from when we last updated (in ms)

        velocityEst.inte += (0.5 * deltaTime * (velocityEst.lastZAccel + cycle.accel.z)) * 1000;
        velocityEst.diff = ((cycle.rawAlt - velocityEst.lastAlt) / deltaTime) * 1000;
        velocityEst.velocity = highPassFilter(0.98, velocityEst.inte) + lowPassFilter(0.03, velocityEst.diff);

        cycle.calculateRunningAvgOfDiffVelocity(velocityEst.diff);

        velocityEst.lastUpdate = cycle.millis;
        velocityEst.lastZAccel = cycle.accel.z;
        velocityEst.lastAlt = cycle.rawAlt;
    } else {
        velocityEst.lastUpdate = cycle.millis;
        velocityEst.lastZAccel = cycle.accel.z;
        velocityEst.lastAlt = cycle.rawAlt;

        velocityEst.lastVarsSetup = true;
    }

    // Record max altitude
    if (cycle.alt > BARO.maxAlt) {
        BARO.maxAlt = cycle.alt;
    }

    // Control the air brakes
    if (stateMachine.state == stateMachine.sUnpoweredAccent && !airBrakes.useStaticAngle) {
        airBrakes.error = apogeeEst.estApogee(cycle.alt, BARO.initAlt, cycle.diffVelocity, airBrakes.currentAngle, 0.0, airBrakes.maxAngle, cycle.press, cycle.BAROTemp) - meta.targetApogee;
        airBrakes.targetAngle = airBrakes.calculatePID(airBrakes.error, cycle.millis); // Target air brake angle (in deg)

        // Ensure we don't set the air brakes to an invalid angle
        if (airBrakes.targetAngle <= 0) {
            airBrakes.setPWM(airBrakes.min);
        } else if (airBrakes.targetAngle > airBrakes.maxAngle) {
            airBrakes.setPWM(airBrakes.max);
        } else {
            airBrakes.setAngle(airBrakes.targetAngle);
        }
    }

    // Update the LED based on the current state
    if ((cycle.millis - stateMachine.lastUpdate) >= stateMachine.updateFreq) {
        if (stateMachine.offThisUpdate) {
            stateMachineLED.setPixelColor(0, 0, 0, 0);
        } else {
            if (stateMachine.state == stateMachine.sBootUp) {
                // Error. We should never be in this state here
            } else if (stateMachine.state == stateMachine.sAwaitingArming) {
                stateMachineLED.setPixelColor(0, 255, 0, 255);
            } else if (stateMachine.state == stateMachine.sAwaitingLaunch) {
                stateMachineLED.setPixelColor(0, 255, 0, 0);
            } else if (stateMachine.inFlight()) {
                stateMachineLED.setPixelColor(0, 0, 255, 255);
            } else if (stateMachine.state == stateMachine.sLanded) {
                stateMachineLED.setPixelColor(0, 0, 0, 255);
            } else if (stateMachine.isErroring()) {
                if (stateMachine.state == stateMachine.sSDError) {
                    stateMachineLED.setPixelColor(0, 255, 255, 0);
                } else {
                    stateMachineLED.setPixelColor(0, 0, 255, 0);
                }
            }
        }

        stateMachineLED.show(); // Push the color to the LED
        stateMachine.lastUpdate = cycle.millis;
        stateMachine.offThisUpdate = !stateMachine.offThisUpdate;
    }

    // Play the buzzer heartbeat
    if (stateMachine.state == stateMachine.sAwaitingArming) {
        if (!buzzer.heartbeatPlaying && ((cycle.millis - buzzer.lastUpdate) >= buzzer.updateFreq)) {
            PWM.setPWM(buzzer.pin, 0, buzzer.heartbeatPWM);

            buzzer.lastUpdate = cycle.millis;
            buzzer.heartbeatPlaying = true;
        } else if (buzzer.heartbeatPlaying && ((cycle.millis - buzzer.lastUpdate) >= buzzer.heartbeatLength)) {
            PWM.setPWM(buzzer.pin, 0, 0);

            buzzer.lastUpdate = cycle.millis;
            buzzer.heartbeatPlaying = false;
        }
    }

    // Logging
    if (logging.file) {
        // General
        logging.flog(cycle.millis);
        logging.flog(stateMachine.state);
        logging.flog(cycle.millis - stateMachine.launchTime);
        logging.flog(cycle.millis - logging.lastLog);

        // IMU
        logging.flog(cycle.accel.x);
        logging.flog(cycle.accel.y);
        logging.flog(cycle.accel.z);

        logging.flog(cycle.mag.x);
        logging.flog(cycle.mag.y);
        logging.flog(cycle.mag.z);

        logging.flog(cycle.gyro.x);
        logging.flog(cycle.gyro.y);
        logging.flog(cycle.gyro.z);

        logging.flog(cycle.IMUTemp);

        // Raw IMU
        logging.flog(cycle.rawAccel.x);
        logging.flog(cycle.rawAccel.y);
        logging.flog(cycle.rawAccel.z);

        logging.flog(cycle.rawMag.x);
        logging.flog(cycle.rawMag.y);
        logging.flog(cycle.rawMag.z);

        logging.flog(cycle.rawGyro.x);
        logging.flog(cycle.rawGyro.y);
        logging.flog(cycle.rawGyro.z);

        logging.flog(cycle.rawIMUTemp);

        // BARO
        logging.flog(cycle.BAROTemp);
        logging.flog(cycle.press);
        logging.flog(cycle.alt);

        logging.flog(cycle.rawBAROTemp);
        logging.flog(cycle.rawPress);
        logging.flog(cycle.rawAlt);

        // Velocity est
        logging.flog(velocityEst.lastUpdate);
        logging.flog(velocityEst.lastZAccel);
        logging.flog(velocityEst.lastAlt);

        logging.flog(velocityEst.inte);
        logging.flog(velocityEst.diff);
        logging.flog(velocityEst.velocity);

        // ACS
        logging.flog(apogeeEst.lastEstApogee);
        logging.flog(airBrakes.targetAngle);
        logging.flog(airBrakes.currentAngle);
        logging.flog(airBrakes.error);

        // Other
        logging.flog(getBatteryVoltage());
        logging.flog(cycle.accel.z * (apogeeEst.rocketWetMass - apogeeEst.burnoutSubtrahend), false);
        logging.flog("\n", false);

        logging.lastLog = cycle.millis;

        if ((cycle.millis - logging.lastFlush) >= logging.flushFreq) {
            logging.file.flush();
            logging.lastFlush = cycle.millis;
        }
    }
}

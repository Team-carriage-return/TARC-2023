// ----------- //
/* Import Libs */
// ----------- //

#include <Adafruit_PWMServoDriver.h> // PWM v2.4.1

// ---- //
/* Vars */
// ---- //

int pins[4] = {4, 5, 6, 7}; // Pins of the air brakes
int offsets[4] = {0, +5, +20, +20}; // Air brake offsets (Offsets are added to the commanded value)

String debugCommand = ""; // Input via Serial
char debugCommandArr[128]; // debugCommand as a c-string
char debugCommandFirstWord[128]; // All characters of debugCommand up to the first present space (The space is excluded)
char buffer[128];

Adafruit_PWMServoDriver PWM = Adafruit_PWMServoDriver(); // PWM servo driver

// --------- //
/* Functions */
// --------- //

/** Splices a char[] at the indexes provided
 * @param buffer The buffer that will contain the splice
 * @param inputCharArray
 * @param startingIndex The index to start the splice (Inclusive). Must be less than endingIndex
 * @param endingIndex The index to stop the splice (Exclusive)
 */
char* spliceCharArray(char buffer[], char inputCharArray[], int startingIndex, int endingIndex) {
    for (int i = 0; i < sizeof(buffer); i++) { // Clear the buffer
        buffer[i] = '\0';
    }

    for (int i = 0; startingIndex + i < endingIndex; i++) {
        buffer[i] = inputCharArray[startingIndex + i];
    }

    return buffer;
}

/** Compares two char arrays
 * @param firstCharArray
 * @param secondCharArray
 * @returns bool
 */
bool compareCharArrays(char firstCharArray[], char secondCharArray[]) {
    int i = 0;
    char terminatingChar = '\0';

    do {
        if (firstCharArray[i] != secondCharArray[i]) {
            return false;
        }
    } while (firstCharArray[i++] != terminatingChar && secondCharArray[i] != terminatingChar);

    return true;
}

/** Extracts all characters from an input char[] up to the first space.
 * NOTE: DO NOT MOVE TO THE MAIN LOOP, it will break this functionality
 *
 * @param buffer A buffer to contain the first word
 * @param inputCharArray
 * @param bufferSize The result of sizeof() on buffer
 * @param inputCharSize The result of sizeof() on inputCharArray
 */
void extractFirstWord(char buffer[], char inputCharArray[], int bufferSize, int inputCharSize) {
    const char space = ' ';
    const char terminator = '\0';

    for (int i = 0; i < bufferSize; i++) { // Clear buffer
        buffer[i] = '\0';
    }

    for (int i = 0; i < inputCharSize; i++) { // Extract the first word of inputCharArray
        if (inputCharArray[i] == space || inputCharArray[i] == terminator) {
            return;
        } else {
            buffer[i] = inputCharArray[i];
        }
    }
}

// ---- //
/* Main */
// ---- //

/** Code put here is run once at startup */
void setup() {
    // Debugging
    Serial.begin(9600);
    Serial.setTimeout(0);

    // PWM
    PWM.begin();
    PWM.setPWMFreq(60);
}

/** Code put here runs once every cycle */
void loop() {
    if (Serial && Serial.available() > 0) {
        debugCommand = Serial.readString();
        debugCommand.trim();

        // Create debugging.commandArr and debugging.commandFirstWord
        debugCommand.toCharArray(debugCommandArr, sizeof(debugCommandArr));
        extractFirstWord(debugCommandFirstWord, debugCommandArr, sizeof(debugCommandFirstWord), sizeof(debugCommandArr));

        // Print out what the user gave us
        Serial.print("\n\n");
        Serial.println("> " + debugCommand);

        if (debugCommand == "HELP" || debugCommand == "help") {
            Serial.println("Air-brake Tuning Tool");
            Serial.println("");
            Serial.println("Available commands are shown below. For [v] use 4 digits, for [p] use 2.");
            Serial.println("");
            Serial.println("operations:");
            Serial.println("    ab-set [v]          set air-brakes to PWM [v], using saved offsets");
            Serial.println("    ab-set-raw [v]      set air-brakes to PWM [v], without using offsets");
            Serial.println("    pwm-set [p] [v]     set PWM device on pin [p] to [v] (does not check for offsets)");
        } else if (compareCharArrays("ab-set", debugCommandFirstWord)) {
            int value = atoi(spliceCharArray(buffer, debugCommandArr, 7, 11));

            for (int i = 0; i < 4; i++) {
                PWM.setPWM(pins[i], 0, value + offsets[i]);
            }

            sprintf(buffer, "Set air brakes to %i", value); Serial.println(buffer);
        } else if (compareCharArrays("ab-set-raw", debugCommandFirstWord)) {
            int value = atoi(spliceCharArray(buffer, debugCommandArr, 11, 15));

            for (int i = 0; i < 4; i++) {
                PWM.setPWM(pins[i], 0, value);
            }

            sprintf(buffer, "Set air brakes to %i", value); Serial.println(buffer);
        } else if (compareCharArrays("pwm-set", debugCommandFirstWord)) {
            int pin = atoi(spliceCharArray(buffer, debugCommandArr, 8, 10));
            int value = atoi(spliceCharArray(buffer, debugCommandArr, 11, 15));

            PWM.setPWM(pin, 0, value);
            sprintf(buffer, "Set PWM on pin %i to %i", pin, value); Serial.println(buffer);
        } else {
            Serial.println("Unknown command! Use HELP for help.");
        }
    }
}

// --------- //
/* Functions */
// --------- //

/** Finds initial altitude and sets up cycle with 5 variables */
void calibrationAndCycleSetup() {
    temp.counter = 0.0;

    // Initial altitude
    for (int i = 1; i <= 20; i++) {
        temp.counter += BARO.a.readAltitude(BARO.seaLevelPress);
        delay(50);
    }
    BARO.initAlt = temp.counter / 20;

    // Cycle setup
    for (int i = 1; i <= 5; i++) {
        cycle.millis = millis();

        IMU.getData();
        cycle.calculateRunningAvgs(
            IMU.lastAccelData,
            IMU.lastMagData,
            IMU.lastGyroData,
            IMU.lastTempData,
            BARO.a.readTemperature(),
            BARO.a.readPressure(),
            BARO.a.readAltitude(BARO.seaLevelPress)
        );

        delay(50);
    }
}

/** "Makes" sprintf support floats
 * @param buffer The buffer sprintf will use and will be returned
 * @param inputFloat
 * @returns sprintfBuffer containing one float in char[] form
 */
char* sprintfFloat(char buffer[], float inputFloat) {
    int wholeFromFloat = inputFloat;
    int decimalFromFloat = trunc((inputFloat - wholeFromFloat) * 100);

    sprintf(buffer, "%i.%i", wholeFromFloat, decimalFromFloat);
    return buffer;
}

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

/** Gets the battery voltage
 * @returns float
 */
float getBatteryVoltage() {
    float batteryVoltage = analogRead(A6);

    batteryVoltage *= 2; // We divided by 2, so multiply back
    batteryVoltage *= 3.3; // Multiply by 3.3V, our reference voltage
    batteryVoltage /= 1024; // Convert to voltage

    return batteryVoltage;
}

/** Applies a high pass filter to a value, given a time constant. Complementary with lowPassFilter
 * @param multiplier HPF multiplier(?)
 * @param x The number to apply the high pass filter to
 * @returns x run through the high pass filter
 */
float highPassFilter(float multiplier, float x) {
    return (multiplier * x) / (1 + (multiplier * x));
}

/** Applies a low pass filter to a value, given a time constant. Complementary with highPassFilter
 * @param multiplier LPF multiplier(?)
 * @param x The number to apply the low pass filter to
 * @returns x run through the low pass filter
 */
float lowPassFilter(float multiplier, float x) {
    return 1 / (1 + (multiplier * x));
}

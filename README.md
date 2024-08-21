<!-- PRIMARY VERSION REMINDER: The primary version of this file is kept in the notebook. -->

# Team \\r TARC 2023

[2023 TARC rules](https://rocketcontest.org/wp-content/uploads/FINAL-2023-American-Rocketry-Challenge-Rules-v5.16.22.pdf)

## Disclaimer and License

Rocketry is an inherently dangerous sport/hobby, both physically and in rare cases legally. Team \\r and its members are not responsible for damages or losses that may result from using this codebase or derivatives. Use at your own risk and fly responsibly.

Additionally, please note that some rocket active control systems (mainly TVC systems) are listed in the United States Munition List (USML; 22 CFR Part 121) and subject to export restrictions by International Traffic in Arms Regulations (ITAR). Out of an abundance of caution, please be careful when sharing this codebase or derivatives.

This repo is licensed under the MIT license:

```txt
Copyright (c) 2024 Team \r

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

## Files/Folders

+ `Flight-archive` - Flight data and reports about flights
    + `F(num).csv` contains flight data zoomed into the actual flight
    + `Flight-log.(log|csv)` contain raw flight data
    + `Flight-log.postflight` sometimes includes a manual description of the flight and sometimes includes all variables and their values after touchdown
+ `Happy-little-data-gap-fixer` - A tool to fix negative numbers being logged as "-12.-34" instead of "-12.34" in some early flights (this caused data gaps in our graphing software and we coined the term "happy little data gaps")
+ `TARC-air-brake-tuner` - An air brake tuning tool. IDK if this was ever used
+ `TARC-codebase` - The actual codebase
+ `Flash-code.sh` - A utility to help write code to the STM32F405

## Codebase

This project was my introduction to Arduino C++. As such, there are many things that I did then that I would do differently now. For example, we calculate the rolling average of a bunch of data we never use. No idea why I decided to do that lol.

If you have questions about the codebase feel free to reach out, though because the project is over bugs probably won't be fixed.

## Hardware

+ Micro controller: [Adafruit Feather STM32F405 Express](https://www.adafruit.com/product/4382)
+ IMU: [LSM9DS1](https://www.adafruit.com/product/4634)
+ BARO: [BMP390 ](https://www.adafruit.com/product/4816)
+ PWM board: [Zio 16 Servo Controller](https://www.sparkfun.com/products/retired/16773)

We planned to use additional hardware for the descent control system (DCS), but because no code was written, it's been omitted from this list.

## Development Steps

1. Use the Arduino IDE to export a compiled binary file
2. Bridge pins `B0` and a `3V3` pin and power cycle the STM32F405
3. Push the compiled binary using dfu-util. See `./Flash-code.sh` for an example command

I forget whether the Arduino compile settings were different from the default ones, or if it matters. In the event they're helpful, here you go:

| Setting         | Value                                   |
| --------------- | --------------------------------------- |
| Board           | Generic STM32F4 series                  |
| Debug symbols   | None                                    |
| Optimize        | Smallest (-Os default)                  |
| Board part num  | Adafruit Feather STM32F405              |
| C Runtime lib   | Newlib Nano (default)                   |
| Upload method   | BMP (Black Magic Probe)                 |
| USB support     | CDC (generic Serial superscede U(S)ART) |
| U(S)ART support | Enabled (generic Serial)                |
| USB speed       | Low/Full speed                          |

## Log Header Descriptions

### General

+ `millis()` - Time reported by millis()
+ `Current_state` - The current state number
+ `Calculated_flight_time` - millis() minus the detected launch time
+ `Time_since_last_log` - millis() minus the last time we logged a record

### IMU

X, Y, and Z logged here use our standard axes.

+ `IMU_accel_N` - N (X, Y, or Z) acceleration reported from the IMU
+ `IMU_mag_N` - N (X, Y, or Z) magnetometer data reported from the IMU
+ `IMU_gyro_N` - N (X, Y, or Z) gyroscope data reported from the IMU
+ `IMU_temp` - Temperature reported from the IMU

### BARO

+ `BARO_temp` - Temperature reported from the BARO in deg C
+ `BARO_press` - Pressure reported from the BARO
+ `BARO_alt` - Altitude reported from the BARO

### Orientation

+ `Orientation_yaw` - Actual yaw orientation
+ `Orientation_pitch` - Actual pitch orientation
+ `Orientation_roll` - Actual roll orientation

### Velocity Est

+ `Velocity_inte` - The integral of acceleration in the z axis
+ `Velocity_diff` - The derivative of the altitude
+ `Velocity_est` - The estimated velocity

### Apogee Est

+ `Est_apogee` - The output of the `apogeeEst.estApogee()` function

### PNUT

+ `PNUT_time` - Time in seconds since launch was detected
+ `PNUT_alt` - Altitude change from ground altitude
+ `PNUT_velocity` - Velocity
+ `PNUT_temp` - Temperature in F
+ `PNUT_voltge` - Voltage pull of the PNUT?
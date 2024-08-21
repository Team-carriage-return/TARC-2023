# This file contains the necessary code to flash the code to the feather.
# Compile the project using the Arduino IDE and then run this command

dfu-util -a 0 -w --dfuse-address 0x08000000 -D ./TARC-codebase/build/STMicroelectronics.stm32.GenF4/TARC-codebase.ino.bin

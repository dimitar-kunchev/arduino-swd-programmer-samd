Implementation of access to SAMx5 microcontroller from Arduino Zero over SWD

My goal is to deliver basic protocol communication over SWD. Final goal is to build a custom programmer with SAMD processor that reads from SD card, but that will be outside the scope of this repository.

Here the project will end with reading/wrigin fuses and erasing the chip.

Note that access to the DP/AP over SWD is actually very complicated and requires knowedge of the inner workings of the ARM core. I am piecing information from https://github.com/adafruit/Adafruit_DAP (which for some reason does not work for me) and the official ARM documentation.

Use this code as reference for development only. It is not intended to be easy to reuse!
Implementation of access to SAMx5 microcontroller from Arduino Zero over SWD

My goal is to deliver basic protocol communication over SWD. Final goal is to build a custom programmer with SAMD processor that reads from SD card, but that will be outside the scope of this repository.

Note that access to the DP/AP over SWD is actually very complicated and requires knowedge of the inner workings of the ARM core. I am piecing information from https://github.com/adafruit/Adafruit_DAP (which for some reason does not work for me) and the official ARM documentation.

Use this code as reference for development only. It is not intended to be easy to reuse!

What I have working:
 - Attaching to the target
 - Access to the internal registers (MEM-AP) which pretty much gives you full control
 - Reading, erasing, uploading firmware
 - Checksum calculation
 - Performing MBIST (not sure if it works)
 - Access to the User area of the flash

Tested with Arduino M0 Pro attached to a custom board with SAMD51J20A.

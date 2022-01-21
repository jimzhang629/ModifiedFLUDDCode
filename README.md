# FLOOD MSP432 Firmware
This firmware runs on the MSP432, the core control chip in the FLOOD water level measurement system. It handles reading data from attached sensors, saving data to an SD card, and transmitting data to a remote server

## Building
Code Composer is the preferred tool to build this project, but it also supports GCC if a more flexible build process is desired.
### Build requirements
- [Code Composer](https://www.ti.com/tool/CCSTUDIO), or [arm-none-eabi-gcc](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
- [MSP432 SimpleLink SDK](https://www.ti.com/tool/SIMPLELINK-MSP432-SDK)
- [SYSConfig](https://www.ti.com/tool/SYSCONFIG)

### Build Process
If using Code Composer, simply import the project and build it.

If using gcc, `cd` to the `gcc-build` directory, and edit the following variables within the `makefile` to match your install:
- `SIMPLELINK_MSP432_SDK_INSTALL_DIR`
- `GCC_ARM_COMPILER`
- `XDC_INSTALL_DIR`
- `SYSCONFIG_TOOL`

From there, the project should compile with `make`

### Flashing firmware
First, the MSP432 debugger must have the `TDI`, `TDO`, `TCK`, `TMS`, and `RST` pins connected to the target board, which must also have its `+3v3` line powered. `RXD` and `TXD` can be connected to the UART `UCA0` breakout pins if the serial cli is desired. (In this case, the debugger should also have a shared ground connected to the target board)

With Code Composer, the debugging can simply be launched at this point.

If using the gcc toolchain, the [openocd](http://openocd.org/) tool should have support for flashing the build file. Note that the build file created with gcc **CANNOT** be used without a debugger connected, due to the how semihosting is implemented in gcc.

For detailed building and flashing steps, see [here](documentation/Building.md)

## Radar Sampling Module
This module is responsible for powering on the radar daughterboard, and waiting for it to produce distance samples. It also applies an offset to the samples to calculate the water level using the raw distance data

## Storage Modules
This module handles all interfacing with the attached SD card. It stores all recorded water level data, along with timestamps, into a CSV file on the SD card. It also handles logging system events to the SD card, as well as reading the configuration parameters stored within the `config.txt` file on the SD card.

## Transmission Module
This module supports interfacing with the attached SIM7000 daughterboard. It supports setting the system's real time clock using the time synchronized from a remote NTP timesever. It also handles transmission of timestamped water level data to the backend hosted on AWS.

## Debug CLI
This firmware includes a debug CLI, accessible over a serial port. It provides a help menu, and allows for several useful debugging tasks, such as remounting the SD card and loading new system configurations. See [here](documentation/CLI.md) for a list of commands.


## Additional Documentation
This project is documented with Doxygen. Documentation can be created in the `generated-documentation` folder using the command `doxygen Doxyfile`.

Alternatively, here are direct links to the Markdown documents for this project (stored in the `documentation` folder):
- [Building and Flashing Guide](documentation/Building.md)
- [Command Line](documentation/CLI.md)
- [Firmware Concepts](documentation/Firmware-Development.md)
- [Main System Task](documentation/Main-Task.md)
- [Radar Module](documentation/Radar.md)
- [SIM7000 LTE Module Library](documentation/SIM7000.md)
- [Storage Module](documentation/Storage.md)
- [System-Introduction](documentation/System-Introduction.md)
- [Transmission Module](documentation/Transmission.md)
- [Links to External Documentation](documentation/Further-Reading.md)
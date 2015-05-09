# The wongboard keyboard

## Current status

- Started project (blink a light). That's about half way there right?

STM32 based keyboard firmware/hardware for STM32F0x2 processors (although any STM32 with usb should work too). This is my first project using the STM32 series of MCU's.

This is a basic project / template project for the NUCLEO-F072RB board which has a STM32F072RB MCU onboard. The code is mostly auto generated from stm32cube, however I have created a batch file (...should really be a make file) which allows compilation on windows with only the 'arm-none-eabi-gcc' gcc package installed. This may be useful if you're trying to figure out what compile commands/what includes are necessary to get a basic build going. 

The project uses the HAL library, although a version without HAL is coming soon.

If you are using a different MCU, you should regenerate the 'Drivers' folder by making a new project using STM32Cube. Make sure to select 'SW4STM' to get the correct .ld (linker config) files.

Todo: 
- make proper readme file
- get some sort of debugging working


Useful links:
- STM32 HAL Usb library documentation http://www.st.com/st-web-ui/static/active/en/resource/technical/document/user_manual/DM00108129.pdf
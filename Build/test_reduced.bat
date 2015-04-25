:: Most of this configuration is pulled from the "System Workbench for STM32" makefile, atlhough i didn't know it was in there initially and tried to work it out from someone's blogpost on the same topic. 
cls

del *.o *.elf *.bin

set GCC_LOC="C:\Program Files (x86)\GNU Tools ARM Embedded\4.9 2015q1\bin\arm-none-eabi-gcc.exe"
set SIZE_LOC="C:\Program Files (x86)\GNU Tools ARM Embedded\4.9 2015q1\bin\arm-none-eabi-size.exe" 
set OBJCOPY_LOC="C:\Program Files (x86)\GNU Tools ARM Embedded\4.9 2015q1\bin\arm-none-eabi-objcopy.exe"

set HAL_INC="..\Drivers\STM32F0xx_HAL_Driver\Inc"
set CMSIS_INC="..\Drivers\CMSIS\Include"
set DEVICE_INC="..\Drivers\CMSIS\Device\ST\STM32F0xx\Include"
set STM32CUBE_INC="..\Inc"

set LD_PATH="..\SW4STM32\wongboard Configuration\STM32F072RBTx_FLASH.ld"
set STARTUP_ASM_PATH="..\Drivers\CMSIS\Device\ST\STM32F0xx\Source\Templates\gcc\startup_stm32f072xb.s"

:: make Cflags
set INCLUDE_ARGS=-I%CMSIS_INC% -I%HAL_INC% -I%STM32CUBE_INC% -I%DEVICE_INC%
set CFLAGS= -DUSE_HAL_DRIVER -DSTM32F072xB -mcpu=cortex-m0 -mthumb -Wall %INCLUDE_ARGS% -ffunction-sections -fdata-sections

:: build HAL library
%GCC_LOC% %CFLAGS% -c "..\Drivers\STM32F0xx_HAL_Driver\Src\*.c" 

:: build system_stm32[...].c and startup[...].s (startup is assembly which defines startup vectors, I am not sure what system_stm32 defines (refer to DM00105879.pdf) 
%GCC_LOC% %CFLAGS% -c "..\Drivers\CMSIS\Device\ST\STM32F0xx\Source\Templates\system_stm32f0xx.c" %STARTUP_ASM_PATH%

:: buld user files
%GCC_LOC% %CFLAGS% -c "..\Src\*.c"  

:: link everything. -T%LD_PATH% includes the linker config file.
:: http://stackoverflow.com/questions/19419782/exit-c-text0x18-undefined-reference-to-exit-when-using-arm-none-eabi-gcc
:: use '--specs=nosys.specs' instead?
:: -Wl,--gc-sections remove unused code to prevent error/reduce binary size. See https://gcc.gnu.org/onlinedocs/gnat_ugn/Compilation-options.html 
:: -Wl,[option] passes options to the linker, so --gc-sections means garbage collect unused sections. supposed to compile with first '-ffunction-sections' '-fdata-sections'
%GCC_LOC% -DSTM32F072xB -mcpu=cortex-m0 -mthumb -Wl,--gc-sections -T%LD_PATH% *.o -o main.elf -Wall

:: Print out size and convert .elf to .bin
%SIZE_LOC% main.elf

%OBJCOPY_LOC% -O binary main.elf main.bin

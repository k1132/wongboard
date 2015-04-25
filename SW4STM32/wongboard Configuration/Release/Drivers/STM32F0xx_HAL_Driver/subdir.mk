################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c \
C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c \
C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c \
C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c \
C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c \
C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c \
C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c \
C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c \
C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c \
C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c 

OBJS += \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_cortex.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_dma.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash_ex.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_gpio.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr_ex.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc_ex.o 

C_DEPS += \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_cortex.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_dma.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash_ex.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_gpio.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr_ex.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal.o: C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal.d" -MT"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_cortex.o: C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_cortex.d" -MT"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_cortex.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_dma.o: C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_dma.d" -MT"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_dma.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash.o: C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash.d" -MT"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash_ex.o: C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash_ex.d" -MT"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash_ex.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_gpio.o: C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_gpio.d" -MT"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_gpio.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr.o: C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr.d" -MT"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr_ex.o: C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr_ex.d" -MT"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr_ex.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc.o: C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc.d" -MT"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc_ex.o: C:/temp/stm32/wongboard/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc_ex.d" -MT"Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc_ex.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/drojf/Dropbox/current\ programming/STM32/wongboard/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32f0xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32f0xx.o: C:/drojf/Dropbox/current\ programming/STM32/wongboard/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Drivers/CMSIS/system_stm32f0xx.d" -MT"Drivers/CMSIS/system_stm32f0xx.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



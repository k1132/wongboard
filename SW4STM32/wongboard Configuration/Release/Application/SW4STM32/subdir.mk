################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/temp/stm32/wongboard/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f072xb.s 

OBJS += \
./Application/SW4STM32/startup_stm32f072xb.o 


# Each subdirectory must supply rules for building sources it contributes
Application/SW4STM32/startup_stm32f072xb.o: C:/temp/stm32/wongboard/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f072xb.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo %cd%
	arm-none-eabi-as -mcpu=cortex-m0 -mthumb -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



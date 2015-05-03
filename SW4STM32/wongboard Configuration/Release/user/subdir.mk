################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/drojf/Dropbox/current\ programming/STM32/wongboard/SW4STM32/syscalls.c 

OBJS += \
./user/syscalls.o 

C_DEPS += \
./user/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
user/syscalls.o: C:/drojf/Dropbox/current\ programming/STM32/wongboard/SW4STM32/syscalls.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"user/syscalls.d" -MT"user/syscalls.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



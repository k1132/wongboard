################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/drojf/Dropbox/current\ programming/STM32/wongboard/Src/main.c \
C:/drojf/Dropbox/current\ programming/STM32/wongboard/Src/stm32f0xx_hal_msp.c \
C:/drojf/Dropbox/current\ programming/STM32/wongboard/Src/stm32f0xx_it.c 

OBJS += \
./Application/User/main.o \
./Application/User/stm32f0xx_hal_msp.o \
./Application/User/stm32f0xx_it.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/stm32f0xx_hal_msp.d \
./Application/User/stm32f0xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: C:/drojf/Dropbox/current\ programming/STM32/wongboard/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Application/User/main.d" -MT"Application/User/main.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f0xx_hal_msp.o: C:/drojf/Dropbox/current\ programming/STM32/wongboard/Src/stm32f0xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Application/User/stm32f0xx_hal_msp.d" -MT"Application/User/stm32f0xx_hal_msp.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f0xx_it.o: C:/drojf/Dropbox/current\ programming/STM32/wongboard/Src/stm32f0xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -DUSE_HAL_DRIVER -DSTM32F072xB -I../../../Inc -I../../../Drivers/STM32F0xx_HAL_Driver/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F0xx/Include -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"Application/User/stm32f0xx_it.d" -MT"Application/User/stm32f0xx_it.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



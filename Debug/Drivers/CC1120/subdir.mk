################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CC1120/cc1120_hal_spi.c \
../Drivers/CC1120/cc1120_hal_txInit.c 

OBJS += \
./Drivers/CC1120/cc1120_hal_spi.o \
./Drivers/CC1120/cc1120_hal_txInit.o 

C_DEPS += \
./Drivers/CC1120/cc1120_hal_spi.d \
./Drivers/CC1120/cc1120_hal_txInit.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CC1120/%.o: ../Drivers/CC1120/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F401xE -I"/Users/victorsira/orbit/workspace/SandBox/Inc" -I"/Users/victorsira/orbit/workspace/SandBox/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/Users/victorsira/orbit/workspace/SandBox/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/Users/victorsira/orbit/workspace/SandBox/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/victorsira/orbit/workspace/SandBox/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/bmp280/bmp280.c 

OBJS += \
./Drivers/bmp280/bmp280.o 

C_DEPS += \
./Drivers/bmp280/bmp280.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/bmp280/bmp280.o: ../Drivers/bmp280/bmp280.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F373xC -DDEBUG -c -I"D:/STM32_WORKDIR/Temperature_Controller-SW/Drivers/u8g2/csrc" -I../Core/Inc -I"D:/STM32_WORKDIR/Temperature_Controller-SW/Drivers/bmp280" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/bmp280/bmp280.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


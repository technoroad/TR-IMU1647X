################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sensors/src/adis16470.c \
../sensors/src/adis16475.c \
../sensors/src/adis16477.c 

OBJS += \
./sensors/src/adis16470.o \
./sensors/src/adis16475.o \
./sensors/src/adis16477.o 

C_DEPS += \
./sensors/src/adis16470.d \
./sensors/src/adis16475.d \
./sensors/src/adis16477.d 


# Each subdirectory must supply rules for building sources it contributes
sensors/src/%.o sensors/src/%.su: ../sensors/src/%.c sensors/src/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m0 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F070xB -c -I../Inc -I../lib/inc -I../sensors/inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-sensors-2f-src

clean-sensors-2f-src:
	-$(RM) ./sensors/src/adis16470.d ./sensors/src/adis16470.o ./sensors/src/adis16470.su ./sensors/src/adis16475.d ./sensors/src/adis16475.o ./sensors/src/adis16475.su ./sensors/src/adis16477.d ./sensors/src/adis16477.o ./sensors/src/adis16477.su

.PHONY: clean-sensors-2f-src


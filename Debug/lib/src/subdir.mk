################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/src/BinaryCmdParse.c \
../lib/src/USART_CTRL.c \
../lib/src/delay.c \
../lib/src/flash_wr.c \
../lib/src/mkAE_functions.c \
../lib/src/spi.c \
../lib/src/str_edit.c \
../lib/src/usb_uart_select.c \
../lib/src/usbprint.c 

OBJS += \
./lib/src/BinaryCmdParse.o \
./lib/src/USART_CTRL.o \
./lib/src/delay.o \
./lib/src/flash_wr.o \
./lib/src/mkAE_functions.o \
./lib/src/spi.o \
./lib/src/str_edit.o \
./lib/src/usb_uart_select.o \
./lib/src/usbprint.o 

C_DEPS += \
./lib/src/BinaryCmdParse.d \
./lib/src/USART_CTRL.d \
./lib/src/delay.d \
./lib/src/flash_wr.d \
./lib/src/mkAE_functions.d \
./lib/src/spi.d \
./lib/src/str_edit.d \
./lib/src/usb_uart_select.d \
./lib/src/usbprint.d 


# Each subdirectory must supply rules for building sources it contributes
lib/src/%.o lib/src/%.su: ../lib/src/%.c lib/src/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m0 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F070xB -c -I../Inc -I../lib/inc -I../sensors/inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-lib-2f-src

clean-lib-2f-src:
	-$(RM) ./lib/src/BinaryCmdParse.d ./lib/src/BinaryCmdParse.o ./lib/src/BinaryCmdParse.su ./lib/src/USART_CTRL.d ./lib/src/USART_CTRL.o ./lib/src/USART_CTRL.su ./lib/src/delay.d ./lib/src/delay.o ./lib/src/delay.su ./lib/src/flash_wr.d ./lib/src/flash_wr.o ./lib/src/flash_wr.su ./lib/src/mkAE_functions.d ./lib/src/mkAE_functions.o ./lib/src/mkAE_functions.su ./lib/src/spi.d ./lib/src/spi.o ./lib/src/spi.su ./lib/src/str_edit.d ./lib/src/str_edit.o ./lib/src/str_edit.su ./lib/src/usb_uart_select.d ./lib/src/usb_uart_select.o ./lib/src/usb_uart_select.su ./lib/src/usbprint.d ./lib/src/usbprint.o ./lib/src/usbprint.su

.PHONY: clean-lib-2f-src


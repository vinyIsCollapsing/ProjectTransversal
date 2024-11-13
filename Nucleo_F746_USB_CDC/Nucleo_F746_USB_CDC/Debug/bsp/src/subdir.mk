################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/src/bsp.c \
../bsp/src/delay.c \
../bsp/src/usb.c 

OBJS += \
./bsp/src/bsp.o \
./bsp/src/delay.o \
./bsp/src/usb.o 

C_DEPS += \
./bsp/src/bsp.d \
./bsp/src/delay.d \
./bsp/src/usb.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/src/%.o bsp/src/%.su bsp/src/%.cyclo: ../bsp/src/%.c bsp/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F746xx -c -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/app/inc" -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/bsp/inc" -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/cmsis/core" -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/cmsis/device/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bsp-2f-src

clean-bsp-2f-src:
	-$(RM) ./bsp/src/bsp.cyclo ./bsp/src/bsp.d ./bsp/src/bsp.o ./bsp/src/bsp.su ./bsp/src/delay.cyclo ./bsp/src/delay.d ./bsp/src/delay.o ./bsp/src/delay.su ./bsp/src/usb.cyclo ./bsp/src/usb.d ./bsp/src/usb.o ./bsp/src/usb.su

.PHONY: clean-bsp-2f-src


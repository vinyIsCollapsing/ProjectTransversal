################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../app/src/main.c \
../app/src/printf-stdarg.c \
../app/src/stm32f7xx_it.c \
../app/src/syscalls.c \
../app/src/sysmem.c 

OBJS += \
./app/src/main.o \
./app/src/printf-stdarg.o \
./app/src/stm32f7xx_it.o \
./app/src/syscalls.o \
./app/src/sysmem.o 

C_DEPS += \
./app/src/main.d \
./app/src/printf-stdarg.d \
./app/src/stm32f7xx_it.d \
./app/src/syscalls.d \
./app/src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
app/src/%.o app/src/%.su app/src/%.cyclo: ../app/src/%.c app/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F746xx -c -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/app/inc" -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/bsp/inc" -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/cmsis/core" -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/cmsis/device/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-app-2f-src

clean-app-2f-src:
	-$(RM) ./app/src/main.cyclo ./app/src/main.d ./app/src/main.o ./app/src/main.su ./app/src/printf-stdarg.cyclo ./app/src/printf-stdarg.d ./app/src/printf-stdarg.o ./app/src/printf-stdarg.su ./app/src/stm32f7xx_it.cyclo ./app/src/stm32f7xx_it.d ./app/src/stm32f7xx_it.o ./app/src/stm32f7xx_it.su ./app/src/syscalls.cyclo ./app/src/syscalls.d ./app/src/syscalls.o ./app/src/syscalls.su ./app/src/sysmem.cyclo ./app/src/sysmem.d ./app/src/sysmem.o ./app/src/sysmem.su

.PHONY: clean-app-2f-src


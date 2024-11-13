################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../cmsis/device/src/startup_stm32f746xx.s 

C_SRCS += \
../cmsis/device/src/system_stm32f7xx.c 

OBJS += \
./cmsis/device/src/startup_stm32f746xx.o \
./cmsis/device/src/system_stm32f7xx.o 

S_DEPS += \
./cmsis/device/src/startup_stm32f746xx.d 

C_DEPS += \
./cmsis/device/src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
cmsis/device/src/%.o: ../cmsis/device/src/%.s cmsis/device/src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
cmsis/device/src/%.o cmsis/device/src/%.su cmsis/device/src/%.cyclo: ../cmsis/device/src/%.c cmsis/device/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F746xx -c -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/app/inc" -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/bsp/inc" -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/cmsis/core" -I"C:/workspaces/workspace/Nucleo_F746_USB_CDC/cmsis/device/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-cmsis-2f-device-2f-src

clean-cmsis-2f-device-2f-src:
	-$(RM) ./cmsis/device/src/startup_stm32f746xx.d ./cmsis/device/src/startup_stm32f746xx.o ./cmsis/device/src/system_stm32f7xx.cyclo ./cmsis/device/src/system_stm32f7xx.d ./cmsis/device/src/system_stm32f7xx.o ./cmsis/device/src/system_stm32f7xx.su

.PHONY: clean-cmsis-2f-device-2f-src


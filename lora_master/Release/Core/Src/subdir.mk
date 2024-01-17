################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/SX1278.c \
../Core/Src/SX1278_hw.c \
../Core/Src/eeprom.c \
../Core/Src/led.c \
../Core/Src/lm75.c \
../Core/Src/main.c \
../Core/Src/module.c \
../Core/Src/rs485.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/uart1.c 

OBJS += \
./Core/Src/SX1278.o \
./Core/Src/SX1278_hw.o \
./Core/Src/eeprom.o \
./Core/Src/led.o \
./Core/Src/lm75.o \
./Core/Src/main.o \
./Core/Src/module.o \
./Core/Src/rs485.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/uart1.o 

C_DEPS += \
./Core/Src/SX1278.d \
./Core/Src/SX1278_hw.d \
./Core/Src/eeprom.d \
./Core/Src/led.d \
./Core/Src/lm75.d \
./Core/Src/main.d \
./Core/Src/module.d \
./Core/Src/rs485.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/uart1.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/SX1278.cyclo ./Core/Src/SX1278.d ./Core/Src/SX1278.o ./Core/Src/SX1278.su ./Core/Src/SX1278_hw.cyclo ./Core/Src/SX1278_hw.d ./Core/Src/SX1278_hw.o ./Core/Src/SX1278_hw.su ./Core/Src/eeprom.cyclo ./Core/Src/eeprom.d ./Core/Src/eeprom.o ./Core/Src/eeprom.su ./Core/Src/led.cyclo ./Core/Src/led.d ./Core/Src/led.o ./Core/Src/led.su ./Core/Src/lm75.cyclo ./Core/Src/lm75.d ./Core/Src/lm75.o ./Core/Src/lm75.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/module.cyclo ./Core/Src/module.d ./Core/Src/module.o ./Core/Src/module.su ./Core/Src/rs485.cyclo ./Core/Src/rs485.d ./Core/Src/rs485.o ./Core/Src/rs485.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/uart1.cyclo ./Core/Src/uart1.d ./Core/Src/uart1.o ./Core/Src/uart1.su

.PHONY: clean-Core-2f-Src


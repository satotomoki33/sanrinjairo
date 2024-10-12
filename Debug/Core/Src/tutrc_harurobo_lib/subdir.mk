################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/tutrc_harurobo_lib/can.cpp \
../Core/Src/tutrc_harurobo_lib/gpio.cpp \
../Core/Src/tutrc_harurobo_lib/timer.cpp 

OBJS += \
./Core/Src/tutrc_harurobo_lib/can.o \
./Core/Src/tutrc_harurobo_lib/gpio.o \
./Core/Src/tutrc_harurobo_lib/timer.o 

CPP_DEPS += \
./Core/Src/tutrc_harurobo_lib/can.d \
./Core/Src/tutrc_harurobo_lib/gpio.d \
./Core/Src/tutrc_harurobo_lib/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/tutrc_harurobo_lib/%.o Core/Src/tutrc_harurobo_lib/%.su Core/Src/tutrc_harurobo_lib/%.cyclo: ../Core/Src/tutrc_harurobo_lib/%.cpp Core/Src/tutrc_harurobo_lib/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-tutrc_harurobo_lib

clean-Core-2f-Src-2f-tutrc_harurobo_lib:
	-$(RM) ./Core/Src/tutrc_harurobo_lib/can.cyclo ./Core/Src/tutrc_harurobo_lib/can.d ./Core/Src/tutrc_harurobo_lib/can.o ./Core/Src/tutrc_harurobo_lib/can.su ./Core/Src/tutrc_harurobo_lib/gpio.cyclo ./Core/Src/tutrc_harurobo_lib/gpio.d ./Core/Src/tutrc_harurobo_lib/gpio.o ./Core/Src/tutrc_harurobo_lib/gpio.su ./Core/Src/tutrc_harurobo_lib/timer.cyclo ./Core/Src/tutrc_harurobo_lib/timer.d ./Core/Src/tutrc_harurobo_lib/timer.o ./Core/Src/tutrc_harurobo_lib/timer.su

.PHONY: clean-Core-2f-Src-2f-tutrc_harurobo_lib


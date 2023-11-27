################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/printf/test/test_suite.cpp 

OBJS += \
./Core/Src/printf/test/test_suite.o 

CPP_DEPS += \
./Core/Src/printf/test/test_suite.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/printf/test/%.o Core/Src/printf/test/%.su: ../Core/Src/printf/test/%.cpp Core/Src/printf/test/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-printf-2f-test

clean-Core-2f-Src-2f-printf-2f-test:
	-$(RM) ./Core/Src/printf/test/test_suite.d ./Core/Src/printf/test/test_suite.o ./Core/Src/printf/test/test_suite.su

.PHONY: clean-Core-2f-Src-2f-printf-2f-test


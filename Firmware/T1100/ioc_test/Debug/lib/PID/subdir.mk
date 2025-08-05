################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/PID/PID.c 

OBJS += \
./lib/PID/PID.o 

C_DEPS += \
./lib/PID/PID.d 


# Each subdirectory must supply rules for building sources it contributes
lib/PID/%.o lib/PID/%.su lib/PID/%.cyclo: ../lib/PID/%.c lib/PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I"/Users/michaelshipman/Developer/Terminator/Firmware/T1100/ioc_test/lib/SX1262" -I"/Users/michaelshipman/Developer/Terminator/Firmware/T1100/ioc_test/lib/PID" -I"/Users/michaelshipman/Developer/Terminator/Firmware/T1100/ioc_test/lib/t1100_lib" -I"/Users/michaelshipman/Developer/Terminator/Firmware/T1100/ioc_test/lib/ms5607" -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-lib-2f-PID

clean-lib-2f-PID:
	-$(RM) ./lib/PID/PID.cyclo ./lib/PID/PID.d ./lib/PID/PID.o ./lib/PID/PID.su

.PHONY: clean-lib-2f-PID


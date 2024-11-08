# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/PID/PID.c

OBJS += \
./lib/PID/PID.o

C_DEPS += \
./lib/PID/PID.d

# Each subdirectory must supply rules for building sources it contributes
lib/PID/%.o lib/PID/%.su lib/PID/%.cyclo: ../lib/PID/%.c lib/PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DMSI_VALUE=4000000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=0 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -DSTM32L412xx -c $(INCLUDES) -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-PID

clean-lib-2f-PID:
	-$(RM) 	./lib/FRAM_lib/PID.cyclo ./lib/FRAM_lib/PID.d ./lib/FRAM_lib/PID.o ./lib/FRAM_lib/PID.su

.PHONY: clean-lib-2f-PID

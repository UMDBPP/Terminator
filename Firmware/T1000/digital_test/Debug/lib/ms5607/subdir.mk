# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/ms5607/MS5607.c

OBJS += \
./lib/ms5607/MS5607.o

C_DEPS += \
./lib/ms5607/MS5607.d

# Each subdirectory must supply rules for building sources it contributes
lib/ms5607/%.o lib/ms5607/%.su lib/ms5607/%.cyclo: ../lib/ms5607/%.c lib/ms5607/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DMSI_VALUE=4000000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=0 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -DSTM32L422xx -c $(INCLUDES) -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-ms5607

clean-lib-2f-ms5607:
	-$(RM) 	./lib/ms5607/ms5607.cyclo ./lib/ms5607/MS5607.d ./lib/ms5607/MS5607.o ./lib/ms5607/MS5607.su

.PHONY: clean-lib-2f-ms5607

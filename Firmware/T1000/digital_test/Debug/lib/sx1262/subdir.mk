# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/sx1262/SX1262.c

OBJS += \
./lib/sx1262/SX1262.o

C_DEPS += \
./lib/sx1262/SX1262.d

# Each subdirectory must supply rules for building sources it contributes
lib/sx1262/%.o lib/sx1262/%.su lib/sx1262/%.cyclo: ../lib/sx1262/%.c lib/sx1262/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DMSI_VALUE=4000000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=0 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -DSTM32L422xx -c $(INCLUDES) -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-sx1262

clean-lib-2f-sx1262:
	-$(RM) 	./lib/sx1262/SX1262.cyclo ./lib/sx1262/SX1262.d ./lib/sx1262/SX1262.o ./lib/sx1262/SX1262.su

.PHONY: clean-lib-2f-sx1262

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/FRAM_lib/fram.c \
../lib/FRAM_lib/fs_util.c

OBJS += \
./lib/FRAM_lib/fram.o \
./lib/FRAM_lib/fs_util.o

C_DEPS += \
./lib/FRAM_lib/fram.d \
./lib/FRAM_lib/fs_util.d


# Each subdirectory must supply rules for building sources it contributes
lib/FRAM_lib/%.o lib/FRAM_lib/%.su lib/FRAM_lib/%.cyclo: ../lib/FRAM_lib/%.c lib/FRAM_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DMSI_VALUE=4000000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=0 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -DSTM32L422xx -c $(INCLUDES) -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-FRAM_lib

clean-lib-2f-FRAM_lib:
	-$(RM) 	./lib/FRAM_lib/fram.cyclo ./lib/FRAM_lib/fram.d ./lib/FRAM_lib/fram.o ./lib/FRAM_lib/fram.su \
			./lib/FRAM_lib/fs_util.cyclo ./lib/FRAM_lib/fs_util.d ./lib/FRAM_lib/fs_util.o ./lib/FRAM_lib/fs_util.su 

.PHONY: clean-lib-2f-FRAM_lib

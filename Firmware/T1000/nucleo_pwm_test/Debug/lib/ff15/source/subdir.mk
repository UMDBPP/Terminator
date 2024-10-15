# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/ff15/source/ff.c \
../lib/ff15/source/ffsystem.c \
../lib/ff15/source/ffunicode.c


OBJS += \
./lib/ff15/source/ff.o \
./lib/ff15/source/ffsystem.o \
./lib/ff15/source/ffunicode.o

C_DEPS += \
./lib/ff15/source/ff.d \
./lib/ff15/source/ffsystem.d \
./lib/ff15/source/ffunicode.d


# Each subdirectory must supply rules for building sources it contributes
lib/ff15/source/%.o lib/ff15/source/%.su lib/ff15/source/%.cyclo: ../lib/ff15/source/%.c lib/ff15/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DMSI_VALUE=4000000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=0 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -DSTM32L412xx -c $(INCLUDES) -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-ff15

clean-lib-2f-ff15:
	-$(RM) 	./lib/ff15/source/ff.cyclo \
			./lib/ff15/source/ffsystem.cyclo \
			./lib/ff15/source/ffunicode.cyclo \
			./lib/ff15/source/ff.o \
			./lib/ff15/source/ffsystem.o \
			./lib/ff15/source/ffunicode.o \
			./lib/ff15/source/ff.d \
			./lib/ff15/source/ffsystem.d \
			./lib/ff15/source/ffunicode.d \
			./lib/ff15/source/ff.su \
			./lib/ff15/source/ffsystem.su \
			./lib/ff15/source/ffunicode.su

.PHONY: clean-lib-2f-ff15

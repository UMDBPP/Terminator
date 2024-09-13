CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m4 --specs=nosys.specs -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
CPPFLAGS=-DSTM32L412xx \
	 -I../vendor/CMSIS/Device/ST/STM32L4/Include \
	 -I../vendor/CMSIS/CMSIS/Core/Include

LINKER_FILE=STM32L412RBTXP_FLASH.ld
LDFLAGS=-T $(LINKER_FILE)
STARTUP=../vendor/CMSIS/Device/ST/STM32L4/Source/Templates/gcc/startup_stm32l412xx.s
SYSTEM=../vendor/CMSIS/Device/ST/STM32L4/Source/Templates/system_stm32l4xx.c

all: test.elf

test.elf: main.o startup.o system_stm32l4xx.o syscalls.o sysmem.o
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $^ -o test.elf

main.o: ../src/main.c
	$(CC) $(CFLAGS) $(CPPFLAGS) ../src/main.c -c

syscalls.o: ../src/syscalls.c
	$(CC) $(CFLAGS) $(CPPFLAGS) ../src/syscalls.c -c

sysmem.o: ../src/sysmem.c
	$(CC) $(CFLAGS) $(CPPFLAGS) ../src/sysmem.c -c

startup.o: $(STARTUP)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(STARTUP) -c -o startup.o

system_stm32l4xx.o: $(SYSTEM)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(SYSTEM) -c

.PHONY: clean
clean:
	rm -f *.o *.elf
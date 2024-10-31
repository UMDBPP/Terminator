################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf


INCLUDES := \
	-I../Core/Inc \
	-I../Drivers/STM32L4xx_HAL_Driver/Inc \
	-I../Drivers/CMSIS/Device/ST/STM32L4xx/Include \
	-I../Drivers/CMSIS/Include \
	-I../lib/FRAM_Lib \
	-I../lib/littlefs \
	-I../lib/ms5607


# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk
-include lib/FRAM_lib/subdir.mk
-include lib/littlefs/subdir.mk
-include lib/ms5607/subdir.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := digital_cube_proj
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
digital_cube_proj.elf \

MAP_FILES += \
digital_cube_proj.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
digital_cube_proj.list \


# All Target
all: main-build

# Main-build Target
main-build: digital_cube_proj.elf secondary-outputs

# Tool invocations
digital_cube_proj.elf digital_cube_proj.map: $(OBJS) $(USER_OBJS) /Users/michaelshipman/Developer/Terminator/Firmware/T1000/digital_cube_proj/STM32L422KBUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "digital_cube_proj.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/michaelshipman/Developer/Terminator/Firmware/T1000/digital_cube_proj/STM32L422KBUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="digital_cube_proj.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

digital_cube_proj.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "digital_cube_proj.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout digital_cube_proj.elf digital_cube_proj.list digital_cube_proj.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets

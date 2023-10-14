################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include screen/subdir.mk
-include rtc/subdir.mk
-include io/subdir.mk
-include _W5500/subdir.mk
-include UnitTest/subdir.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Modbus_tcp/src/subdir.mk
-include Modbus_tcp/subdir.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include LCD/Src/subdir.mk
-include IO_Driver/Internet/DHCP/Src/subdir.mk
-include IO_Driver/Ethernet/Src/subdir.mk
-include FlashMemory/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

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


BUILD_ARTIFACT_NAME := Firmware_IO_inputCapture
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Firmware_IO_inputCapture.elf \

MAP_FILES += \
Firmware_IO_inputCapture.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Firmware_IO_inputCapture.list \

OBJCOPY_HEX += \
Firmware_IO_inputCapture.hex \

OBJCOPY_BIN += \
Firmware_IO_inputCapture.bin \


# All Target
all: main-build

# Main-build Target
main-build: Firmware_IO_inputCapture.elf secondary-outputs

# Tool invocations
Firmware_IO_inputCapture.elf Firmware_IO_inputCapture.map: $(OBJS) $(USER_OBJS) C:\Users\Admin\Desktop\STM32-release\Firmware_IO_inputCapture\STM32F072VBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Firmware_IO_inputCapture.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\Admin\Desktop\STM32-release\Firmware_IO_inputCapture\STM32F072VBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Firmware_IO_inputCapture.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Firmware_IO_inputCapture.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Firmware_IO_inputCapture.list"
	@echo 'Finished building: $@'
	@echo ' '

Firmware_IO_inputCapture.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Firmware_IO_inputCapture.hex"
	@echo 'Finished building: $@'
	@echo ' '

Firmware_IO_inputCapture.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Firmware_IO_inputCapture.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Firmware_IO_inputCapture.bin Firmware_IO_inputCapture.elf Firmware_IO_inputCapture.hex Firmware_IO_inputCapture.list Firmware_IO_inputCapture.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets

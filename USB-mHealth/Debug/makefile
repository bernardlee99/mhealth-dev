################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
mHealth_USB.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
mHealth_USB.list \

OBJCOPY_BIN += \
mHealth_USB.bin \


# All Target
all: mHealth_USB.elf secondary-outputs

# Tool invocations
mHealth_USB.elf: $(OBJS) $(USER_OBJS) C:\Users\SMR-Lab\ Computer\Desktop\Bernard\mhealth-dev\USB-mHealth\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "mHealth_USB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\SMR-Lab Computer\Desktop\Bernard\mhealth-dev\USB-mHealth\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="mHealth_USB.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

mHealth_USB.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "mHealth_USB.list"
	@echo 'Finished building: $@'
	@echo ' '

mHealth_USB.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "mHealth_USB.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

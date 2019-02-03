################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include protocol/serial/subdir.mk
-include protocol/firmata/subdir.mk
-include protocol/bluetooth/lib/EFR32BG1B/GCC/subdir.mk
-include protocol/balena/subdir.mk
-include platform/emlib/src/subdir.mk
-include platform/emdrv/ustimer/src/subdir.mk
-include platform/emdrv/uartdrv/src/subdir.mk
-include platform/emdrv/tempdrv/src/subdir.mk
-include platform/emdrv/sleep/src/subdir.mk
-include platform/emdrv/gpiointerrupt/src/subdir.mk
-include platform/emdrv/dmadrv/src/subdir.mk
-include platform/Device/SiliconLabs/EFR32BG1B/Source/GCC/subdir.mk
-include platform/Device/SiliconLabs/EFR32BG1B/Source/subdir.mk
-include hardware/kit/common/drivers/subdir.mk
-include hardware/kit/common/bsp/subdir.mk
-include app/bluetooth/common/util/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: balena-firmata.axf

# Tool invocations
balena-firmata.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C++ Linker'
	arm-none-eabi-g++ -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "/home/alex/SimplicityStudio/v4_workspace/balena-firmata/efr32bg1b232f256gm48.ld" --specs=nosys.specs -Xlinker --gc-sections -Xlinker -Map="balena-firmata.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o balena-firmata.axf "./app/bluetooth/common/util/infrastructure.o" "./application_properties.o" "./init_app.o" "./init_board_efr32xg1.o" "./init_mcu_efr32xg1.o" "./main-test.o" "./pti.o" "./hardware/kit/common/bsp/bsp_stk.o" "./hardware/kit/common/drivers/mx25flash_spi.o" "./platform/Device/SiliconLabs/EFR32BG1B/Source/GCC/startup_efr32bg1b.o" "./platform/Device/SiliconLabs/EFR32BG1B/Source/system_efr32bg1b.o" "./platform/emdrv/dmadrv/src/dmadrv.o" "./platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./platform/emdrv/sleep/src/sleep.o" "./platform/emdrv/tempdrv/src/tempdrv.o" "./platform/emdrv/uartdrv/src/uartdrv.o" "./platform/emdrv/ustimer/src/ustimer.o" "./platform/emlib/src/em_adc.o" "./platform/emlib/src/em_assert.o" "./platform/emlib/src/em_cmu.o" "./platform/emlib/src/em_core.o" "./platform/emlib/src/em_cryotimer.o" "./platform/emlib/src/em_crypto.o" "./platform/emlib/src/em_emu.o" "./platform/emlib/src/em_gpio.o" "./platform/emlib/src/em_i2c.o" "./platform/emlib/src/em_idac.o" "./platform/emlib/src/em_ldma.o" "./platform/emlib/src/em_leuart.o" "./platform/emlib/src/em_msc.o" "./platform/emlib/src/em_rmu.o" "./platform/emlib/src/em_rtcc.o" "./platform/emlib/src/em_se.o" "./platform/emlib/src/em_system.o" "./platform/emlib/src/em_timer.o" "./platform/emlib/src/em_usart.o" "./protocol/balena/balena.o" "./protocol/firmata/Firmata.o" "./protocol/firmata/FirmataMarshaller.o" "./protocol/firmata/FirmataParser.o" "./protocol/serial/Serial.o" "/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/bluetooth/lib/EFR32BG1B/GCC/binapploader.o" "/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/bluetooth/lib/EFR32BG1B/GCC/libmbedtls.a" "/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/bluetooth/lib/EFR32BG1B/GCC/libpsstore.a" "/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/radio/rail_lib/autogen/librail_release/librail_efr32xg1_gcc_release.a" "/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/bluetooth/lib/EFR32BG1B/GCC/libbluetooth.a" -lm -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: balena-firmata.hex'
	arm-none-eabi-objcopy -O ihex "balena-firmata.axf" "balena-firmata.hex"
	@echo ' '

	@echo 'Building bin file: balena-firmata.bin'
	arm-none-eabi-objcopy -O binary "balena-firmata.axf" "balena-firmata.bin"
	@echo ' '

	@echo 'Building s37 file: balena-firmata.s37'
	arm-none-eabi-objcopy -O srec "balena-firmata.axf" "balena-firmata.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "balena-firmata.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(C_DEPS)$(CPP_DEPS) balena-firmata.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

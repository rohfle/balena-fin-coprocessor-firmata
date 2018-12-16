################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emdrv/nvm/config/nvm_config.c 

OBJS += \
./platform/emdrv/nvm/config/nvm_config.o 

C_DEPS += \
./platform/emdrv/nvm/config/nvm_config.d 


# Each subdirectory must supply rules for building sources it contributes
platform/emdrv/nvm/config/nvm_config.o: /home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emdrv/nvm/config/nvm_config.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32BG1B232F256GM48=1' '-D__StackLimit=0x20000000' '-D__HEAP_SIZE=0xD00' '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x800' -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/Device/SiliconLabs/EFR32BG1B/Include" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/CMSIS/Include" -I"/home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emlib/src" -I"/home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emlib/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/hardware/kit/common/drivers" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/hardware/kit/common/bsp" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/hardware/kit/common/halconfig" -I"/home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emdrv/uartdrv/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/bootloader/api" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/bluetooth/ble_stack/inc/common" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/hardware/kit/EFR32BG1_BRD4300C/config" -I"/home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emdrv/gpiointerrupt/inc" -I"/home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emdrv/tempdrv/src" -I"/home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emdrv/sleep/src" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/bluetooth/ble_stack/inc/soc" -I"/home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emdrv/sleep/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/app/bluetooth/common/util" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/Device/SiliconLabs/EFR32BG1B/Source" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/radio/rail_lib/common" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/Device/SiliconLabs/EFR32BG1B/Source/GCC" -I"/home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emdrv/tempdrv/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/halconfig/inc/hal-config" -I"/home/alex/Downloads/SimplicityStudio_v4/developer/sdks/gecko_sdk_suite/v2.4/platform/emdrv/common/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/emdrv/nvm/config/nvm_config.d" -MT"platform/emdrv/nvm/config/nvm_config.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



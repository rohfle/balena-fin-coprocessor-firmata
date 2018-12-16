################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hardware/kit/common/bsp/bsp_stk.c 

OBJS += \
./hardware/kit/common/bsp/bsp_stk.o 

C_DEPS += \
./hardware/kit/common/bsp/bsp_stk.d 


# Each subdirectory must supply rules for building sources it contributes
hardware/kit/common/bsp/bsp_stk.o: ../hardware/kit/common/bsp/bsp_stk.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32BG1B232F256GM48=1' '-D__StackLimit=0x20000000' '-D__HEAP_SIZE=0xD00' '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x800' -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/Device/SiliconLabs/EFR32BG1B/Include" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/CMSIS/Include" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/emlib/src" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/emlib/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/hardware/kit/common/drivers" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/hardware/kit/common/bsp" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/hardware/kit/common/halconfig" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/emdrv/uartdrv/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/bootloader/api" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/bluetooth/ble_stack/inc/common" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/hardware/kit/EFR32BG1_BRD4300C/config" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/emdrv/gpiointerrupt/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/emdrv/tempdrv/src" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/emdrv/sleep/src" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/bluetooth/ble_stack/inc/soc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/emdrv/sleep/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/app/bluetooth/common/util" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/Device/SiliconLabs/EFR32BG1B/Source" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/radio/rail_lib/common" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/Device/SiliconLabs/EFR32BG1B/Source/GCC" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/emdrv/tempdrv/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/halconfig/inc/hal-config" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/emdrv/common/inc" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/platform/bootloader" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/balena" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/firmata" -I"/home/alex/SimplicityStudio/v4_workspace/balena-firmata/protocol/serial" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"hardware/kit/common/bsp/bsp_stk.d" -MT"hardware/kit/common/bsp/bsp_stk.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



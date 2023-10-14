################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../_W5500/W5500_config.c 

OBJS += \
./_W5500/W5500_config.o 

C_DEPS += \
./_W5500/W5500_config.d 


# Each subdirectory must supply rules for building sources it contributes
_W5500/%.o _W5500/%.su _W5500/%.cyclo: ../_W5500/%.c _W5500/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Modbus_tcp/inc -I../IO_Driver/Ethernet/Inc -I../IO_Driver/Internet/DHCP/Inc -I../LCD/Inc -I../_W5500 -I../common -I../timer -I../rtc -I../screen -I../io -I../FlashMemory -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../UnitTest -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-_W5500

clean-_W5500:
	-$(RM) ./_W5500/W5500_config.cyclo ./_W5500/W5500_config.d ./_W5500/W5500_config.o ./_W5500/W5500_config.su

.PHONY: clean-_W5500


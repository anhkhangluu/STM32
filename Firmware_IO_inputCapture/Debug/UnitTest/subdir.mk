################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UnitTest/unittest.c 

OBJS += \
./UnitTest/unittest.o 

C_DEPS += \
./UnitTest/unittest.d 


# Each subdirectory must supply rules for building sources it contributes
UnitTest/%.o UnitTest/%.su UnitTest/%.cyclo: ../UnitTest/%.c UnitTest/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Modbus_tcp/inc -I../IO_Driver/Ethernet/Inc -I../IO_Driver/Internet/DHCP/Inc -I../LCD/Inc -I../_W5500 -I../common -I../timer -I../rtc -I../screen -I../io -I../FlashMemory -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../UnitTest -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-UnitTest

clean-UnitTest:
	-$(RM) ./UnitTest/unittest.cyclo ./UnitTest/unittest.d ./UnitTest/unittest.o ./UnitTest/unittest.su

.PHONY: clean-UnitTest


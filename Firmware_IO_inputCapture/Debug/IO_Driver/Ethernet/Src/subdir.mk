################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../IO_Driver/Ethernet/Src/socket.c \
../IO_Driver/Ethernet/Src/w5500.c \
../IO_Driver/Ethernet/Src/wizchip_conf.c 

OBJS += \
./IO_Driver/Ethernet/Src/socket.o \
./IO_Driver/Ethernet/Src/w5500.o \
./IO_Driver/Ethernet/Src/wizchip_conf.o 

C_DEPS += \
./IO_Driver/Ethernet/Src/socket.d \
./IO_Driver/Ethernet/Src/w5500.d \
./IO_Driver/Ethernet/Src/wizchip_conf.d 


# Each subdirectory must supply rules for building sources it contributes
IO_Driver/Ethernet/Src/%.o IO_Driver/Ethernet/Src/%.su IO_Driver/Ethernet/Src/%.cyclo: ../IO_Driver/Ethernet/Src/%.c IO_Driver/Ethernet/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Modbus_tcp/inc -I../IO_Driver/Ethernet/Inc -I../IO_Driver/Internet/DHCP/Inc -I../LCD/Inc -I../_W5500 -I../common -I../timer -I../rtc -I../screen -I../io -I../FlashMemory -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../UnitTest -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-IO_Driver-2f-Ethernet-2f-Src

clean-IO_Driver-2f-Ethernet-2f-Src:
	-$(RM) ./IO_Driver/Ethernet/Src/socket.cyclo ./IO_Driver/Ethernet/Src/socket.d ./IO_Driver/Ethernet/Src/socket.o ./IO_Driver/Ethernet/Src/socket.su ./IO_Driver/Ethernet/Src/w5500.cyclo ./IO_Driver/Ethernet/Src/w5500.d ./IO_Driver/Ethernet/Src/w5500.o ./IO_Driver/Ethernet/Src/w5500.su ./IO_Driver/Ethernet/Src/wizchip_conf.cyclo ./IO_Driver/Ethernet/Src/wizchip_conf.d ./IO_Driver/Ethernet/Src/wizchip_conf.o ./IO_Driver/Ethernet/Src/wizchip_conf.su

.PHONY: clean-IO_Driver-2f-Ethernet-2f-Src


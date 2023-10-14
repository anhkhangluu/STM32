################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modbus_tcp/src/mbfunccoils.c \
../Modbus_tcp/src/mbfuncdiag.c \
../Modbus_tcp/src/mbfuncdisc.c \
../Modbus_tcp/src/mbfuncholding.c \
../Modbus_tcp/src/mbfuncinput.c \
../Modbus_tcp/src/mbfuncother.c \
../Modbus_tcp/src/mbtcp.c \
../Modbus_tcp/src/mbutils.c \
../Modbus_tcp/src/portevent.c \
../Modbus_tcp/src/porttcp.c 

OBJS += \
./Modbus_tcp/src/mbfunccoils.o \
./Modbus_tcp/src/mbfuncdiag.o \
./Modbus_tcp/src/mbfuncdisc.o \
./Modbus_tcp/src/mbfuncholding.o \
./Modbus_tcp/src/mbfuncinput.o \
./Modbus_tcp/src/mbfuncother.o \
./Modbus_tcp/src/mbtcp.o \
./Modbus_tcp/src/mbutils.o \
./Modbus_tcp/src/portevent.o \
./Modbus_tcp/src/porttcp.o 

C_DEPS += \
./Modbus_tcp/src/mbfunccoils.d \
./Modbus_tcp/src/mbfuncdiag.d \
./Modbus_tcp/src/mbfuncdisc.d \
./Modbus_tcp/src/mbfuncholding.d \
./Modbus_tcp/src/mbfuncinput.d \
./Modbus_tcp/src/mbfuncother.d \
./Modbus_tcp/src/mbtcp.d \
./Modbus_tcp/src/mbutils.d \
./Modbus_tcp/src/portevent.d \
./Modbus_tcp/src/porttcp.d 


# Each subdirectory must supply rules for building sources it contributes
Modbus_tcp/src/%.o Modbus_tcp/src/%.su Modbus_tcp/src/%.cyclo: ../Modbus_tcp/src/%.c Modbus_tcp/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Modbus_tcp/inc -I../IO_Driver/Ethernet/Inc -I../IO_Driver/Internet/DHCP/Inc -I../LCD/Inc -I../_W5500 -I../common -I../timer -I../rtc -I../screen -I../io -I../FlashMemory -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../UnitTest -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Modbus_tcp-2f-src

clean-Modbus_tcp-2f-src:
	-$(RM) ./Modbus_tcp/src/mbfunccoils.cyclo ./Modbus_tcp/src/mbfunccoils.d ./Modbus_tcp/src/mbfunccoils.o ./Modbus_tcp/src/mbfunccoils.su ./Modbus_tcp/src/mbfuncdiag.cyclo ./Modbus_tcp/src/mbfuncdiag.d ./Modbus_tcp/src/mbfuncdiag.o ./Modbus_tcp/src/mbfuncdiag.su ./Modbus_tcp/src/mbfuncdisc.cyclo ./Modbus_tcp/src/mbfuncdisc.d ./Modbus_tcp/src/mbfuncdisc.o ./Modbus_tcp/src/mbfuncdisc.su ./Modbus_tcp/src/mbfuncholding.cyclo ./Modbus_tcp/src/mbfuncholding.d ./Modbus_tcp/src/mbfuncholding.o ./Modbus_tcp/src/mbfuncholding.su ./Modbus_tcp/src/mbfuncinput.cyclo ./Modbus_tcp/src/mbfuncinput.d ./Modbus_tcp/src/mbfuncinput.o ./Modbus_tcp/src/mbfuncinput.su ./Modbus_tcp/src/mbfuncother.cyclo ./Modbus_tcp/src/mbfuncother.d ./Modbus_tcp/src/mbfuncother.o ./Modbus_tcp/src/mbfuncother.su ./Modbus_tcp/src/mbtcp.cyclo ./Modbus_tcp/src/mbtcp.d ./Modbus_tcp/src/mbtcp.o ./Modbus_tcp/src/mbtcp.su ./Modbus_tcp/src/mbutils.cyclo ./Modbus_tcp/src/mbutils.d ./Modbus_tcp/src/mbutils.o ./Modbus_tcp/src/mbutils.su ./Modbus_tcp/src/portevent.cyclo ./Modbus_tcp/src/portevent.d ./Modbus_tcp/src/portevent.o ./Modbus_tcp/src/portevent.su ./Modbus_tcp/src/porttcp.cyclo ./Modbus_tcp/src/porttcp.d ./Modbus_tcp/src/porttcp.o ./Modbus_tcp/src/porttcp.su

.PHONY: clean-Modbus_tcp-2f-src


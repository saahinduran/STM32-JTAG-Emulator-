################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DAP.c \
../Core/Src/DAP_vendor.c \
../Core/Src/JTAG_DP.c \
../Core/Src/SWO.c \
../Core/Src/SW_DP.c \
../Core/Src/UART.c \
../Core/Src/arm.c \
../Core/Src/dpacc.c \
../Core/Src/jtag.c \
../Core/Src/main.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c 

OBJS += \
./Core/Src/DAP.o \
./Core/Src/DAP_vendor.o \
./Core/Src/JTAG_DP.o \
./Core/Src/SWO.o \
./Core/Src/SW_DP.o \
./Core/Src/UART.o \
./Core/Src/arm.o \
./Core/Src/dpacc.o \
./Core/Src/jtag.o \
./Core/Src/main.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o 

C_DEPS += \
./Core/Src/DAP.d \
./Core/Src/DAP_vendor.d \
./Core/Src/JTAG_DP.d \
./Core/Src/SWO.d \
./Core/Src/SW_DP.d \
./Core/Src/UART.d \
./Core/Src/arm.d \
./Core/Src/dpacc.d \
./Core/Src/jtag.d \
./Core/Src/main.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/DAP.cyclo ./Core/Src/DAP.d ./Core/Src/DAP.o ./Core/Src/DAP.su ./Core/Src/DAP_vendor.cyclo ./Core/Src/DAP_vendor.d ./Core/Src/DAP_vendor.o ./Core/Src/DAP_vendor.su ./Core/Src/JTAG_DP.cyclo ./Core/Src/JTAG_DP.d ./Core/Src/JTAG_DP.o ./Core/Src/JTAG_DP.su ./Core/Src/SWO.cyclo ./Core/Src/SWO.d ./Core/Src/SWO.o ./Core/Src/SWO.su ./Core/Src/SW_DP.cyclo ./Core/Src/SW_DP.d ./Core/Src/SW_DP.o ./Core/Src/SW_DP.su ./Core/Src/UART.cyclo ./Core/Src/UART.d ./Core/Src/UART.o ./Core/Src/UART.su ./Core/Src/arm.cyclo ./Core/Src/arm.d ./Core/Src/arm.o ./Core/Src/arm.su ./Core/Src/dpacc.cyclo ./Core/Src/dpacc.d ./Core/Src/dpacc.o ./Core/Src/dpacc.su ./Core/Src/jtag.cyclo ./Core/Src/jtag.d ./Core/Src/jtag.o ./Core/Src/jtag.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su

.PHONY: clean-Core-2f-Src


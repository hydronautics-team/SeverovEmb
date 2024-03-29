################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FreeRTOSTick.c \
../Core/Src/MS5837.c \
../Core/Src/checksum.c \
../Core/Src/communication.c \
../Core/Src/dma.c \
../Core/Src/flash.c \
../Core/Src/freertos.c \
../Core/Src/global.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/stabilization.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_hal_timebase_tim.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c \
../Core/Src/thrusters.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/FreeRTOSTick.o \
./Core/Src/MS5837.o \
./Core/Src/checksum.o \
./Core/Src/communication.o \
./Core/Src/dma.o \
./Core/Src/flash.o \
./Core/Src/freertos.o \
./Core/Src/global.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/stabilization.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_hal_timebase_tim.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o \
./Core/Src/thrusters.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/FreeRTOSTick.d \
./Core/Src/MS5837.d \
./Core/Src/checksum.d \
./Core/Src/communication.d \
./Core/Src/dma.d \
./Core/Src/flash.d \
./Core/Src/freertos.d \
./Core/Src/global.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/stabilization.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_hal_timebase_tim.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d \
./Core/Src/thrusters.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/FreeRTOSTick.cyclo ./Core/Src/FreeRTOSTick.d ./Core/Src/FreeRTOSTick.o ./Core/Src/FreeRTOSTick.su ./Core/Src/MS5837.cyclo ./Core/Src/MS5837.d ./Core/Src/MS5837.o ./Core/Src/MS5837.su ./Core/Src/checksum.cyclo ./Core/Src/checksum.d ./Core/Src/checksum.o ./Core/Src/checksum.su ./Core/Src/communication.cyclo ./Core/Src/communication.d ./Core/Src/communication.o ./Core/Src/communication.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/flash.cyclo ./Core/Src/flash.d ./Core/Src/flash.o ./Core/Src/flash.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/global.cyclo ./Core/Src/global.d ./Core/Src/global.o ./Core/Src/global.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stabilization.cyclo ./Core/Src/stabilization.d ./Core/Src/stabilization.o ./Core/Src/stabilization.su ./Core/Src/stm32f3xx_hal_msp.cyclo ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_hal_timebase_tim.cyclo ./Core/Src/stm32f3xx_hal_timebase_tim.d ./Core/Src/stm32f3xx_hal_timebase_tim.o ./Core/Src/stm32f3xx_hal_timebase_tim.su ./Core/Src/stm32f3xx_it.cyclo ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f3xx.cyclo ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su ./Core/Src/thrusters.cyclo ./Core/Src/thrusters.d ./Core/Src/thrusters.o ./Core/Src/thrusters.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src


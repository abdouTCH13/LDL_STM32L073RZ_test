################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Debug/debug.c \
../Core/Inc/Debug/stm32l0xx_hal_uart.c \
../Core/Inc/Debug/stm32l0xx_hal_uart_ex.c \
../Core/Inc/Debug/stm32l0xx_hal_usart.c 

OBJS += \
./Core/Inc/Debug/debug.o \
./Core/Inc/Debug/stm32l0xx_hal_uart.o \
./Core/Inc/Debug/stm32l0xx_hal_uart_ex.o \
./Core/Inc/Debug/stm32l0xx_hal_usart.o 

C_DEPS += \
./Core/Inc/Debug/debug.d \
./Core/Inc/Debug/stm32l0xx_hal_uart.d \
./Core/Inc/Debug/stm32l0xx_hal_uart_ex.d \
./Core/Inc/Debug/stm32l0xx_hal_usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Debug/debug.o: ../Core/Inc/Debug/debug.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/Debug/debug.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Inc/Debug/stm32l0xx_hal_uart.o: ../Core/Inc/Debug/stm32l0xx_hal_uart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/Debug/stm32l0xx_hal_uart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Inc/Debug/stm32l0xx_hal_uart_ex.o: ../Core/Inc/Debug/stm32l0xx_hal_uart_ex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/Debug/stm32l0xx_hal_uart_ex.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Inc/Debug/stm32l0xx_hal_usart.o: ../Core/Inc/Debug/stm32l0xx_hal_usart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/Debug/stm32l0xx_hal_usart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


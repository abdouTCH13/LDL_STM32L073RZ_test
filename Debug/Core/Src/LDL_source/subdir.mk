################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LDL_source/LDL_interface.c \
../Core/Src/LDL_source/ldl_aes.c \
../Core/Src/LDL_source/ldl_cmac.c \
../Core/Src/LDL_source/ldl_ctr.c \
../Core/Src/LDL_source/ldl_frame.c \
../Core/Src/LDL_source/ldl_mac.c \
../Core/Src/LDL_source/ldl_mac_commands.c \
../Core/Src/LDL_source/ldl_ops.c \
../Core/Src/LDL_source/ldl_radio.c \
../Core/Src/LDL_source/ldl_region.c \
../Core/Src/LDL_source/ldl_sm.c \
../Core/Src/LDL_source/ldl_stream.c \
../Core/Src/LDL_source/ldl_system.c 

OBJS += \
./Core/Src/LDL_source/LDL_interface.o \
./Core/Src/LDL_source/ldl_aes.o \
./Core/Src/LDL_source/ldl_cmac.o \
./Core/Src/LDL_source/ldl_ctr.o \
./Core/Src/LDL_source/ldl_frame.o \
./Core/Src/LDL_source/ldl_mac.o \
./Core/Src/LDL_source/ldl_mac_commands.o \
./Core/Src/LDL_source/ldl_ops.o \
./Core/Src/LDL_source/ldl_radio.o \
./Core/Src/LDL_source/ldl_region.o \
./Core/Src/LDL_source/ldl_sm.o \
./Core/Src/LDL_source/ldl_stream.o \
./Core/Src/LDL_source/ldl_system.o 

C_DEPS += \
./Core/Src/LDL_source/LDL_interface.d \
./Core/Src/LDL_source/ldl_aes.d \
./Core/Src/LDL_source/ldl_cmac.d \
./Core/Src/LDL_source/ldl_ctr.d \
./Core/Src/LDL_source/ldl_frame.d \
./Core/Src/LDL_source/ldl_mac.d \
./Core/Src/LDL_source/ldl_mac_commands.d \
./Core/Src/LDL_source/ldl_ops.d \
./Core/Src/LDL_source/ldl_radio.d \
./Core/Src/LDL_source/ldl_region.d \
./Core/Src/LDL_source/ldl_sm.d \
./Core/Src/LDL_source/ldl_stream.d \
./Core/Src/LDL_source/ldl_system.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LDL_source/LDL_interface.o: ../Core/Src/LDL_source/LDL_interface.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/LDL_interface.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_aes.o: ../Core/Src/LDL_source/ldl_aes.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_aes.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_cmac.o: ../Core/Src/LDL_source/ldl_cmac.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_cmac.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_ctr.o: ../Core/Src/LDL_source/ldl_ctr.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_ctr.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_frame.o: ../Core/Src/LDL_source/ldl_frame.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_frame.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_mac.o: ../Core/Src/LDL_source/ldl_mac.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_mac.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_mac_commands.o: ../Core/Src/LDL_source/ldl_mac_commands.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_mac_commands.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_ops.o: ../Core/Src/LDL_source/ldl_ops.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_ops.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_radio.o: ../Core/Src/LDL_source/ldl_radio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_radio.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_region.o: ../Core/Src/LDL_source/ldl_region.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_region.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_sm.o: ../Core/Src/LDL_source/ldl_sm.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_sm.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_stream.o: ../Core/Src/LDL_source/ldl_stream.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_stream.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/LDL_source/ldl_system.o: ../Core/Src/LDL_source/ldl_system.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DLDL_ENABLE_SX1272 -DUSE_HAL_DRIVER -DLDL_ENABLE_EU_863_870 -DSTM32L073xx -DDEBUG -c -I../Core/Inc/Debug -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Core/Inc/LDL_include -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/LDL_source/ldl_system.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/IMU/Src/CompFilter.c \
../Drivers/IMU/Src/IMU_I2C.c \
../Drivers/IMU/Src/IMU_UartMsg.c \
../Drivers/IMU/Src/MPUXX50.c 

OBJS += \
./Drivers/IMU/Src/CompFilter.o \
./Drivers/IMU/Src/IMU_I2C.o \
./Drivers/IMU/Src/IMU_UartMsg.o \
./Drivers/IMU/Src/MPUXX50.o 

C_DEPS += \
./Drivers/IMU/Src/CompFilter.d \
./Drivers/IMU/Src/IMU_I2C.d \
./Drivers/IMU/Src/IMU_UartMsg.d \
./Drivers/IMU/Src/MPUXX50.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IMU/Src/%.o Drivers/IMU/Src/%.su Drivers/IMU/Src/%.cyclo: ../Drivers/IMU/Src/%.c Drivers/IMU/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F446xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/bheesma/STM32CubeIDE/workspace_1.12.0/STM32_IMU/Drivers/IMU/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-IMU-2f-Src

clean-Drivers-2f-IMU-2f-Src:
	-$(RM) ./Drivers/IMU/Src/CompFilter.cyclo ./Drivers/IMU/Src/CompFilter.d ./Drivers/IMU/Src/CompFilter.o ./Drivers/IMU/Src/CompFilter.su ./Drivers/IMU/Src/IMU_I2C.cyclo ./Drivers/IMU/Src/IMU_I2C.d ./Drivers/IMU/Src/IMU_I2C.o ./Drivers/IMU/Src/IMU_I2C.su ./Drivers/IMU/Src/IMU_UartMsg.cyclo ./Drivers/IMU/Src/IMU_UartMsg.d ./Drivers/IMU/Src/IMU_UartMsg.o ./Drivers/IMU/Src/IMU_UartMsg.su ./Drivers/IMU/Src/MPUXX50.cyclo ./Drivers/IMU/Src/MPUXX50.d ./Drivers/IMU/Src/MPUXX50.o ./Drivers/IMU/Src/MPUXX50.su

.PHONY: clean-Drivers-2f-IMU-2f-Src


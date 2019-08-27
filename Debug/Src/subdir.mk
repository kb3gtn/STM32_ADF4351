################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adf4351.c \
../Src/main.c \
../Src/ringbuffer.c \
../Src/stm32l0xx_hal_msp.c \
../Src/stm32l0xx_it.c \
../Src/system_stm32l0xx.c 

OBJS += \
./Src/adf4351.o \
./Src/main.o \
./Src/ringbuffer.o \
./Src/stm32l0xx_hal_msp.o \
./Src/stm32l0xx_it.o \
./Src/system_stm32l0xx.o 

C_DEPS += \
./Src/adf4351.d \
./Src/main.d \
./Src/ringbuffer.d \
./Src/stm32l0xx_hal_msp.d \
./Src/stm32l0xx_it.d \
./Src/system_stm32l0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L011xx -I"/home/kb3gtn/sandbox/AC6_WORKSPACE/ADF4351_Controller2/ADF4251_Controller2/Inc" -I"/home/kb3gtn/sandbox/AC6_WORKSPACE/ADF4351_Controller2/ADF4251_Controller2/Drivers/STM32L0xx_HAL_Driver/Inc" -I"/home/kb3gtn/sandbox/AC6_WORKSPACE/ADF4351_Controller2/ADF4251_Controller2/Drivers/STM32L0xx_HAL_Driver/Inc/Legacy" -I"/home/kb3gtn/sandbox/AC6_WORKSPACE/ADF4351_Controller2/ADF4251_Controller2/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"/home/kb3gtn/sandbox/AC6_WORKSPACE/ADF4351_Controller2/ADF4251_Controller2/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



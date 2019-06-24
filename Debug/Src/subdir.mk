################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/IMU_Sensors.c \
../Src/L3GD20.c \
../Src/MadgwickAHRS.c \
../Src/freertos.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_hal_timebase_tim.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/IMU_Sensors.o \
./Src/L3GD20.o \
./Src/MadgwickAHRS.o \
./Src/freertos.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_hal_timebase_tim.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/IMU_Sensors.d \
./Src/L3GD20.d \
./Src/MadgwickAHRS.d \
./Src/freertos.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_hal_timebase_tim.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F411xE -I"C:/Users/mihay002/Desktop/MyProjects/DirMk1/Inc" -I"C:/Users/mihay002/Desktop/MyProjects/DirMk1/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/mihay002/Desktop/MyProjects/DirMk1/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/mihay002/Desktop/MyProjects/DirMk1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/mihay002/Desktop/MyProjects/DirMk1/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/mihay002/Desktop/MyProjects/DirMk1/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/mihay002/Desktop/MyProjects/DirMk1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/mihay002/Desktop/MyProjects/DirMk1/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



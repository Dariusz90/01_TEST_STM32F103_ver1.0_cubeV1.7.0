################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MyTask/HalSystemCallback.c \
../MyTask/common_fun.c 

OBJS += \
./MyTask/HalSystemCallback.o \
./MyTask/common_fun.o 

C_DEPS += \
./MyTask/HalSystemCallback.d \
./MyTask/common_fun.d 


# Each subdirectory must supply rules for building sources it contributes
MyTask/%.o: ../MyTask/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -std=c99 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/Inc" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/MyTask" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/GPSTracker_Library" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



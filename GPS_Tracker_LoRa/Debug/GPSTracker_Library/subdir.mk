################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../GPSTracker_Library/DB_GPS_NMEA_Simple.c \
../GPSTracker_Library/DB_GSM_SIM800.c \
../GPSTracker_Library/DB_RFM98W.c \
../GPSTracker_Library/DB_RFM98W_LoRa.c \
../GPSTracker_Library/DB_ring_buffer.c \
../GPSTracker_Library/DB_soft_timer.c \
../GPSTracker_Library/DB_string.c 

OBJS += \
./GPSTracker_Library/DB_GPS_NMEA_Simple.o \
./GPSTracker_Library/DB_GSM_SIM800.o \
./GPSTracker_Library/DB_RFM98W.o \
./GPSTracker_Library/DB_RFM98W_LoRa.o \
./GPSTracker_Library/DB_ring_buffer.o \
./GPSTracker_Library/DB_soft_timer.o \
./GPSTracker_Library/DB_string.o 

C_DEPS += \
./GPSTracker_Library/DB_GPS_NMEA_Simple.d \
./GPSTracker_Library/DB_GSM_SIM800.d \
./GPSTracker_Library/DB_RFM98W.d \
./GPSTracker_Library/DB_RFM98W_LoRa.d \
./GPSTracker_Library/DB_ring_buffer.d \
./GPSTracker_Library/DB_soft_timer.d \
./GPSTracker_Library/DB_string.d 


# Each subdirectory must supply rules for building sources it contributes
GPSTracker_Library/%.o: ../GPSTracker_Library/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -std=c99 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/Inc" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/MyTask" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/GPSTracker_Library" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Prezes/workspace/GPS_Tracker_LoRa/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



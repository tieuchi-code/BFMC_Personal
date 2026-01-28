################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MyDriver/Src/as5600.c \
../MyDriver/Src/bno055.c \
../MyDriver/Src/i2c.c \
../MyDriver/Src/servo.c \
../MyDriver/Src/timer.c \
../MyDriver/Src/uart.c 

OBJS += \
./MyDriver/Src/as5600.o \
./MyDriver/Src/bno055.o \
./MyDriver/Src/i2c.o \
./MyDriver/Src/servo.o \
./MyDriver/Src/timer.o \
./MyDriver/Src/uart.o 

C_DEPS += \
./MyDriver/Src/as5600.d \
./MyDriver/Src/bno055.d \
./MyDriver/Src/i2c.d \
./MyDriver/Src/servo.d \
./MyDriver/Src/timer.d \
./MyDriver/Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
MyDriver/Src/%.o MyDriver/Src/%.su MyDriver/Src/%.cyclo: ../MyDriver/Src/%.c MyDriver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Admin/Desktop/BFMC_Personal/STM32/BFMC_Sensor_401/MyDriver/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MyDriver-2f-Src

clean-MyDriver-2f-Src:
	-$(RM) ./MyDriver/Src/as5600.cyclo ./MyDriver/Src/as5600.d ./MyDriver/Src/as5600.o ./MyDriver/Src/as5600.su ./MyDriver/Src/bno055.cyclo ./MyDriver/Src/bno055.d ./MyDriver/Src/bno055.o ./MyDriver/Src/bno055.su ./MyDriver/Src/i2c.cyclo ./MyDriver/Src/i2c.d ./MyDriver/Src/i2c.o ./MyDriver/Src/i2c.su ./MyDriver/Src/servo.cyclo ./MyDriver/Src/servo.d ./MyDriver/Src/servo.o ./MyDriver/Src/servo.su ./MyDriver/Src/timer.cyclo ./MyDriver/Src/timer.d ./MyDriver/Src/timer.o ./MyDriver/Src/timer.su ./MyDriver/Src/uart.cyclo ./MyDriver/Src/uart.d ./MyDriver/Src/uart.o ./MyDriver/Src/uart.su

.PHONY: clean-MyDriver-2f-Src


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/Driver_gpio.c 

OBJS += \
./Drivers/Src/Driver_gpio.o 

C_DEPS += \
./Drivers/Src/Driver_gpio.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"F:/2024/STM32_Workspace/Mastering Microcontroller and Embedded Driver Development CM4 FastBit/Stm32F4_Drivers/Drivers/Inc" -I"F:/2024/STM32_Workspace/Mastering Microcontroller and Embedded Driver Development CM4 FastBit/Driver_GPIO/Drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/Driver_gpio.d ./Drivers/Src/Driver_gpio.o ./Drivers/Src/Driver_gpio.su

.PHONY: clean-Drivers-2f-Src


################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Device/ST/STM32F30x/Source/Templates/system_stm32f30x.c 

OBJS += \
./src/CMSIS/Device/ST/STM32F30x/Source/Templates/system_stm32f30x.o 

C_DEPS += \
./src/CMSIS/Device/ST/STM32F30x/Source/Templates/system_stm32f30x.d 


# Each subdirectory must supply rules for building sources it contributes
src/CMSIS/Device/ST/STM32F30x/Source/Templates/system_stm32f30x.o: /home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Device/ST/STM32F30x/Source/Templates/system_stm32f30x.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



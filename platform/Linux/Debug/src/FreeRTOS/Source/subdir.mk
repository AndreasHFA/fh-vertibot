################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/croutine.c \
/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/list.c \
/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/queue.c \
/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/tasks.c \
/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/timers.c 

OBJS += \
./src/FreeRTOS/Source/croutine.o \
./src/FreeRTOS/Source/list.o \
./src/FreeRTOS/Source/queue.o \
./src/FreeRTOS/Source/tasks.o \
./src/FreeRTOS/Source/timers.o 

C_DEPS += \
./src/FreeRTOS/Source/croutine.d \
./src/FreeRTOS/Source/list.d \
./src/FreeRTOS/Source/queue.d \
./src/FreeRTOS/Source/tasks.d \
./src/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
src/FreeRTOS/Source/croutine.o: /home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/croutine.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/FreeRTOS/Source/list.o: /home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/list.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/FreeRTOS/Source/queue.o: /home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/queue.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/FreeRTOS/Source/tasks.o: /home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/tasks.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/FreeRTOS/Source/timers.o: /home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/timers.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/FH-Vertibot/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



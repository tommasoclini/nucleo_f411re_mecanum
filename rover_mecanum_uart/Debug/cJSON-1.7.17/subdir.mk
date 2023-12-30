################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../cJSON-1.7.17/cJSON.c \
../cJSON-1.7.17/cJSON_Utils.c 

OBJS += \
./cJSON-1.7.17/cJSON.o \
./cJSON-1.7.17/cJSON_Utils.o 

C_DEPS += \
./cJSON-1.7.17/cJSON.d \
./cJSON-1.7.17/cJSON_Utils.d 


# Each subdirectory must supply rules for building sources it contributes
cJSON-1.7.17/%.o cJSON-1.7.17/%.su cJSON-1.7.17/%.cyclo: ../cJSON-1.7.17/%.c cJSON-1.7.17/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tommaso/git_repos/esp32_cam_nucleo_f411re_mecanum/nucleo_f411re_mecanum/rover_mecanum_uart/lwrb/src/include" -I"/home/tommaso/git_repos/esp32_cam_nucleo_f411re_mecanum/nucleo_f411re_mecanum/rover_mecanum_uart/lwpkt/src/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-cJSON-2d-1-2e-7-2e-17

clean-cJSON-2d-1-2e-7-2e-17:
	-$(RM) ./cJSON-1.7.17/cJSON.cyclo ./cJSON-1.7.17/cJSON.d ./cJSON-1.7.17/cJSON.o ./cJSON-1.7.17/cJSON.su ./cJSON-1.7.17/cJSON_Utils.cyclo ./cJSON-1.7.17/cJSON_Utils.d ./cJSON-1.7.17/cJSON_Utils.o ./cJSON-1.7.17/cJSON_Utils.su

.PHONY: clean-cJSON-2d-1-2e-7-2e-17


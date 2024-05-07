################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS/system_stm32f4xx.c 

OBJS += \
./CMSIS/system_stm32f4xx.o 

C_DEPS += \
./CMSIS/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/%.o CMSIS/%.su CMSIS/%.cyclo: ../CMSIS/%.c CMSIS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411xE -c -I../Inc -I../CMSIS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS

clean-CMSIS:
	-$(RM) ./CMSIS/system_stm32f4xx.cyclo ./CMSIS/system_stm32f4xx.d ./CMSIS/system_stm32f4xx.o ./CMSIS/system_stm32f4xx.su

.PHONY: clean-CMSIS


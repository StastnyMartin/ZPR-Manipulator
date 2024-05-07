################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Src/startup_stm32f411retx.s 

C_SRCS += \
../Src/main.c \
../Src/map_shield.c \
../Src/stm_core.c \
../Src/stm_usart.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/main.o \
./Src/map_shield.o \
./Src/startup_stm32f411retx.o \
./Src/stm_core.o \
./Src/stm_usart.o \
./Src/syscalls.o \
./Src/sysmem.o 

S_DEPS += \
./Src/startup_stm32f411retx.d 

C_DEPS += \
./Src/main.d \
./Src/map_shield.d \
./Src/stm_core.d \
./Src/stm_usart.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411xE -c -I../Inc -I../CMSIS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/%.o: ../Src/%.s Src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/map_shield.cyclo ./Src/map_shield.d ./Src/map_shield.o ./Src/map_shield.su ./Src/startup_stm32f411retx.d ./Src/startup_stm32f411retx.o ./Src/stm_core.cyclo ./Src/stm_core.d ./Src/stm_core.o ./Src/stm_core.su ./Src/stm_usart.cyclo ./Src/stm_usart.d ./Src/stm_usart.o ./Src/stm_usart.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src


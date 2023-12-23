################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tensorflow/tensorflow/lite/c/common.c 

C_DEPS += \
./tensorflow/tensorflow/lite/c/common.d 

OBJS += \
./tensorflow/tensorflow/lite/c/common.o 


# Each subdirectory must supply rules for building sources it contributes
tensorflow/tensorflow/lite/c/%.o tensorflow/tensorflow/lite/c/%.su: ../tensorflow/tensorflow/lite/c/%.c tensorflow/tensorflow/lite/c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DMCU_SERIES_H7 -DSTM32H7 -c -I../Core/Inc -I../third_party/ruy -I../third_party/gemmlowp -I../third_party/flatbuffers/include -I../tensorflow -I../third_party -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tensorflow-2f-tensorflow-2f-lite-2f-c

clean-tensorflow-2f-tensorflow-2f-lite-2f-c:
	-$(RM) ./tensorflow/tensorflow/lite/c/common.d ./tensorflow/tensorflow/lite/c/common.o ./tensorflow/tensorflow/lite/c/common.su

.PHONY: clean-tensorflow-2f-tensorflow-2f-lite-2f-c


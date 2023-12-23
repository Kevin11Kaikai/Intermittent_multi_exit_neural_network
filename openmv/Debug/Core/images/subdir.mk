################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/images/mnist_1_21.c \
../Core/images/mnist_6_374.c \
../Core/images/mnist_idx18.c \
../Core/images/mnist_idx68.c 

C_DEPS += \
./Core/images/mnist_1_21.d \
./Core/images/mnist_6_374.d \
./Core/images/mnist_idx18.d \
./Core/images/mnist_idx68.d 

OBJS += \
./Core/images/mnist_1_21.o \
./Core/images/mnist_6_374.o \
./Core/images/mnist_idx18.o \
./Core/images/mnist_idx68.o 


# Each subdirectory must supply rules for building sources it contributes
Core/images/%.o Core/images/%.su: ../Core/images/%.c Core/images/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DMCU_SERIES_H7 -DSTM32H7 -c -I../Core/Inc -I../third_party/ruy -I../third_party/gemmlowp -I../third_party/flatbuffers/include -I../tensorflow -I../third_party -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-images

clean-Core-2f-images:
	-$(RM) ./Core/images/mnist_1_21.d ./Core/images/mnist_1_21.o ./Core/images/mnist_1_21.su ./Core/images/mnist_6_374.d ./Core/images/mnist_6_374.o ./Core/images/mnist_6_374.su ./Core/images/mnist_idx18.d ./Core/images/mnist_idx18.o ./Core/images/mnist_idx18.su ./Core/images/mnist_idx68.d ./Core/images/mnist_idx68.o ./Core/images/mnist_idx68.su

.PHONY: clean-Core-2f-images


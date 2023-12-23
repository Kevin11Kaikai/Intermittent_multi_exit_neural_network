################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../tensorflow/tensorflow/lite/micro/kernels/activations.cc \
../tensorflow/tensorflow/lite/micro/kernels/add.cc \
../tensorflow/tensorflow/lite/micro/kernels/all_ops_resolver.cc \
../tensorflow/tensorflow/lite/micro/kernels/arg_min_max.cc \
../tensorflow/tensorflow/lite/micro/kernels/ceil.cc \
../tensorflow/tensorflow/lite/micro/kernels/circular_buffer.cc \
../tensorflow/tensorflow/lite/micro/kernels/comparisons.cc \
../tensorflow/tensorflow/lite/micro/kernels/concatenation.cc \
../tensorflow/tensorflow/lite/micro/kernels/conv.cc \
../tensorflow/tensorflow/lite/micro/kernels/depthwise_conv.cc \
../tensorflow/tensorflow/lite/micro/kernels/dequantize.cc \
../tensorflow/tensorflow/lite/micro/kernels/elementwise.cc \
../tensorflow/tensorflow/lite/micro/kernels/floor.cc \
../tensorflow/tensorflow/lite/micro/kernels/fully_connected.cc \
../tensorflow/tensorflow/lite/micro/kernels/l2norm.cc \
../tensorflow/tensorflow/lite/micro/kernels/logical.cc \
../tensorflow/tensorflow/lite/micro/kernels/logistic.cc \
../tensorflow/tensorflow/lite/micro/kernels/maximum_minimum.cc \
../tensorflow/tensorflow/lite/micro/kernels/mul.cc \
../tensorflow/tensorflow/lite/micro/kernels/neg.cc \
../tensorflow/tensorflow/lite/micro/kernels/pack.cc \
../tensorflow/tensorflow/lite/micro/kernels/pad.cc \
../tensorflow/tensorflow/lite/micro/kernels/pooling.cc \
../tensorflow/tensorflow/lite/micro/kernels/prelu.cc \
../tensorflow/tensorflow/lite/micro/kernels/quantize.cc \
../tensorflow/tensorflow/lite/micro/kernels/reduce.cc \
../tensorflow/tensorflow/lite/micro/kernels/reshape.cc \
../tensorflow/tensorflow/lite/micro/kernels/resize_nearest_neighbor.cc \
../tensorflow/tensorflow/lite/micro/kernels/round.cc \
../tensorflow/tensorflow/lite/micro/kernels/softmax.cc \
../tensorflow/tensorflow/lite/micro/kernels/split.cc \
../tensorflow/tensorflow/lite/micro/kernels/strided_slice.cc \
../tensorflow/tensorflow/lite/micro/kernels/sub.cc \
../tensorflow/tensorflow/lite/micro/kernels/svdf.cc \
../tensorflow/tensorflow/lite/micro/kernels/unpack.cc 

CC_DEPS += \
./tensorflow/tensorflow/lite/micro/kernels/activations.d \
./tensorflow/tensorflow/lite/micro/kernels/add.d \
./tensorflow/tensorflow/lite/micro/kernels/all_ops_resolver.d \
./tensorflow/tensorflow/lite/micro/kernels/arg_min_max.d \
./tensorflow/tensorflow/lite/micro/kernels/ceil.d \
./tensorflow/tensorflow/lite/micro/kernels/circular_buffer.d \
./tensorflow/tensorflow/lite/micro/kernels/comparisons.d \
./tensorflow/tensorflow/lite/micro/kernels/concatenation.d \
./tensorflow/tensorflow/lite/micro/kernels/conv.d \
./tensorflow/tensorflow/lite/micro/kernels/depthwise_conv.d \
./tensorflow/tensorflow/lite/micro/kernels/dequantize.d \
./tensorflow/tensorflow/lite/micro/kernels/elementwise.d \
./tensorflow/tensorflow/lite/micro/kernels/floor.d \
./tensorflow/tensorflow/lite/micro/kernels/fully_connected.d \
./tensorflow/tensorflow/lite/micro/kernels/l2norm.d \
./tensorflow/tensorflow/lite/micro/kernels/logical.d \
./tensorflow/tensorflow/lite/micro/kernels/logistic.d \
./tensorflow/tensorflow/lite/micro/kernels/maximum_minimum.d \
./tensorflow/tensorflow/lite/micro/kernels/mul.d \
./tensorflow/tensorflow/lite/micro/kernels/neg.d \
./tensorflow/tensorflow/lite/micro/kernels/pack.d \
./tensorflow/tensorflow/lite/micro/kernels/pad.d \
./tensorflow/tensorflow/lite/micro/kernels/pooling.d \
./tensorflow/tensorflow/lite/micro/kernels/prelu.d \
./tensorflow/tensorflow/lite/micro/kernels/quantize.d \
./tensorflow/tensorflow/lite/micro/kernels/reduce.d \
./tensorflow/tensorflow/lite/micro/kernels/reshape.d \
./tensorflow/tensorflow/lite/micro/kernels/resize_nearest_neighbor.d \
./tensorflow/tensorflow/lite/micro/kernels/round.d \
./tensorflow/tensorflow/lite/micro/kernels/softmax.d \
./tensorflow/tensorflow/lite/micro/kernels/split.d \
./tensorflow/tensorflow/lite/micro/kernels/strided_slice.d \
./tensorflow/tensorflow/lite/micro/kernels/sub.d \
./tensorflow/tensorflow/lite/micro/kernels/svdf.d \
./tensorflow/tensorflow/lite/micro/kernels/unpack.d 

OBJS += \
./tensorflow/tensorflow/lite/micro/kernels/activations.o \
./tensorflow/tensorflow/lite/micro/kernels/add.o \
./tensorflow/tensorflow/lite/micro/kernels/all_ops_resolver.o \
./tensorflow/tensorflow/lite/micro/kernels/arg_min_max.o \
./tensorflow/tensorflow/lite/micro/kernels/ceil.o \
./tensorflow/tensorflow/lite/micro/kernels/circular_buffer.o \
./tensorflow/tensorflow/lite/micro/kernels/comparisons.o \
./tensorflow/tensorflow/lite/micro/kernels/concatenation.o \
./tensorflow/tensorflow/lite/micro/kernels/conv.o \
./tensorflow/tensorflow/lite/micro/kernels/depthwise_conv.o \
./tensorflow/tensorflow/lite/micro/kernels/dequantize.o \
./tensorflow/tensorflow/lite/micro/kernels/elementwise.o \
./tensorflow/tensorflow/lite/micro/kernels/floor.o \
./tensorflow/tensorflow/lite/micro/kernels/fully_connected.o \
./tensorflow/tensorflow/lite/micro/kernels/l2norm.o \
./tensorflow/tensorflow/lite/micro/kernels/logical.o \
./tensorflow/tensorflow/lite/micro/kernels/logistic.o \
./tensorflow/tensorflow/lite/micro/kernels/maximum_minimum.o \
./tensorflow/tensorflow/lite/micro/kernels/mul.o \
./tensorflow/tensorflow/lite/micro/kernels/neg.o \
./tensorflow/tensorflow/lite/micro/kernels/pack.o \
./tensorflow/tensorflow/lite/micro/kernels/pad.o \
./tensorflow/tensorflow/lite/micro/kernels/pooling.o \
./tensorflow/tensorflow/lite/micro/kernels/prelu.o \
./tensorflow/tensorflow/lite/micro/kernels/quantize.o \
./tensorflow/tensorflow/lite/micro/kernels/reduce.o \
./tensorflow/tensorflow/lite/micro/kernels/reshape.o \
./tensorflow/tensorflow/lite/micro/kernels/resize_nearest_neighbor.o \
./tensorflow/tensorflow/lite/micro/kernels/round.o \
./tensorflow/tensorflow/lite/micro/kernels/softmax.o \
./tensorflow/tensorflow/lite/micro/kernels/split.o \
./tensorflow/tensorflow/lite/micro/kernels/strided_slice.o \
./tensorflow/tensorflow/lite/micro/kernels/sub.o \
./tensorflow/tensorflow/lite/micro/kernels/svdf.o \
./tensorflow/tensorflow/lite/micro/kernels/unpack.o 


# Each subdirectory must supply rules for building sources it contributes
tensorflow/tensorflow/lite/micro/kernels/%.o tensorflow/tensorflow/lite/micro/kernels/%.su: ../tensorflow/tensorflow/lite/micro/kernels/%.cc tensorflow/tensorflow/lite/micro/kernels/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DMCU_SERIES_H7 -DSTM32H7 -c -I../Core/Inc -I../third_party/ruy -I../third_party/gemmlowp -I../third_party/flatbuffers/include -I../tensorflow -I../third_party -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tensorflow-2f-tensorflow-2f-lite-2f-micro-2f-kernels

clean-tensorflow-2f-tensorflow-2f-lite-2f-micro-2f-kernels:
	-$(RM) ./tensorflow/tensorflow/lite/micro/kernels/activations.d ./tensorflow/tensorflow/lite/micro/kernels/activations.o ./tensorflow/tensorflow/lite/micro/kernels/activations.su ./tensorflow/tensorflow/lite/micro/kernels/add.d ./tensorflow/tensorflow/lite/micro/kernels/add.o ./tensorflow/tensorflow/lite/micro/kernels/add.su ./tensorflow/tensorflow/lite/micro/kernels/all_ops_resolver.d ./tensorflow/tensorflow/lite/micro/kernels/all_ops_resolver.o ./tensorflow/tensorflow/lite/micro/kernels/all_ops_resolver.su ./tensorflow/tensorflow/lite/micro/kernels/arg_min_max.d ./tensorflow/tensorflow/lite/micro/kernels/arg_min_max.o ./tensorflow/tensorflow/lite/micro/kernels/arg_min_max.su ./tensorflow/tensorflow/lite/micro/kernels/ceil.d ./tensorflow/tensorflow/lite/micro/kernels/ceil.o ./tensorflow/tensorflow/lite/micro/kernels/ceil.su ./tensorflow/tensorflow/lite/micro/kernels/circular_buffer.d ./tensorflow/tensorflow/lite/micro/kernels/circular_buffer.o ./tensorflow/tensorflow/lite/micro/kernels/circular_buffer.su ./tensorflow/tensorflow/lite/micro/kernels/comparisons.d ./tensorflow/tensorflow/lite/micro/kernels/comparisons.o ./tensorflow/tensorflow/lite/micro/kernels/comparisons.su ./tensorflow/tensorflow/lite/micro/kernels/concatenation.d ./tensorflow/tensorflow/lite/micro/kernels/concatenation.o ./tensorflow/tensorflow/lite/micro/kernels/concatenation.su ./tensorflow/tensorflow/lite/micro/kernels/conv.d ./tensorflow/tensorflow/lite/micro/kernels/conv.o ./tensorflow/tensorflow/lite/micro/kernels/conv.su ./tensorflow/tensorflow/lite/micro/kernels/depthwise_conv.d ./tensorflow/tensorflow/lite/micro/kernels/depthwise_conv.o ./tensorflow/tensorflow/lite/micro/kernels/depthwise_conv.su ./tensorflow/tensorflow/lite/micro/kernels/dequantize.d ./tensorflow/tensorflow/lite/micro/kernels/dequantize.o ./tensorflow/tensorflow/lite/micro/kernels/dequantize.su ./tensorflow/tensorflow/lite/micro/kernels/elementwise.d ./tensorflow/tensorflow/lite/micro/kernels/elementwise.o ./tensorflow/tensorflow/lite/micro/kernels/elementwise.su ./tensorflow/tensorflow/lite/micro/kernels/floor.d ./tensorflow/tensorflow/lite/micro/kernels/floor.o ./tensorflow/tensorflow/lite/micro/kernels/floor.su ./tensorflow/tensorflow/lite/micro/kernels/fully_connected.d ./tensorflow/tensorflow/lite/micro/kernels/fully_connected.o ./tensorflow/tensorflow/lite/micro/kernels/fully_connected.su ./tensorflow/tensorflow/lite/micro/kernels/l2norm.d ./tensorflow/tensorflow/lite/micro/kernels/l2norm.o ./tensorflow/tensorflow/lite/micro/kernels/l2norm.su ./tensorflow/tensorflow/lite/micro/kernels/logical.d ./tensorflow/tensorflow/lite/micro/kernels/logical.o ./tensorflow/tensorflow/lite/micro/kernels/logical.su ./tensorflow/tensorflow/lite/micro/kernels/logistic.d ./tensorflow/tensorflow/lite/micro/kernels/logistic.o ./tensorflow/tensorflow/lite/micro/kernels/logistic.su ./tensorflow/tensorflow/lite/micro/kernels/maximum_minimum.d ./tensorflow/tensorflow/lite/micro/kernels/maximum_minimum.o ./tensorflow/tensorflow/lite/micro/kernels/maximum_minimum.su ./tensorflow/tensorflow/lite/micro/kernels/mul.d ./tensorflow/tensorflow/lite/micro/kernels/mul.o ./tensorflow/tensorflow/lite/micro/kernels/mul.su ./tensorflow/tensorflow/lite/micro/kernels/neg.d ./tensorflow/tensorflow/lite/micro/kernels/neg.o ./tensorflow/tensorflow/lite/micro/kernels/neg.su ./tensorflow/tensorflow/lite/micro/kernels/pack.d ./tensorflow/tensorflow/lite/micro/kernels/pack.o ./tensorflow/tensorflow/lite/micro/kernels/pack.su ./tensorflow/tensorflow/lite/micro/kernels/pad.d ./tensorflow/tensorflow/lite/micro/kernels/pad.o ./tensorflow/tensorflow/lite/micro/kernels/pad.su ./tensorflow/tensorflow/lite/micro/kernels/pooling.d ./tensorflow/tensorflow/lite/micro/kernels/pooling.o ./tensorflow/tensorflow/lite/micro/kernels/pooling.su ./tensorflow/tensorflow/lite/micro/kernels/prelu.d ./tensorflow/tensorflow/lite/micro/kernels/prelu.o ./tensorflow/tensorflow/lite/micro/kernels/prelu.su ./tensorflow/tensorflow/lite/micro/kernels/quantize.d ./tensorflow/tensorflow/lite/micro/kernels/quantize.o ./tensorflow/tensorflow/lite/micro/kernels/quantize.su ./tensorflow/tensorflow/lite/micro/kernels/reduce.d ./tensorflow/tensorflow/lite/micro/kernels/reduce.o ./tensorflow/tensorflow/lite/micro/kernels/reduce.su ./tensorflow/tensorflow/lite/micro/kernels/reshape.d ./tensorflow/tensorflow/lite/micro/kernels/reshape.o ./tensorflow/tensorflow/lite/micro/kernels/reshape.su ./tensorflow/tensorflow/lite/micro/kernels/resize_nearest_neighbor.d ./tensorflow/tensorflow/lite/micro/kernels/resize_nearest_neighbor.o ./tensorflow/tensorflow/lite/micro/kernels/resize_nearest_neighbor.su ./tensorflow/tensorflow/lite/micro/kernels/round.d ./tensorflow/tensorflow/lite/micro/kernels/round.o ./tensorflow/tensorflow/lite/micro/kernels/round.su ./tensorflow/tensorflow/lite/micro/kernels/softmax.d ./tensorflow/tensorflow/lite/micro/kernels/softmax.o ./tensorflow/tensorflow/lite/micro/kernels/softmax.su ./tensorflow/tensorflow/lite/micro/kernels/split.d ./tensorflow/tensorflow/lite/micro/kernels/split.o ./tensorflow/tensorflow/lite/micro/kernels/split.su ./tensorflow/tensorflow/lite/micro/kernels/strided_slice.d ./tensorflow/tensorflow/lite/micro/kernels/strided_slice.o ./tensorflow/tensorflow/lite/micro/kernels/strided_slice.su ./tensorflow/tensorflow/lite/micro/kernels/sub.d ./tensorflow/tensorflow/lite/micro/kernels/sub.o ./tensorflow/tensorflow/lite/micro/kernels/sub.su ./tensorflow/tensorflow/lite/micro/kernels/svdf.d ./tensorflow/tensorflow/lite/micro/kernels/svdf.o ./tensorflow/tensorflow/lite/micro/kernels/svdf.su ./tensorflow/tensorflow/lite/micro/kernels/unpack.d ./tensorflow/tensorflow/lite/micro/kernels/unpack.o ./tensorflow/tensorflow/lite/micro/kernels/unpack.su

.PHONY: clean-tensorflow-2f-tensorflow-2f-lite-2f-micro-2f-kernels


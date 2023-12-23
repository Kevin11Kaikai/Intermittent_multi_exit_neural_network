################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Cam/array.c \
../Core/Src/Cam/cam_hw_init.c \
../Core/Src/Cam/cam_main.c \
../Core/Src/Cam/cambus.c \
../Core/Src/Cam/collections.c \
../Core/Src/Cam/dma_alloc.c \
../Core/Src/Cam/fb_alloc.c \
../Core/Src/Cam/ff_wrapper.c \
../Core/Src/Cam/fmath.c \
../Core/Src/Cam/framebuffer.c \
../Core/Src/Cam/imlib.c \
../Core/Src/Cam/jpeg.c \
../Core/Src/Cam/mutex.c \
../Core/Src/Cam/ov7725.c \
../Core/Src/Cam/rainbow_tab.c \
../Core/Src/Cam/sensor.c \
../Core/Src/Cam/sensor_utils.c \
../Core/Src/Cam/umm_malloc.c \
../Core/Src/Cam/unaligned_memcpy.c \
../Core/Src/Cam/xalloc.c 

C_DEPS += \
./Core/Src/Cam/array.d \
./Core/Src/Cam/cam_hw_init.d \
./Core/Src/Cam/cam_main.d \
./Core/Src/Cam/cambus.d \
./Core/Src/Cam/collections.d \
./Core/Src/Cam/dma_alloc.d \
./Core/Src/Cam/fb_alloc.d \
./Core/Src/Cam/ff_wrapper.d \
./Core/Src/Cam/fmath.d \
./Core/Src/Cam/framebuffer.d \
./Core/Src/Cam/imlib.d \
./Core/Src/Cam/jpeg.d \
./Core/Src/Cam/mutex.d \
./Core/Src/Cam/ov7725.d \
./Core/Src/Cam/rainbow_tab.d \
./Core/Src/Cam/sensor.d \
./Core/Src/Cam/sensor_utils.d \
./Core/Src/Cam/umm_malloc.d \
./Core/Src/Cam/unaligned_memcpy.d \
./Core/Src/Cam/xalloc.d 

OBJS += \
./Core/Src/Cam/array.o \
./Core/Src/Cam/cam_hw_init.o \
./Core/Src/Cam/cam_main.o \
./Core/Src/Cam/cambus.o \
./Core/Src/Cam/collections.o \
./Core/Src/Cam/dma_alloc.o \
./Core/Src/Cam/fb_alloc.o \
./Core/Src/Cam/ff_wrapper.o \
./Core/Src/Cam/fmath.o \
./Core/Src/Cam/framebuffer.o \
./Core/Src/Cam/imlib.o \
./Core/Src/Cam/jpeg.o \
./Core/Src/Cam/mutex.o \
./Core/Src/Cam/ov7725.o \
./Core/Src/Cam/rainbow_tab.o \
./Core/Src/Cam/sensor.o \
./Core/Src/Cam/sensor_utils.o \
./Core/Src/Cam/umm_malloc.o \
./Core/Src/Cam/unaligned_memcpy.o \
./Core/Src/Cam/xalloc.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Cam/%.o Core/Src/Cam/%.su: ../Core/Src/Cam/%.c Core/Src/Cam/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DMCU_SERIES_H7 -DSTM32H7 -c -I../Core/Inc -I../third_party/ruy -I../third_party/gemmlowp -I../third_party/flatbuffers/include -I../tensorflow -I../third_party -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Cam

clean-Core-2f-Src-2f-Cam:
	-$(RM) ./Core/Src/Cam/array.d ./Core/Src/Cam/array.o ./Core/Src/Cam/array.su ./Core/Src/Cam/cam_hw_init.d ./Core/Src/Cam/cam_hw_init.o ./Core/Src/Cam/cam_hw_init.su ./Core/Src/Cam/cam_main.d ./Core/Src/Cam/cam_main.o ./Core/Src/Cam/cam_main.su ./Core/Src/Cam/cambus.d ./Core/Src/Cam/cambus.o ./Core/Src/Cam/cambus.su ./Core/Src/Cam/collections.d ./Core/Src/Cam/collections.o ./Core/Src/Cam/collections.su ./Core/Src/Cam/dma_alloc.d ./Core/Src/Cam/dma_alloc.o ./Core/Src/Cam/dma_alloc.su ./Core/Src/Cam/fb_alloc.d ./Core/Src/Cam/fb_alloc.o ./Core/Src/Cam/fb_alloc.su ./Core/Src/Cam/ff_wrapper.d ./Core/Src/Cam/ff_wrapper.o ./Core/Src/Cam/ff_wrapper.su ./Core/Src/Cam/fmath.d ./Core/Src/Cam/fmath.o ./Core/Src/Cam/fmath.su ./Core/Src/Cam/framebuffer.d ./Core/Src/Cam/framebuffer.o ./Core/Src/Cam/framebuffer.su ./Core/Src/Cam/imlib.d ./Core/Src/Cam/imlib.o ./Core/Src/Cam/imlib.su ./Core/Src/Cam/jpeg.d ./Core/Src/Cam/jpeg.o ./Core/Src/Cam/jpeg.su ./Core/Src/Cam/mutex.d ./Core/Src/Cam/mutex.o ./Core/Src/Cam/mutex.su ./Core/Src/Cam/ov7725.d ./Core/Src/Cam/ov7725.o ./Core/Src/Cam/ov7725.su ./Core/Src/Cam/rainbow_tab.d ./Core/Src/Cam/rainbow_tab.o ./Core/Src/Cam/rainbow_tab.su ./Core/Src/Cam/sensor.d ./Core/Src/Cam/sensor.o ./Core/Src/Cam/sensor.su ./Core/Src/Cam/sensor_utils.d ./Core/Src/Cam/sensor_utils.o ./Core/Src/Cam/sensor_utils.su ./Core/Src/Cam/umm_malloc.d ./Core/Src/Cam/umm_malloc.o ./Core/Src/Cam/umm_malloc.su ./Core/Src/Cam/unaligned_memcpy.d ./Core/Src/Cam/unaligned_memcpy.o ./Core/Src/Cam/unaligned_memcpy.su ./Core/Src/Cam/xalloc.d ./Core/Src/Cam/xalloc.o ./Core/Src/Cam/xalloc.su

.PHONY: clean-Core-2f-Src-2f-Cam


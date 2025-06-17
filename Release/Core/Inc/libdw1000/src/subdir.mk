################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/libdw1000/src/libdw1000.c \
../Core/Inc/libdw1000/src/libdw1000Spi.c 

OBJS += \
./Core/Inc/libdw1000/src/libdw1000.o \
./Core/Inc/libdw1000/src/libdw1000Spi.o 

C_DEPS += \
./Core/Inc/libdw1000/src/libdw1000.d \
./Core/Inc/libdw1000/src/libdw1000Spi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/libdw1000/src/%.o Core/Inc/libdw1000/src/%.su Core/Inc/libdw1000/src/%.cyclo: ../Core/Inc/libdw1000/src/%.c Core/Inc/libdw1000/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"C:/Users/rcosm/STM32CubeIDE/workspace_1.18.0/DECAWAVE/Core/Inc/libdw1000" -I"C:/Users/rcosm/STM32CubeIDE/workspace_1.18.0/DECAWAVE/Core/Inc/libdw1000/inc" -I"C:/Users/rcosm/STM32CubeIDE/workspace_1.18.0/DECAWAVE/Core/Inc/libdw1000/src" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Core/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-libdw1000-2f-src

clean-Core-2f-Inc-2f-libdw1000-2f-src:
	-$(RM) ./Core/Inc/libdw1000/src/libdw1000.cyclo ./Core/Inc/libdw1000/src/libdw1000.d ./Core/Inc/libdw1000/src/libdw1000.o ./Core/Inc/libdw1000/src/libdw1000.su ./Core/Inc/libdw1000/src/libdw1000Spi.cyclo ./Core/Inc/libdw1000/src/libdw1000Spi.d ./Core/Inc/libdw1000/src/libdw1000Spi.o ./Core/Inc/libdw1000/src/libdw1000Spi.su

.PHONY: clean-Core-2f-Inc-2f-libdw1000-2f-src


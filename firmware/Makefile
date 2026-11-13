PROJECT := FOC
BUILD_DIR := Debug
OBJ_DIR := $(BUILD_DIR)/obj

TOOLCHAIN := D:/STM32/STM32CubeIDE/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin/arm-none-eabi-
CC := $(TOOLCHAIN)gcc.exe
OBJCOPY := $(TOOLCHAIN)objcopy.exe
SIZE := $(TOOLCHAIN)size.exe

STARTUP := Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s
SRCS := \
  Core/Src/main.c Core/Src/adc.c Core/Src/can.c Core/Src/dma.c Core/Src/gpio.c \
  Core/Src/i2c.c Core/Src/stm32f1xx_hal_msp.c Core/Src/stm32f1xx_it.c \
  Core/Src/system_stm32f1xx.c Core/Src/tim.c Core/Src/usart.c \
  App/Src/Foc.c App/Src/can_proto.c App/Src/motor_ctrl.c App/Src/motor_msg.c App/Src/uart_proto.c \
  Bsp/Src/angle_as5600.c Bsp/Src/flash_storage.c Bsp/Src/pwm_3phase.c \
  Drv/Src/drv_adc.c Drv/Src/drv_can.c Drv/Src/drv_tim.c Drv/Src/led_indicator.c \
  Lib/Src/data_process.c Lib/Src/foc_math.c Lib/Src/observer_diff.c Lib/Src/observer_luenberger.c Lib/Src/pid.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c

OBJS := $(addprefix $(OBJ_DIR)/,$(SRCS:.c=.o)) $(OBJ_DIR)/$(STARTUP:.s=.o)
DEPS := $(OBJS:.o=.d)

INCLUDES := -ICore/Inc -IApp/Inc -IBsp/Inc -IDrv/Inc -ILib/Inc \
  -IDrivers/STM32F1xx_HAL_Driver/Inc \
  -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
  -IDrivers/CMSIS/Include

CFLAGS := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -O0 -g3 -Wall -Wextra \
  -ffunction-sections -fdata-sections -fstack-usage -MMD -MP \
  -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB $(INCLUDES)
LDFLAGS := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -TSTM32F103C8TX_FLASH.ld \
  -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map -Wl,--gc-sections \
  --specs=nano.specs --specs=nosys.specs -Wl,-u,_printf_float

.PHONY: all clean flash

all: $(BUILD_DIR)/$(PROJECT).bin

$(BUILD_DIR)/$(PROJECT).elf: $(OBJS) STM32F103C8TX_FLASH.ld

	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"
	$(CC) $(OBJS) $(LDFLAGS) -Wl,--start-group -lc -lm -lnosys -Wl,--end-group -o $@
	$(SIZE) $@

$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf

	$(OBJCOPY) -O binary $< $@

$(OBJ_DIR)/%.o: %.c

	@if not exist "$(dir $@)" (mkdir "$(dir $@)" 2>NUL || exit 0)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: %.s

	@if not exist "$(dir $@)" (mkdir "$(dir $@)" 2>NUL || exit 0)
	$(CC) -mcpu=cortex-m3 -mthumb -x assembler-with-cpp -c $< -o $@

clean:

	@if exist "$(BUILD_DIR)" rmdir /S /Q "$(BUILD_DIR)"

-include $(DEPS)

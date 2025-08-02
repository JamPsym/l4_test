# Define paths
CUBEL4_DIR = ../lib/STM32CubeL4
TOOLCHAIN = arm-none-eabi-
CC = $(TOOLCHAIN)gcc
LD = $(TOOLCHAIN)ld
OBJCOPY = $(TOOLCHAIN)objcopy
SIZE = $(TOOLCHAIN)size

BUILD_DIR = build

MCU = -mcpu=cortex-m4 -mthumb
DEFS = -DSTM32L432xx -DUSE_FULL_LL_DRIVER

# Include paths (CMSIS and LL)
INCLUDES = \
	-Iinc \
	-I$(CUBEL4_DIR)/Drivers/CMSIS/Device/ST/STM32L4xx/Include \
	-I$(CUBEL4_DIR)/Drivers/CMSIS/Include \
	-I$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Inc

# Sources: main.c + CMSIS system/startup files
CSRCS = \
	main.c \
	system_stm32l4xx.c

ASRCS = startup_stm32l432xx.s

# LL sources
LLSRCS = \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_adc.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_comp.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_crc.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_crs.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dac.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma2d.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_i2c.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lptim.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lpuart.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_opamp.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pka.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pwr.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rng.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rtc.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_swpmi.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c \
	$(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c

# Linker script
LDSCRIPT = STM32L432KCUX_FLASH.ld  # In project root

CFLAGS = $(MCU) -g3 -O0 -Wall -std=c17 -Wpedantic $(DEFS) $(INCLUDES)
ASFLAGS = $(MCU) -g3 -x assembler-with-cpp --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LDFLAGS = $(MCU) -T$(LDSCRIPT) -Wl,--gc-sections --specs=nano.specs

COBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(CSRCS))
AOBJS = $(patsubst %.s,$(BUILD_DIR)/%.o,$(ASRCS))
LLOBJS = $(patsubst $(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/%.c,$(BUILD_DIR)/%.o,$(LLSRCS))


all: app.elf

app.elf: $(COBJS) $(AOBJS) $(LLOBJS)
	$(CC) $(LDFLAGS) $^ -o $@
	$(OBJCOPY) -O binary $@ app.bin
	$(SIZE) $@

$(BUILD_DIR)/%.o: %.c
	mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	mkdir -p $(@D)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(CUBEL4_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/%.c
	mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) app.elf app.bin

# Optional: Init submodules if needed
submodules:
	git submodule update --init --recursive

flash: app.elf
	gdb-multiarch -ex 'target extended-remote localhost:4242' -ex 'load' -ex 'monitor reset halt' -ex 'monitor flash write_image erase' -ex 'monitor reset' -ex 'quit' app.elf

.PHONY: all clean submodules flash

gdb: app.elf
	gdb-multiarch -ex 'target extended-remote localhost:4242' -ex 'load' -ex 'break main' -ex 'svd_load STM32L432.svd' app.elf
# STM32H7 Makefile for GCC

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
AR = $(PREFIX)ar
LD = $(PREFIX)gcc
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# MCU
MCU = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb

# Defines
DEFS = -DUSE_HAL_DRIVER -DSTM32H723xx

# Include directories
INCLUDES = -ICore/Inc \
           -IDrivers/STM32H7xx_HAL_Driver/Inc \
           -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
           -IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
           -IDrivers/CMSIS/Include

# Compile flags
CFLAGS = $(MCU) $(DEFS) $(INCLUDES) -O2 -Wall -fdata-sections -ffunction-sections -g -gdwarf-2
CXXFLAGS = $(CFLAGS) -fno-rtti -fno-exceptions
ASFLAGS = $(MCU) $(DEFS) $(INCLUDES) -Wall -fdata-sections -ffunction-sections
LDFLAGS = $(MCU) -specs=nano.specs -TSTM32H723VGTx_FLASH.ld -lc -lm -lnosys -Wl,-Map=build/H7TEST.map,--cref -Wl,--gc-sections

# Source files
C_SOURCES = $(wildcard Core/Src/*.c) \
            $(wildcard Drivers/STM32H7xx_HAL_Driver/Src/*.c)

# 排除重复或冲突的文件
C_SOURCES := $(filter-out %/crc_ccitt.c %/gom_protocol.c %/unitree_motor.c,$(C_SOURCES))
C_SOURCES += Core/Src/crc_ccitt.c Core/Src/gom_protocol.c Core/Src/unitree_motor.c

ASM_SOURCES = Core/Startup/startup_stm32h723xx_gcc.s

# Object files
OBJECTS = $(addprefix build/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix build/,$(notdir $(ASM_SOURCES:.s=.o)))

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Targets
all: build/H7TEST.elf build/H7TEST.hex build/H7TEST.bin

build/%.o: %.c Makefile | build
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=build/$(notdir $(<:.c=.lst)) $< -o $@

build/%.o: %.s Makefile | build
	$(AS) -c $(ASFLAGS) $< -o $@

build/H7TEST.elf: $(OBJECTS) Makefile
	$(LD) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

build/H7TEST.hex: build/H7TEST.elf | build
	$(HEX) $< $@

build/H7TEST.bin: build/H7TEST.elf | build
	$(BIN) $< $@

build:
	mkdir -p build

clean:
	rm -rf build

flash: build/H7TEST.elf
	openocd -f interface/jlink.cfg -f target/stm32h7x.cfg -c "program $< verify reset exit"

debug: build/H7TEST.elf
	openocd -f interface/jlink.cfg -f target/stm32h7x.cfg

.PHONY: all clean flash debug

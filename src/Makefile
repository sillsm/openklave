# path to STM32F103 standard peripheral library
STD_PERIPH_LIBS ?= ./STM32F10x_StdPeriph_Lib_V3.5.0/

# list of source files
SOURCES  = main.c
SOURCES += $(STD_PERIPH_LIBS)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
SOURCES += $(STD_PERIPH_LIBS)/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
SOURCES += $(STD_PERIPH_LIBS)/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c
SOURCES += $(STD_PERIPH_LIBS)/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c

SOURCES += $(STD_PERIPH_LIBS)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/TrueSTUDIO/startup_stm32f10x_md.s

# name for output binary files
PROJECT ?= led

# compiler, objcopy (should be in PATH)
CC = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy

# path to st-flash (or should be specified in PATH)
ST_FLASH ?= st-flash

# specify compiler flags
CFLAGS  = -g -O2 -Wall -Wunused --specs=nosys.specs -std=c++17
#CFLAGS += -T$(STD_PERIPH_LIBS)/Project/STM32F10x_StdPeriph_Template/TrueSTUDIO/STM3210B-EVAL/stm32_flash.ld
CFLAGS += -T stm32_flash.ld
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
CFLAGS += -Wl,--gc-sections
CFLAGS += -I.
CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/
CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/CMSIS/CM3/CoreSupport
CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/STM32F10x_StdPeriph_Driver/inc

OBJS = $(SOURCES:.c=.o)

all: $(PROJECT).elf MakeAkaiSysex

# compile
$(PROJECT).elf: $(SOURCES)
	$(CC) $(CFLAGS) $^ -o $@
	$(OBJCOPY) -v -O ihex $(PROJECT).elf $(PROJECT).hex
	$(OBJCOPY) -O binary $(PROJECT).elf $(PROJECT).bin
	$(OBJCOPY) --input-target=binary --output-target=ihex $(PROJECT).bin try.hex

.PHONY: MakeAkaiSysex
MakeAkaiSysex:
	printf "\xF0\x47\x00\x24\x70" | cat - $(PROJECT).hex > $(PROJECT).syx
	printf "\xF7" >> $(PROJECT).syx
# remove binary files
clean:
	rm -f *.o *.elf *.hex *.bin *.syx

# flash
burn:
	sudo $(ST_FLASH) write $(PROJECT).bin 0x8000000

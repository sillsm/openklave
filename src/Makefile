# path to STM32F103 standard peripheral library
STD_PERIPH_LIBS ?= ./STM32F10x_StdPeriph_Lib_V3.5.0/
THIRD_PARTY ?= ./third_party/
CONFIG ?= ./config/

# list of source files

SOURCES  = kernel.c
SOURCES += $(CONFIG)/system_stm32f10x.c
SOURCES += $(THIRD_PARTY)/misc.c

SOURCES += $(CONFIG)/mpk249start.s

TESTBIN= test_kernel.c
MAINBIN= main.c

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
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3 -mthumb-interwork
CFLAGS += -mfloat-abi=soft -mfpu=fpv4-sp-d16
CFLAGS += -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
CFLAGS += -Wl,--gc-sections
CFLAGS += -I.
CFLAGS += -I$(THIRD_PARTY)/
#CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/
#CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/CMSIS/CM3/CoreSupport
#CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/STM32F10x_StdPeriph_Driver/inc

#OBJS = $(SOURCES:.c=.o)

all: $(PROJECT).elf MakeAkaiSysex test.elf

# compile
$(PROJECT).elf: $(SOURCES) $(MAINBIN)
	$(CC) $(CFLAGS) $^ -o $@
	$(OBJCOPY) -v -O ihex $(PROJECT).elf $(PROJECT).hex
	$(OBJCOPY) -O binary $(PROJECT).elf $(PROJECT).bin
	$(OBJCOPY) --input-target=binary --output-target=ihex $(PROJECT).bin try.hex
	$(OBJCOPY) --only-keep-debug $(PROJECT).elf main.debug

test.elf: $(TESTBIN)
	$(CC) -w -c $(CFLAGS) $^ -o $@
	@$(OBJCOPY) -v -O ihex test.elf test.hex


.PHONY: MakeAkaiSysex
MakeAkaiSysex:
	printf "\xF0\x47\x00\x24\x70" | cat - $(PROJECT).hex > $(PROJECT).syx
	printf "\xF7" >> $(PROJECT).syx
# remove binary files
clean:
	rm -f *.o *.elf *.hex *.bin *.syx *.debug

# flash
burn:
	clear
	echo "Please make sure you have turned off the device and\n"
	echo "turned it back on with the Push To Enter knob held down."
	python ../util/wipe_os.py
	sleep 5
	ls -lh $(PROJECT).syx
	python ../util/flash_os.py --no-prompt $(PROJECT).syx
	echo "Device sucessfully flashed, please restart it.\n\n"

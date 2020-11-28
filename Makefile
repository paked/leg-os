PREFIX=arm-none-eabi

CFLAGS=-mcpu=cortex-m4 -mthumb -Wall -Werror -O0 -gdwarf-4 -g3 -Iinclude
LDFLAGS=-nostdlib -nostartfiles -T link.ld

CC=$(PREFIX)-gcc
LD=$(PREFIX)-ld
OBJCOPY=$(PREFIX)-objcopy

SRCS := $(shell find $(SRC_DIRS) -name *.c -or -name *.S)
OBJS := $(addsuffix .o,$(basename $(SRCS)))

TARGET ?= kernel.elf

$(TARGET): $(OBJS)
	$(LD) $(LDFLAGS) $^ -o $@

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

%.o: %.S
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: flash
flash: 
	$(OBJCOPY) -O binary $(TARGET) kernel.bin
	st-flash --reset write kernel.bin 0x8000000

.PHONY: debug
debug: 
	arm-none-eabi-gdb $(TARGET)

.PHONY: clean
clean:
	rm -f $(OBJS) $(TARGET) *.bin

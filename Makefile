PREFIX=arm-none-eabi

CFLAGS=-mcpu=cortex-m4 -mthumb -Wall -Werror -O0 -gdwarf-4 -g3 -Ilib
LINKER_FLAGS=-T link.ld

CC=$(PREFIX)-gcc
LD=$(PREFIX)-ld
OBJCOPY=$(PREFIX)-objcopy

OBJ=start.o kernel.o user.o syscall.o

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

%.o: %.S
	$(CC) $(CFLAGS) -o $@ -c $<

build: $(OBJ)
	$(LD) -nostdlib -nostartfiles $(LINKER_FLAGS) $(OBJ) -o kernel.elf
	$(OBJCOPY) -O binary kernel.elf kernel.bin

.PHONY: flash
flash: build
	st-flash --reset write kernel.bin 0x8000000

.PHONY: debug
debug: build
	arm-none-eabi-gdb kernel.elf

.PHONY: clean
clean:
	rm *.o *.elf *.bin

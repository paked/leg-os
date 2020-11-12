PREFIX=arm-none-eabi

CFLAGS="-mcpu=cortex-m4 -mthumb -Wall -O2 -ggdb"

CC=$PREFIX-gcc
LD=$PREFIX-ld
OBJCOPY=$PREFIX-objcopy

SRC="start.S kernel.c"

$CC $CFLAGS -c $SRC

$LD -nostdlib -nostartfiles start.o kernel.o -T link.ld -o kernel.elf

$OBJCOPY -O binary kernel.elf kernel.bin

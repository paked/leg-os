target extended-remote localhost:4242

load kernel.elf

monitor reset

tbreak kernel_main

layout src

target extended-remote localhost:4242

load kernel.elf

monitor reset

break HardFault_Handler
tbreak kernel_main

layout src

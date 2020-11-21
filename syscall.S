.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global sys_putchar, sys_toggle_red, sys_toggle_green

.type sys_toggle_red, %function
sys_toggle_red:
  svc 0

  bx lr
.size sys_toggle_red, .-sys_putchar

.type sys_toggle_green, %function
sys_toggle_green:
  svc 1

  bx lr
.size sys_toggle_green, .-sys_toggle_green

.type sys_putchar, %function
sys_putchar:
  svc 2

  bx lr
.size sys_putchar, .-sys_putchar

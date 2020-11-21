#include "user.h"

#include "syscall.h"

#include <stdbool.h>

int user_putchar(int c);

#define SIMPLE_PRINTF_PUTCHAR user_putchar
#include "printf.h"

int user_putchar(int c) {
    if (c == '\n') {
        user_putchar('\r');
    }

    sys_putchar(c);

    return 0;
}

// trigger red led
void fn_process_1() {
    while (true) {
        /*
        take_lock(&lock1);
        printf("hello from process %d\n", get_current_pid());
        release_lock(&lock1);
        */

        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }

        printf("blinking red LED\n");

        sys_toggle_red();
    }
}

// trigger green led
void fn_process_2() {
    sys_toggle_green();

    while (true) {
        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }

        sys_toggle_green();
    }
}

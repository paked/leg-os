#include "user/user.h"
#include "user/syscall.h"

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
        uint32_t start_time = sys_millis();

        /*
        take_lock(&lock1);
        printf("hello from process %d\n", get_current_pid());
        release_lock(&lock1);
        */

        uint32_t now_time = sys_millis();
        while (now_time < start_time + 1000) {
            now_time = sys_millis();
        }

        // printf("blinking red LED\n");

        sys_toggle_red();
    }
}

// trigger green led
void fn_process_2() {
    while (true) {
        uint32_t start_time = sys_millis();
        uint32_t now_time = sys_millis();

        printf("hello world!\n");
        while (now_time < start_time + 1000) {
            now_time = sys_millis();
        }

        sys_toggle_green();
    }
}

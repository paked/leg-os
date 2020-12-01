#include "user/user.h"
#include "user/syscall.h"

#define printf(fmt, ...) xprintf(&user_write, fmt, ##__VA_ARGS__)
#include "xprintf.h"

#define BEFORE(a, period) (sys_millis() - a < period)


int user_putchar(int c) {
    if (c == '\n') {
        user_putchar('\r');
    }

    sys_putchar(c);

    return 0;
}

uint32_t user_write(char *buf, uint32_t len) {
	int i = 0;
	for (; i < len; i++) {
		user_putchar(buf[i]);
	}

	return i;
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

        uint32_t period = 1000;
        while (BEFORE(start_time, period));

        // printf("blinking red LED\n");

        sys_toggle_red();
    }
}

// trigger green led
void fn_process_2() {
    while (true) {
        uint32_t start_time = sys_millis();

        printf("hello world!\n");
        while (BEFORE(start_time, 1000));

        sys_toggle_green();
    }
}

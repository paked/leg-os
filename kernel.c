#include <stdint.h>
#include <stddef.h>

#define WRITE(reg, data) *(volatile uint32_t*)reg = data
#define READ(reg) *(volatile uint32_t*)reg

#define SET_BIT(reg, i) *(volatile uint32_t*)reg |= 1 << i;
#define CLEAR_BIT(reg, i) *(volatile uint32_t*)reg &= ~(1 << i);

enum {
    RCC_BASE = 0x40021000,
    RCC_AHB2ENR = RCC_BASE + 0x4C,

    GPIOx_MODER_OFFS = 0x0,
    GPIOx_ODR_OFFS = 0x14,

    GPIOB_BASE = 0x48000400,

    GPIOB_MODER = GPIOB_BASE + GPIOx_MODER_OFFS,
    GPIOB_ODR = GPIOB_BASE + GPIOx_ODR_OFFS
};

void kernel_main() {
    //
    // turn on the red LED on pin PB2 (port B, pin 2)
    //

    // enable clock for Port B
    SET_BIT(RCC_AHB2ENR, 1);

    // set mode for pin 2 (01 for general purpose)
    CLEAR_BIT(GPIOB_MODER, 5);
    SET_BIT(GPIOB_MODER, 4);

    // turn on red led
    SET_BIT(GPIOB_ODR, 2);

    while (1);
}

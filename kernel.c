#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define WRITE(reg, data) *(volatile uint32_t *)reg = data
#define READ(reg) *(volatile uint32_t *)reg

#define SET_BIT(reg, i) *(volatile uint32_t *)reg |= 1 << i;
#define CLEAR_BIT(reg, i) *(volatile uint32_t *)reg &= ~(1 << i);

// #define WRITE_BITS(reg, start, len, data) *(volatile uint32_t *)reg |= data << i;

enum
{
    RCC_BASE = 0x40021000,
    RCC_AHB2ENR = RCC_BASE + 0x4C,
    RCC_APB2ENR = RCC_BASE + 0x60,
    RCC_CFGR = RCC_BASE + 0x08,
    RCC_CR = RCC_BASE + 0,

    GPIOx_MODER_OFFS = 0x0,
    GPIOx_ODR_OFFS = 0x14,

    GPIOB_BASE = 0x48000400,

    GPIOB_MODER = GPIOB_BASE + GPIOx_MODER_OFFS,
    GPIOB_ODR = GPIOB_BASE + GPIOx_ODR_OFFS,

    USART_BASE = 0x40013800,
    USART_CR1 = USART_BASE + 0,
    USART_BRR = USART_BASE + 0x0C,
};

void kernel_main()
{
    //
    // turn on the red LED on pin PB2 (port B, pin 2)
    //

    // enable clock for Port B
    SET_BIT(RCC_AHB2ENR, 1);

    // set mode for pin 2 (01 for general purpose)
    CLEAR_BIT(GPIOB_MODER, 5);
    SET_BIT(GPIOB_MODER, 4);

    // 11 = 8 + 2 + 1 = 1011
    // set system clock to 80mHZ!!!
    // SET_BIT(RCC_CFGR, 0)
    SET_BIT(RCC_CR, 4)
    SET_BIT(RCC_CR, 5)
    CLEAR_BIT(RCC_CR, 6)
    SET_BIT(RCC_CR, 7)

    SET_BIT(RCC_CR, 3)

    bool led = true;
    while (true)
    {
        for (uint32_t i = 0; i < 400000; i++)
        {
            asm volatile("nop");
        }
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");

        if (led = !led)
        {
            SET_BIT(GPIOB_ODR, 2);
        }
        else
        {
            CLEAR_BIT(GPIOB_ODR, 2);
        }
    }

    // turn on red led
    SET_BIT(GPIOB_ODR, 2);

    // enable clock for usart
    SET_BIT(RCC_APB2ENR, 14)

    // set tx word length to 8 bits
    SET_BIT(USART_CR1, 12)
    SET_BIT(USART_CR1, 28)

    // set baud

    while (1)
        ;
}

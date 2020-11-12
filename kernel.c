#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define WRITE(reg, data) *(volatile uint32_t *)reg = data
#define READ(reg) *(volatile uint32_t *)reg

#define SET_BIT(reg, i) *(volatile uint32_t *)reg |= 1 << i;
#define CLEAR_BIT(reg, i) *(volatile uint32_t *)reg &= ~(1 << i);

void copy_bits(uint32_t reg, uint32_t start, uint32_t size, uint32_t data)
{
    volatile uint32_t *reg_vol = (uint32_t *)reg;
    uint32_t cur_val = *reg_vol;

    uint32_t mask = ~(~0x0 << (32 - start - size) >> (32 - start) << size);
    cur_val &= mask;

    data = data << start;
    cur_val |= data;

    *reg_vol = cur_val;
}

enum
{
    RCC_BASE = 0x40021000,
    RCC_AHB2ENR = RCC_BASE + 0x4C,
    RCC_APB2ENR = RCC_BASE + 0x60,
    RCC_APB1ENR1 = RCC_BASE + 0x58,
    RCC_CFGR = RCC_BASE + 0x08,
    RCC_CR = RCC_BASE + 0,
    RCC_CSR = RCC_BASE + 0x94,

    GPIOx_MODER_OFFS = 0x0,
    GPIOx_ODR_OFFS = 0x14,

    GPIOA_BASE = 0x48000000,
    GPIOA_AFRL = GPIOA_BASE + 0x20,

    GPIOB_BASE = 0x48000400,

    GPIOB_MODER = GPIOB_BASE + GPIOx_MODER_OFFS,
    GPIOB_ODR = GPIOB_BASE + GPIOx_ODR_OFFS,

    USART2_BASE = 0x40004400,
    USART2_CR1 = USART2_BASE + 0x0,
    USART2_CR2 = USART2_BASE + 0x4,
    USART2_BRR = USART2_BASE + 0x0C,
    USART2_TDR = USART2_BASE + 0x28,
    USART2_ISR = USART2_BASE + 0x1C,
    USART2_RDR = USART2_BASE + 0x24,
};

void kernel_main()
{
    //
    // turn on the red LED on pin PB2 (port B, pin 2)
    //

    // enable clock for Port B
    SET_BIT(RCC_AHB2ENR, 1);
    // enable clock for Port A
    SET_BIT(RCC_AHB2ENR, 0);
    // enable clock for USART2
    SET_BIT(RCC_APB1ENR1, 17);

    // enabling alternate 7 for port A pins 2 and 3
    // copy_bits(GPIOA_AFRL, 8, 4, 7);
    // copy_bits(GPIOA_AFRL, 12, 4, 7);
    WRITE(GPIOA_AFRL, (7 << 8) | (7 << 12));

    // set mode for pin 2 (01 for general purpose)
    CLEAR_BIT(GPIOB_MODER, 5);
    SET_BIT(GPIOB_MODER, 4);

    // set system clock to 80mHZ!!!
    copy_bits(RCC_CR, 4, 4, 11);
    // save changes to RCC_CR
    SET_BIT(RCC_CR, 3)

    /*
    bool led = true;
    while (true)
    {
        for (uint32_t i = 0; i < 400000; i++)
        {
            asm volatile("nop");
        }

        if ((led = !led))
        {
            SET_BIT(GPIOB_ODR, 2);
        }
        else
        {
            CLEAR_BIT(GPIOB_ODR, 2);
        }
    }
    */

    // turn on red led
    // SET_BIT(GPIOB_ODR, 2);

    // set tx word length to 8 bits
    // CLEAR_BIT(USART_CR1, 12)
    // CLEAR_BIT(USART_CR1, 28)

    // set BRR to 9600 baud. default value for BRR is 0 so we can hack it and
    // just or 5000 in.
    // WRITE(USART2_BRR, READ(USART2_BRR) | 5000);

    // program the amount of stop bits in USART_CR2 the default is 1 stop bit,
    // i don't see a problem with that so we can leave it.

    // set TE bit
    // SET_BIT(USART2_CR1, 3);

    // enable auto baud
    SET_BIT(USART2_CR2, 20);

    // set RE bit
    SET_BIT(USART2_CR1, 2);

    // enable UE -- start usart-ing
    SET_BIT(USART2_CR1, 0);

    while (true)
    {
        uint32_t rx = READ(USART2_ISR) & (1 << 5);
        // asm volatile("dmb");
        if (rx)
        {
            volatile uint32_t rdr = READ(USART2_RDR);
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
        }
    }

    // while (true) {
    //     // transmit stuff
    //     uint32_t tdr = READ(USART_TDR);
    //     tdr &= ~0xFF;
    //     tdr |= 'F';
    //     WRITE(USART_TDR, tdr);

    //     // check if finished sending (check if TC=1)
    //     while (true) {
    //         // zero is false
    //         // 1 is true
    //         uint32_t tc = READ(USART_ISR) & (1 << 6);
    //         if (tc) {
    //             break;
    //         }
    //     }
    // }

    CLEAR_BIT(GPIOB_ODR, 2);

    while (true)
        ;
}

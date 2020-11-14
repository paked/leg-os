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
    RCC_CCIPR = RCC_BASE + 0x88,
    RCC_CSR = RCC_BASE + 0x94,

    GPIOx_MODER_OFFS = 0x0,
    GPIOx_ODR_OFFS = 0x14,
    GPIOx_OSPEEDR_OFFS = 0x08,
    GPIOx_PUPDR_OFFS = 0x0C,

    GPIOA_BASE = 0x48000000,
    GPIOA_MODER = GPIOA_BASE + GPIOx_MODER_OFFS,
    GPIOA_OSPEEDR = GPIOA_BASE + GPIOx_OSPEEDR_OFFS,
    GPIOA_AFRL = GPIOA_BASE + 0x20,
    GPIOA_PUPDR = GPIOA_BASE + GPIOx_PUPDR_OFFS,

    GPIOB_BASE = 0x48000400,

    GPIOB_MODER = GPIOB_BASE + GPIOx_MODER_OFFS,
    GPIOB_ODR = GPIOB_BASE + GPIOx_ODR_OFFS,

    USART2_BASE = 0x40004400,
    USART2_CR1 = USART2_BASE + 0x0,
    USART2_CR2 = USART2_BASE + 0x4,
    USART2_BRR = USART2_BASE + 0x0C,
    USART2_ICR = USART2_BASE + 0x20,
    USART2_TDR = USART2_BASE + 0x28,
    USART2_ISR = USART2_BASE + 0x1C,
    USART2_RDR = USART2_BASE + 0x24,

    FLASH_BASE = 0x40022000,
    FLASH_ACR = FLASH_BASE + 0x0,
};

void kernel_main()
{

    // set system clock to 48MHZ!!!

    // update flash latency to correct values (to prevent flash reading
    // fuckery)
    CLEAR_BIT(FLASH_ACR, 0);
    SET_BIT(FLASH_ACR, 1);
    CLEAR_BIT(FLASH_ACR, 2);

    // first 3 bytes == 0b111
    while ((READ(FLASH_ACR) & 7) != 2)
        ;

    // set msi to mode 11 (48MHZ)
    // copy_bits(RCC_CR, 4, 4, 11);
    SET_BIT(RCC_CR, 4)
    SET_BIT(RCC_CR, 5)
    CLEAR_BIT(RCC_CR, 6)
    SET_BIT(RCC_CR, 7)
    // save changes to MSI clock
    SET_BIT(RCC_CR, 3)

    SET_BIT(RCC_CR, 0); // make sure MSI is on

    while (!(READ(RCC_CR) & (1 << 1)))
        ;

    // enable clock for Port A
    // enable clock for Port B
    WRITE(RCC_AHB2ENR, READ(RCC_AHB2ENR) | 0b11);

    // use the system clock for usart 2
    // CLEAR_BIT(RCC_CCIPR, 3);
    // CLEAR_BIT(RCC_CCIPR, 2);

    // enable clock for USART2
    SET_BIT(RCC_APB1ENR1, 17);

    /*
    Peripheral alternate function:
    -  Connect the I/O to the desired AFx in one of the GPIOx_AFRL or GPIOx_AFRH
       register.
    –  Select the type, pull-up/pull-down and output speed via the GPIOx_OTYPER,
       GPIOx_PUPDR and GPIOx_OSPEEDER registers, respectively.
    –  Configure the desired I/O as an alternate function in the GPIOx_MODER register.
    */

    WRITE(GPIOA_AFRL, (7 << 8) | (7 << 12));

    // output speed (?) = very high
    SET_BIT(GPIOA_OSPEEDR, 6)
    SET_BIT(GPIOA_OSPEEDR, 7)

    SET_BIT(GPIOA_PUPDR, 6);

    // enabling alternate 7 for port A pins 2 and 3
    // copy_bits(GPIOA_AFRL, 8, 4, 7);
    // copy_bits(GPIOA_AFRL, 12, 4, 7);
    // PA2 - tx
    CLEAR_BIT(GPIOA_MODER, 2);
    SET_BIT(GPIOA_MODER, 3);
    // PA3 - rx
    SET_BIT(GPIOA_MODER, 4);
    CLEAR_BIT(GPIOA_MODER, 5);

    // set mode for pin 2 (01 for general output purpose)
    CLEAR_BIT(GPIOB_MODER, 5);
    SET_BIT(GPIOB_MODER, 4);

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
    // set tx word length to 8 bits
    // CLEAR_BIT(USART_CR1, 12)
    // CLEAR_BIT(USART_CR1, 28)

    // set BRR to 9600 baud. default value for BRR is 0 so we can hack it and
    // just or 5000 in.
    WRITE(USART2_BRR, READ(USART2_BRR) | 5000);

    // program the amount of stop bits in USART_CR2 the default is 1 stop bit,
    // i don't see a problem with that so we can leave it.

    // set TE bit
    SET_BIT(USART2_CR1, 3);

    // enable auto baud
    // SET_BIT(USART2_CR2, 20);

    // set RE bit
    SET_BIT(USART2_CR1, 2);

    // enable UE -- start usart-ing
    SET_BIT(USART2_CR1, 0);

    while (true)
    {
        uint32_t isr = READ(USART2_ISR);
        uint32_t rx = isr & (1 << 5);
        if ((isr & 2) == 2)
        {
            SET_BIT(GPIOB_ODR, 2);
            asm volatile("nop");
            SET_BIT(USART2_ICR, 1); // clear framing error
        }
        else
        {
            CLEAR_BIT(GPIOB_ODR, 2);
        }

        // asm volatile("dmb");
        if (rx)
        {
            volatile uint32_t rdr = READ(USART2_RDR);
            // SET_BIT(GPIOB_ODR, 2);

            uint32_t tdr = READ(USART2_TDR);
            tdr &= ~0xFF;
            tdr |= rdr & '0xFF';

            WRITE(USART2_TDR, tdr);

            // check if finished sending (check if TC=1)
            while (true)
            {
                //    zero is false
                //      1 is true
                uint32_t tc = READ(USART2_ISR) & (1 << 6);
                if (tc)
                {
                    break;
                }
            }
        }
    }

    /*
    while (true) {
        // transmit stuff
        uint32_t tdr = READ(USART2_TDR);
        tdr &= ~0xFF;
        tdr |= 'F';
        WRITE(USART2_TDR, tdr);

        // check if finished sending (check if TC=1)
        while (true) {
            //    zero is false
            //      1 is true
            uint32_t isr = READ(USART2_ISR);
            uint32_t tc = isr & (1 << 6);
            if ((isr & 2) == 2) {
                SET_BIT(GPIOB_ODR, 2);

                SET_BIT(USART2_ICR, 1); // clear framing error
            } else {
                CLEAR_BIT(GPIOB_ODR, 2);
            }

            if (tc) {
                break;
            }
        }
    }
    */

    while (true)
        ;
}

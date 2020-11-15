#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32l476xx.h"

// Blocking USART send
void usart_send(char *data, uint32_t size) {
    // will need a lock on the usart port

    for (uint32_t i = 0; i < size; i++) {
        while (!(USART2->ISR & USART_ISR_TC)) {}

        USART2->TDR = (USART2->TDR & ~USART_TDR_TDR_Msk) | (uint32_t)(data[i]);
    }
}

void kernel_main(void) {
    FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;

    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_2WS) {}

    RCC->CR &= ~RCC_CR_MSIRANGE_Msk;
    RCC->CR |= RCC_CR_MSIRANGE_11;

    RCC->CR |= RCC_CR_MSIRGSEL;

    while ((RCC->CR & RCC_CR_MSIRDY) != RCC_CR_MSIRDY) {}

    // enable port D (for usart rx/tx)
    // enable port b (for red LED)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIODEN;

    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    // enable alternate mode 7 for pin 5 and pin 6 of port D
    GPIOD->AFR[0] |= (7 << GPIO_AFRL_AFSEL5_Pos) | (7 << GPIO_AFRL_AFSEL6_Pos);

    // set pins 5 and 6 to alternate mode (10)

    GPIOD->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6);
    GPIOD->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1;

    // set mode for red led (pin 2) to general purpose output (01)
    GPIOB->MODER &= ~GPIO_MODER_MODE2;
    GPIOB->MODER |= GPIO_MODER_MODE2_0;

    /*
    int led = 0;
    while (true) {
        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }

        if (led) {
            GPIOB->ODR |= GPIO_ODR_OD2;
        } else {
            GPIOB->ODR &= ~GPIO_ODR_OD2;
        }

        led = !led;
    }
    */

    // 5000 happens to be the number we need to reach 9600 baud.
    // probably should be a bit cleverer here, but oh well---we'll
    // just OR it in.
    USART2->BRR |= 5000;

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART2->CR1 |= USART_CR1_UE;

    while (true) {
        if (!(USART2->ISR & USART_ISR_RXNE)) {
            continue;
        }
        char recv = (char)(USART2->RDR & 0xFF);
        usart_send(&recv, 1);
    }

    while (true) {}
}

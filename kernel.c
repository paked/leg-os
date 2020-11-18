#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// #define TEST // define if testing usart printing
#include "printf.c"
#include "stm32l476xx.h"

#define USART_TTY USART2

#define DUMP_REGS                                        \
    uint32_t *stack_pointer;                             \
    uint32_t flags_reg;                                  \
    asm volatile("push {r0-r12,lr}\n\tmov %[result], sp" \
                 : [ result ] "=r"(stack_pointer));      \
    asm volatile("mrs %[result], APSR"                   \
                 : [ result ] "=r"(flags_reg));          \
    printf("General Register Dump\n");                   \
    for (uint32_t i = 0; i < 13; i++) {                  \
        printf("r%d:\t0x%x\n", i, stack_pointer[i]);     \
    }                                                    \
    printf("lr:\t0x%x\n", stack_pointer[13]);            \
    printf("sp:\t0x%x\n", stack_pointer);                \
    printf("flags:\t0x%x\n", flags_reg);

extern uint32_t _sstack;
extern uint32_t _estack;
extern uint32_t end;

extern uint32_t __ldrex(void *addr);
extern bool __strex(void *addr, uint32_t val);

#define STACK_SIZE ((&_estack - &_sstack) * sizeof(uint32_t))
#define HEAP_SIZE (96000 - STACK_SIZE)

// #define TEST // define if testing usart printing

#define USART_TTY USART2

void *memset(void *s, int c, size_t len) {
    char *data = (char *)s;
    for (size_t i = 0; i < len; i++) {
        data[i] = c;
    }

    return data;
}
void *memcpy(void *dest, const void *src, size_t n) {
    char *d = (char *)dest;
    char *s = (char *)src;

    for (size_t i = 0; i < n; i++) {
        d[i] = s[i];
    }

    return dest;
}

struct lock {
    bool locked;
    void *protected_data;
};

struct lock lock1;

struct lock *create_lock(void *protected_data) {
    //malloc a lock?

    lock1.locked = false;
    lock1.protected_data = protected_data;
    return &lock1;
}

void take_lock(struct lock *l) {
    while (true) {
        while (__ldrex(l)) {
            asm("nop");
        }
        if (0 == __strex(l, 1)) {
            break;
        }
    }
}

void release_lock(struct lock *l) {
    l->locked = false;
}

struct registers {
    // we save and restore ourselves in PendSV_Handler
    uint32_t sp; // r13
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7; //fp???
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;

    // saved and restored by the CPU on interrupt
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12; // intra-process scratch register
    uint32_t lr;  // r14
    uint32_t pc;  // r15
    uint32_t xpsr;
};

struct process {
    uint32_t pid;
    struct registers registers;
};

#define PROCESS_STACK_SIZE 128

struct process *process_current;
struct process *process_next;
struct process process_1;
struct process process_2;

struct page {
    uint32_t i;
    uint32_t is_allocated;
    uint32_t owner;
};

struct page *pages;
uint32_t pages_count;
uint32_t pages_metadata_count;
char *heap_start;

#define PAGE_SIZE (2048)

uint32_t get_current_pid() {
    return process_current != 0 ? process_current->pid : 0;
}

void mem_init() {
    // get amount of pages we can fit in memory
    pages_count = HEAP_SIZE / PAGE_SIZE;
    pages = (struct page *)&end;

    // we need some page-aligned space at the start of the heap to store out
    // page metadata
    pages_metadata_count = (pages_count*sizeof(struct page))/PAGE_SIZE + 1;
    pages_count -= pages_metadata_count;

    // clear page metadata to zero
    memset(pages, 0, pages_count * sizeof(struct page));

    // set up IDs for pages
    for (uint32_t i = 0; i < pages_count; i++) {
        pages[i].i = i;
    }

    heap_start = (char*) &end + PAGE_SIZE;
}

void mem_print_page_info() {
    char preview[50];

    printf("page metadata size: %d\n", sizeof(struct page));
    printf("page size: %d\n", PAGE_SIZE);
    printf("heap size: %d\n", HEAP_SIZE);
    printf("amount of pages of metadata we are storing: %d \n", pages_metadata_count);
    printf("amount of pages (excluding metadata) we can fit in memory: %d\n", pages_count);

    for (uint32_t i = 0; i < pages_count; i++) {
        struct page *page = &pages[i];

        printf("i=%d allocated: %s\n", page->i, page->is_allocated ? "T\0" : "F\0");
        if (page->is_allocated) {
            memcpy(preview, heap_start + (page->i * PAGE_SIZE), 46);
            printf("%s\n", preview);
        }
    }
}

void *mem_get_page(uint32_t owner) {
    DISABLE_IRQ;
    void *out = 0;

    for (uint32_t i = 0; i < pages_count; i++) {
        struct page *page = &pages[i];

        if (!page->is_allocated) {
            page->is_allocated = true;
            page->owner = owner;

            out = heap_start + (page->i * PAGE_SIZE);
            goto ret;
        }
    }

    printf("ERROR: no pages left\n");

ret:
    ENABLE_IRQ;
    return out;
}

void *mem_get_user_page() {
    return mem_get_page(get_current_pid());
}

void mem_free_page(void *p) {
    DISABLE_IRQ;

    uint32_t offs = (uint32_t)p - (uint32_t)heap_start;
    if (offs % PAGE_SIZE != 0) {
        printf("ERROR: trying to free a page which does not fall on a page boundary\n");
        goto ret;
    }

    struct page *page = &pages[offs / PAGE_SIZE];
    page->is_allocated = false;

ret:
    ENABLE_IRQ;
}
// TODO(harrison): should we make this disable int  errupts whien it sends the bits?
int putchar(int c) {
    // will need a lock on the usart port

    if (c == '\n') {
        putchar('\r');
    }
    while (!(USART_TTY->ISR & USART_ISR_TC)) {}

    USART_TTY->TDR = (USART_TTY->TDR & ~USART_TDR_TDR_Msk) | (uint32_t)(c);

    return 0;
}

// trigger red led
void fn_process_1() {
    int j = 0;
    while (true) {
        take_lock(&lock1);
        printf("hello from process %d\n", get_current_pid());
        release_lock(&lock1);

        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }

        printf("hello\n");

        if (j == 3) {
            uint32_t* ptr = (uint32_t*) (0x2000c800);
            *ptr = 10;
        }

        j += 1;
    }
}

// trigger green led
void fn_process_2() {
    // mem_print_page_info();

    while (true) {
        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }

        GPIOE->ODR ^= GPIO_ODR_OD8;
    }
}

#define MAX_PROCESSES 16
struct process processes[MAX_PROCESSES] = {0};

void process_quit() {
    uint32_t pid = get_current_pid();

    for (uint32_t i = 0; i < pages_count; i++) {
        if (pages[i].owner == pid) {
            mem_free_page(heap_start + (pages[i].i * PAGE_SIZE));
        }
    }

    processes[pid - 1].pid = 0;

    while (1) {
        asm("nop");
        asm("nop");
    }
}

int create_process(void *func) {
    //returns a process ID
    DISABLE_IRQ;
    int out = 0;

    for (int i = 0; i < MAX_PROCESSES; i++) {
        if (processes[i].pid == 0) {
            uint32_t pid = i + 1;
            uint32_t *stack_address = (uint32_t *)mem_get_page(pid);

            if (stack_address == 0) {
                printf("Failed to allocate a stack for a new process");
                goto ret;
            }
            //later: allocate heap

            uint32_t spi = PROCESS_STACK_SIZE - 16;

            processes[i].registers.sp = (int32_t)&stack_address[spi];
            // process_1_stack[spi + 0] = 0x0;                     // r0
            // process_1_stack[spi + 1] = 0x0;                     // r1
            // process_1_stack[spi + 2] = 0x0;                     // r2
            // process_1_stack[spi + 3] = 0x0;                     // r3
            // process_1_stack[spi + 4] = 0x0;                     // r12
            stack_address[spi + 5] = (uint32_t)&process_quit; // lr
            stack_address[spi + 6] = (uint32_t)func;          // pc
            stack_address[spi + 7] = 0x01000000;              // xpsr

            //register the process with the scheduler.
            out = processes[i].pid = pid;
            goto ret;
        }
    }
    printf("16 tasks are already running!");
ret:
    ENABLE_IRQ;
    return out;
}

void main_user_process() {
    create_process(&fn_process_1);
    create_process(&fn_process_1);
    create_process(&fn_process_1);
    create_process(&fn_process_2);
    asm("nop");
}

void kernel_main(void) {
    FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;

    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_2WS) {}

    RCC->CR &= ~RCC_CR_MSIRANGE_Msk;
    RCC->CR |= RCC_CR_MSIRANGE_11;

    RCC->CR |= RCC_CR_MSIRGSEL;

    while ((RCC->CR & RCC_CR_MSIRDY) != RCC_CR_MSIRDY) {}

    mem_init();

    //
    // SysTick setup
    //

    // trigger 10 times a second
    SysTick->LOAD &= ~SysTick_LOAD_RELOAD_Msk;
    SysTick->LOAD |= 48000; // clocked at 48MHz, 48Mhz / 1000hz is 48000 (tick at 1ms intervals)

    SysTick->CTRL |=
        SysTick_CTRL_CLKSOURCE_Msk | // use system clock (48MHz by now)
        SysTick_CTRL_TICKINT_Msk;    // trigger interrupt at end of count down

    // trigger an interrupt on zero
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    // set PendSV priority to lowest possible
    SCB->SHP[14 - 4] = 0xFF;
    // set SysTick priority to highest possible
    SCB->SHP[15 - 4] = 0x00;

    //
    // MPU Setup
    //

    // fall back to the default memory map in privileged mode
    MPU->CTRL &= ~MPU_CTRL_PRIVDEFENA_Msk;
    MPU->CTRL |= MPU_CTRL_PRIVDEFENA_Msk;

    // enable the MPU
    MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
    MPU->CTRL |= MPU_CTRL_ENABLE_Msk;

    // work with the 1st region
    MPU->RNR &= ~MPU_RNR_REGION_Msk;
    MPU->RNR |= 1;

    // 0x2000c800 is the 6th page

    // set address of 1st region to 0x2000c800
    MPU->RBAR &= ~MPU_RBAR_ADDR_Msk;
    MPU->RBAR |= 0x2000c800;

    // set size to 2048 (11)
    MPU->RASR &= ~MPU_RASR_SIZE_Msk;
    MPU->RASR |= (11 - 1) << MPU_RASR_SIZE_Pos;

    // disable all reads and writes, even in privileged mode
    MPU->RASR &= ~MPU_RASR_AP_Msk;
    MPU->RASR |= 0b000 << MPU_RASR_AP_Pos;

    // enable region 1
    MPU->RASR &= ~MPU_RASR_ENABLE_Msk;
    MPU->RASR |= MPU_RASR_ENABLE_Msk;

    SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

    //
    // USART setup
    //

    // interrupts
    // direct lines don't require EXTI configuration
    NVIC->ISER[1] |= 1 << (38 - 32);

    // enable port D (for usart rx/tx)
    // enable port B (for red LED)
    // enable port E (for green LED)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIODEN | RCC_AHB2ENR_GPIOEEN;

    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    // enable alternate mode 7 for pin 5 and pin 6 of port D
    GPIOD->AFR[0] |= (7 << GPIO_AFRL_AFSEL5_Pos) | (7 << GPIO_AFRL_AFSEL6_Pos);

    // set pins 5 and 6 to alternate mode (10)

    GPIOD->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6);
    GPIOD->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1;

    // set mode for red led (pin 2) to general purpose output (01)
    GPIOB->MODER &= ~GPIO_MODER_MODE2;
    GPIOB->MODER |= GPIO_MODER_MODE2_0;

    // set mode for green led (pin 8) to general purpose output
    GPIOE->MODER &= ~GPIO_MODER_MODE8;
    GPIOE->MODER |= GPIO_MODER_MODE8_0;

    // 5000 happens to be the number we need to reach 9600 baud.
    // probably should be a bit cleverer here, but oh well---we'll
    // just OR it in.
    USART_TTY->BRR |= 1250;

    USART_TTY->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART_TTY->CR1 |= USART_CR1_UE;

#ifdef TEST
    void HardFault_Handler(void);
    print_test();

    // HardFault_Handler();
#endif
    create_process(&main_user_process);
    while (true) {
        putchar('.');
    }
}

struct registers *context_switch(struct registers *registers) {
    if (process_current != 0) {
        process_current->registers = *registers;
    }

    process_current = process_next;
    process_next = 0;

    return &process_current->registers;
}

void protect_memory(void) {
}

void SysTick_Handler(void) {
    // scheduling happens in here, and we choose the next process

    int prev_task_index = process_current != 0 ? process_current->pid - 1 : 0;
    int check_process = prev_task_index + 1;
    for (int i = 0; i < MAX_PROCESSES; i++) {
        if (processes[check_process].pid != 0) {
            process_next = &processes[check_process];
            break;
        }
        check_process = (check_process + 1) % MAX_PROCESSES;
    }
    if (process_next != 0) {
        // trigger PendSV
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    }
    //do nothing as no tasks are running
}

void USART2_IRQHandler(void) {
    char recv = (char)(USART_TTY->RDR & 0xFF);
    putchar(recv);
}

void MemManage_Handler(void) {
    printf("can't access memory address: 0x%x\n", SCB->MMFAR);

    while (true);
}

void HardFault_Handler(void) {
    DUMP_REGS;
    printf("Hard Fault - Possibly Unrecoverable :(\n");

    while (true)
        ;
}

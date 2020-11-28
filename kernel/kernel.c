#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "kernel/lib/stm32l476xx.h"

#include "user/user.h"

#define USART_TTY USART2
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

#define SIMPLE_PRINTF_PUTCHAR putchar

// #define TEST // define if testing usart printing
#include "printf.h"

#define printk simple_printf

#define DUMP_REGS                                        \
    uint32_t *stack_pointer;                             \
    uint32_t flags_reg;                                  \
    asm volatile("push {r0-r12,lr}\n\tmov %[result], sp" \
                 : [ result ] "=r"(stack_pointer));      \
    asm volatile("mrs %[result], APSR"                   \
                 : [ result ] "=r"(flags_reg));          \
    printk("General Register Dump\n");                   \
    for (uint32_t i = 0; i < 13; i++) {                  \
        printk("r%d:\t0x%x\n", i, stack_pointer[i]);     \
    }                                                    \
    printk("lr:\t0x%x\n", stack_pointer[13]);            \
    printk("sp:\t0x%x\n", stack_pointer);                \
    printk("flags:\t0x%x\n", flags_reg);

extern uint32_t _sstack;
extern uint32_t _estack;
extern uint32_t end;

extern uint32_t __ldrex(void *addr);
extern bool __strex(void *addr, uint32_t val);
extern void parasite_process(char *sp, void (*handler)(void));

#define STACK_SIZE ((&_estack - &_sstack) * sizeof(uint32_t))
#define HEAP_SIZE (96000 - STACK_SIZE)

// #define TEST // define if testing usart printing

#define USART_TTY USART2

enum {
    SYSCALL_TOGGLE_RED,
    SYSCALL_TOGGLE_GREEN,
    SYSCALL_PUTCHAR,
    SYSCALL_MICROS,
};

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

    uint32_t *stack_top;
    void (*handler)(void);

    struct registers registers;
};

#define PROCESS_STACK_SIZE 128

struct process *process_current;
struct process *process_next;

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
    pages_metadata_count = (pages_count * sizeof(struct page)) / PAGE_SIZE + 1;
    pages_count -= pages_metadata_count;

    // clear page metadata to zero
    memset(pages, 0, pages_count * sizeof(struct page));

    // set up IDs for pages
    for (uint32_t i = 0; i < pages_count; i++) {
        pages[i].i = i;
    }

    heap_start = (char *)((uint32_t)(&end) + PAGE_SIZE);
}

void mem_print_page_info() {
    char preview[50];

    printk("page metadata size: %d\n", sizeof(struct page));
    printk("page size: %d\n", PAGE_SIZE);
    printk("heap size: %d\n", HEAP_SIZE);
    printk("amount of pages of metadata we are storing: %d \n", pages_metadata_count);
    printk("amount of pages (excluding metadata) we can fit in memory: %d\n", pages_count);

    for (uint32_t i = 0; i < pages_count; i++) {
        struct page *page = &pages[i];

        printk("i=%d allocated: %s\n", page->i, page->is_allocated ? "T\0" : "F\0");
        if (page->is_allocated) {
            memcpy(preview, heap_start + (page->i * PAGE_SIZE), 46);
            printk("%s\n", preview);
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

    printk("ERROR: no pages left\n");

ret:
    ENABLE_IRQ;
    return out;
}

void *mem_get_user_page() {
    return mem_get_page(get_current_pid());
}

#define MALLOC_HEADER_MAGIC (0xD0BEDEAD)
struct malloc_header {
    struct malloc_header *prev;
    struct malloc_header *next;
    uint32_t is_allocated;
    uint32_t size;
    uint32_t magic;
};

char* malloc_heap = 0;
struct malloc_header *malloc_heap_first;
void* kmalloc(uint32_t size) {
    if (malloc_heap == 0) {
        malloc_heap = mem_get_page(0);

        if (malloc_heap == 0) {
            printk("ERROR: can't get page for malloc-ing\n");

            return 0;
        }

        malloc_heap_first = (struct malloc_header*) malloc_heap;
        malloc_heap_first->magic = MALLOC_HEADER_MAGIC;
        malloc_heap_first->prev = 0;
        malloc_heap_first->next = 0;
        malloc_heap_first->is_allocated = false;
        malloc_heap_first->size = PAGE_SIZE;
    }

    size += sizeof(struct malloc_header);

    struct malloc_header *best = 0;
    uint32_t best_size = 0xFFFFFFFF;

    for (struct malloc_header *curr = malloc_heap_first; curr != 0; curr = curr->next) {
        if (!curr->is_allocated && size < curr->size) {
            best = curr;
            best_size = curr->size;
        }
    }

    if (best == 0) {
        printk("ERROR: can't malloc %d (no space)\n", size);

        return 0;
    }

    // split header
    if (best_size - size > 2 * sizeof(struct malloc_header)) {
        best->size = size;
        struct malloc_header* split = (struct malloc_header*) (((char*) best) + best->size);
        split->magic = MALLOC_HEADER_MAGIC;
        split->size = best_size - size;
        split->is_allocated = false;

        best->next = split;
        split->prev = best;
        split->next = 0;
    }

    best->is_allocated = true;

    return best + 1;
}

void kfree(void* loc) {
    // get header
    struct malloc_header* header = loc - sizeof(struct malloc_header);
    if (header->magic != MALLOC_HEADER_MAGIC) {
        printk("ERROR: this isn't the originally returned address\n");

        return;
    }

    header->is_allocated = false;
}

void kmalloc_print_info() {
    if (malloc_heap == 0) {
        printk("heap not allocated\n");

        return;
    }

    for (struct malloc_header *curr = malloc_heap_first; curr != 0; curr = curr->next) {
        if (curr->magic != MALLOC_HEADER_MAGIC) {
            printk("ERROR: iterating through something which isn't a malloc header\n");

            return;
        }

        printk("entry: size=%d is_allocated=%d\n", curr->size, curr->is_allocated);
    }
}

void mem_free_page(void *p) {
    DISABLE_IRQ;

    uint32_t offs = (uint32_t)p - (uint32_t)heap_start;
    if (offs % PAGE_SIZE != 0) {
        printk("ERROR: trying to free a page which does not fall on a page boundary\n");
        goto ret;
    }

    struct page *page = &pages[offs / PAGE_SIZE];
    page->is_allocated = false;

ret:
    ENABLE_IRQ;
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

//returns a process ID
int create_process(void (*handler)(void)) {
    DISABLE_IRQ;
    int out = 0;

    for (int i = 0; i < MAX_PROCESSES; i++) {
        if (processes[i].pid == 0) {
            uint32_t pid = i + 1;
            uint32_t *stack_address = (uint32_t *)mem_get_page(pid);

            if (stack_address == 0) {
                printk("Failed to allocate a stack for a new process");
                goto ret;
            }
            //later: allocate heap

            processes[i].handler = handler;

            // TODO(Harrison): shouldn't this be PAGE_SIZE/4?
            uint32_t spi = PROCESS_STACK_SIZE - 16;

            processes[i].stack_top = stack_address;
            processes[i].registers.sp = (int32_t)&stack_address[spi];
            // process_1_stack[spi + 0] = 0x0;                     // r0
            // process_1_stack[spi + 1] = 0x0;                     // r1
            // process_1_stack[spi + 2] = 0x0;                     // r2
            // process_1_stack[spi + 3] = 0x0;                     // r3
            // process_1_stack[spi + 4] = 0x0;                     // r12
            stack_address[spi + 5] = (uint32_t)&process_quit; // lr
            stack_address[spi + 6] = (uint32_t)handler;       // pc
            stack_address[spi + 7] = 0x01000000;              // xpsr

            //register the process with the scheduler.
            out = processes[i].pid = pid;

            goto ret;
        }
    }
    printk("16 tasks are already running!");
ret:
    ENABLE_IRQ;
    return out;
}

void protect_memory() {
    // enable the MPU
    MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;

    //
    // Access to flash
    //
    int region = 6;
    MPU->RNR &= ~MPU_RNR_REGION_Msk;
    MPU->RNR |= region;

    MPU->RBAR &= ~MPU_RBAR_ADDR_Msk;
    MPU->RBAR |= 0x0;

    MPU->RASR &= ~MPU_RASR_SIZE_Msk;
    MPU->RASR |= (29 - 1) << MPU_RASR_SIZE_Pos;

    MPU->RASR &= ~MPU_RASR_AP_Msk;
    MPU->RASR |= 0b010 << MPU_RASR_AP_Pos;

    MPU->RASR &= ~MPU_RASR_ENABLE_Msk;
    MPU->RASR |= MPU_RASR_ENABLE_Msk;

    int stack_pages_count = ((uint32_t)&_estack - (uint32_t)&_sstack)/PAGE_SIZE;

    // NOTE(harrison): cant_access holds one byte for each possible region
    // which can be allocated. bits set are subregions which can't be accessed.
    // TODO(harrison): calculate 6 based on the actual amount of pages needed
    uint8_t cant_access[6] = {0};

    for (int j = 0; j < pages_count; j++) {
        int i = stack_pages_count + pages_metadata_count + j;
        int region = i / 6;
        int bit = i % 8;
        bool no_rw = pages[j].owner != process_current->pid;

        cant_access[region] |= no_rw << bit;
    }

    // NOTE(harrison): update these two at the same time.
    int region_rasr_size = 14 - 1; // size of region is 2^(rasr_size + 1)
    int region_size = PAGE_SIZE*8; // = 2^14

    for (int i = 0; i < 6; i++) {
        uint8_t protection = cant_access[i];

        // TODO(harrison): skip and reset regions when we need to

        MPU->RNR &= ~MPU_RNR_REGION_Msk;
        MPU->RNR |= i;

        MPU->RBAR &= ~MPU_RBAR_ADDR_Msk;
        MPU->RBAR |= (uint32_t) &_sstack + region_size * i;

        MPU->RASR &= ~MPU_RASR_SIZE_Msk;
        MPU->RASR |= region_rasr_size << MPU_RASR_SIZE_Pos;

        // enable reads and writes in unprivileged mode
        MPU->RASR &= ~MPU_RASR_AP_Msk;
        MPU->RASR |= 0b011 << MPU_RASR_AP_Pos;

        // set up subregions
        MPU->RASR &= ~MPU_RASR_SRD_Msk;
        MPU->RASR |= protection << MPU_RASR_SRD_Pos;

        // enable subregions
        MPU->RASR &= ~MPU_RASR_S_Msk;
        MPU->RASR |= MPU_RASR_S_Msk;

        // enable region
        MPU->RASR &= ~MPU_RASR_ENABLE_Msk;
        MPU->RASR |= MPU_RASR_ENABLE_Msk;
    }

    MPU->CTRL |= MPU_CTRL_ENABLE_Msk;

    __ISB();
    __DSB();
}

// switches to the first process
void scheduler_start() {
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    if (processes[0].pid == 0) {
        printk("ERROR: no processes started. Please start a process.\n");

        return;
    }

    // switch to first process
    process_current = &processes[0];

    protect_memory();

    // sets PSP, switches privileges, and then calls the handler function.
    // does not return
    // NOTE(harrison): if we ever want to use the system stack again, we should
    // handle this better.
    parasite_process(
        (char *)(process_current->stack_top + PROCESS_STACK_SIZE),
        process_current->handler);
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

    // set PendSV priority to lowest possible
    SCB->SHP[14 - 4] = 0xFF;
    // set SysTick priority to highest possible
    SCB->SHP[15 - 4] = 0x00;

    // Privilege level setup
    uint32_t ctrl = __get_CONTROL();
    ctrl &= ~CONTROL_nPRIV_Msk;
    ctrl |= CONTROL_nPRIV_Msk; // thread mode has unprivileged access

    ctrl &= ~CONTROL_SPSEL_Msk;
    ctrl |= CONTROL_SPSEL_Msk; // use different stacks for main and not main process

    //
    // Timer Setup
    //
    // enable nvic interrupt
    NVIC->ISER[1] |= 1 << (55 - 32);

    // enable clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN_Msk;

    // enable tim7 interrupt
    TIM7->DIER |= TIM_DIER_UIE_Msk;

    // set auto reload value
    TIM7->ARR = 0xFFFF;
    // set prescaler value
    // 48 = 48,000,000 / 1,000,000
    TIM7->PSC = 48;

    // enable counter
    TIM7->CR1 |= TIM_CR1_CEN_Msk;

    //
    // MPU Setup
    //

    // fall back to default system memory map in privileged mode
    MPU->CTRL &= ~MPU_CTRL_PRIVDEFENA_Msk;
    MPU->CTRL |= MPU_CTRL_PRIVDEFENA_Msk;

    // enable the MPU
    MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
    MPU->CTRL |= MPU_CTRL_ENABLE_Msk;

    // raise MemManage fault on memory errors
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
    printk("Booting leg-os kernel.\n");

    create_process(&fn_process_1);
    create_process(&fn_process_2);

    mem_print_page_info();

    scheduler_start();

    while (true) {
        asm("nop");
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

    // do nothing as no tasks are running
}

struct profiler_control {
    short overflow;
    //TODO(obi) add profiler storage.
};

struct profiler_control profiler;

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~1;
    profiler.overflow++; //this bad boy overflows
}

void SVC_Handler_C(size_t *sp) {
    char r0 = (char)sp[0];
    char *pc = (char *)sp[6];

    uint8_t svc = *(pc - 2);

    switch (svc) {
    case SYSCALL_TOGGLE_RED:
        GPIOB->ODR ^= GPIO_ODR_OD2;

        break;
    case SYSCALL_TOGGLE_GREEN:
        GPIOE->ODR ^= GPIO_ODR_OD8;
        break;
    case SYSCALL_PUTCHAR:
        putchar(r0);
        break;
    case SYSCALL_MICROS:
        sp[0] = TIM7->CNT;
        sp[0] = (profiler.overflow << sizeof(short)) + sp[0];
        break;
    default:
        printk("ERROR: unknown syscall %d\n", svc);
    }
}

void USART2_IRQHandler(void) {
    char recv = (char)(USART_TTY->RDR & 0xFF);
    putchar(recv);
}

void MemManage_Handler(void) {
    printk("can't access memory address: 0x%x\n", SCB->MMFAR);

    while (true) {}
}

void HardFault_Handler(void) {
    // DUMP_REGS;
    printk("Hard Fault - Possibly Unrecoverable :(\n");

    while (true) {}
}

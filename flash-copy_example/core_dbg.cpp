/*
 * core_dbg.c
 *
 *  Created on: 4 мая 2017 г.
 *      Author: RLeonov
 */

#include "ch.h"
#include "uart.h"
#include "core_dbg.h"

// LOOP awaiting DMA tranfer complete
#define LOOP()                                           {for(uint32_t i = 4021; i != 0; i--) __NOP(); }
#define HALT()                                           {for (;;) {}}

__STATIC_INLINE void fatal()
{
    HALT(); // TODO: remove this
//    __ASM volatile ("dsb");
//    TODO: RESET CORE
//    __ASM volatile ("dsb");
}

#if (CORE_DEVELOPER_MODE)
void stack_dump(unsigned int addr, unsigned int size)
{
    unsigned int start = addr + 0x4000 - size; // 0x4000 - SRAM_SIZE
    PrintfCNow("memory dump 0x%X-0x%X\r", start, start + size);
    unsigned int i = 0;
    for (i = 0; i < size; ++i)
    {
        if ((i % 0x10) == 0)
            PrintfCNow("0x%X: ", start + i);
        PrintfCNow("%X ", ((unsigned char*)start)[i]);
        if ((i % 0x10) == 0xf)
            PrintfCNow("\r");
    }
    if (size % 0x10)
        PrintfCNow("\r");
}
#endif // CORE_DEVELOPER_MODE

void panic()
{
#if (CORE_DEVELOPER_MODE)
    stack_dump(SRAM_BASE, 0x200);
    // dump(SRAM_BASE, 0x200);
    HALT();
#else
    fatal();
#endif //KERNEL_DEVELOPER_MODE
}

#if (CORE_DEBUG)
extern "C" {
void HardFault_Handler(void)
{
    LOOP();
    PrintfCNow("HARD FAULT\r");
    panic();
}

void MemManage_Handler(void)
{
    LOOP();
    PrintfCNow("MEM FAULT\r");
    panic();
}

void BusFault_Handler(void)
{
    LOOP();
    PrintfCNow("BUS FAULT\r");
    panic();
}

void UsageFault_Handler(void)
{
    LOOP();
    PrintfCNow("USAGE FAULT\r");
    panic();
}
}
#endif // CORE_DEBUG

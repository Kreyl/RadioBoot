/*
 * main.c
 *
 *  Created on: 3 мая 2017 г.
 *      Author: RLeonov
 */

#include <stdbool.h>
#include <stdint.h>
#include "../rexos/userspace/stm32/stm32.h"

#if defined(STM32F0)
#define FLASH_PAGE_SIZE                       1024
#elif defined(STM32L1)
#define FLASH_PAGE_SIZE                       256
#endif

#define HALT()                                 for(;;)

#define RELEASE                                1

#if (RELEASE)
#define DEBUG                                  0
#else
#define DEBUG                                  1
#endif

#if (DEBUG)
#include "dbg.h"
#endif

static void flash_cmd_unlock()
{
    /* (1) Wait till no operation is on going */
    /* (2) Check that the Flash is unlocked */
    /* (3) Perform unlock sequence */
#if defined(STM32F0) || defined(STM32L1)
    /* (1) */
    while ((FLASH->SR & FLASH_SR_BSY) != 0)
    {
        __NOP();
        __NOP();
    }
#endif

#if defined(STM32F0)
    if ((FLASH->CR & FLASH_CR_LOCK) != 0)                           /* (2) */
    {
        FLASH->KEYR = FLASH_KEY1;                                   /* (3) */
        FLASH->KEYR = FLASH_KEY2;
    }
#elif defined(STM32L1)
    if((FLASH->PECR & FLASH_PECR_PELOCK) != 0)                      /* (2) */
    {
        FLASH->PEKEYR = FLASH_PEKEY1;                               /* (3) */
        FLASH->PEKEYR = FLASH_PEKEY2;
        FLASH->SR = FLASH_SR_WRPERR;
        FLASH->PECR &= ~FLASH_PECR_FTDW;
        while((FLASH->PECR & FLASH_PECR_PELOCK) != 0);
    }

    if((FLASH->PECR & FLASH_PECR_PRGLOCK) != 0)
    {
        FLASH->PRGKEYR = FLASH_PRGKEY1;
        FLASH->PRGKEYR = FLASH_PRGKEY2;
        while((FLASH->PECR & FLASH_PECR_PRGLOCK) != 0);
    }
#endif
}

static inline void flash_cmd_lock()
{
    /* lock FLASH */
#if defined(STM32F0)
    FLASH->CR |= FLASH_CR_LOCK;
#elif defined(STM32L1)
    FLASH->PECR |= FLASH_PECR_PELOCK;
    FLASH->PECR |= FLASH_PECR_PRGLOCK;
#endif
}

static void flash_cmd_program_prepare()
{
#if defined(STM32F0)
    FLASH->CR |= FLASH_CR_PG;                       /* (1) Set the PG bit in the FLASH_CR register to enable programming */
                                                    /* (2) Perform program */
#endif
}

static int flash_cmd_program_execute()
{
#if defined(STM32F0)
    while ((FLASH->SR & FLASH_SR_BSY) != 0) {}      /* (3) Wait until the BSY bit is reset in the FLASH_SR register */

    if ((FLASH->SR & FLASH_SR_EOP) != 0)            /* (4) Check the EOP flag in the FLASH_SR register */
        FLASH->SR = FLASH_SR_EOP;                   /* (5) clear it by software by writing it at 1 */
    else
    {
        /* manage the error cases */
    }
    FLASH->CR &= ~FLASH_CR_PG;                      /* (6) Reset the PG Bit to disable programming */
#endif
    return 0;
}

#if defined(STM32L1)
static inline void flash_cmd_enable_erase()
{
    if(!(FLASH->PECR & FLASH_PECR_ERASE))
        FLASH->PECR |= FLASH_PECR_ERASE;            /* (1) Set the ERASE bit in the FLASH_PECR register */
    if(!(FLASH->PECR & FLASH_PECR_PROG))
        FLASH->PECR |= FLASH_PECR_PROG;             /* (2) Set the PROG bit in the FLASH_PECR register to choose program page */
    while ((FLASH->SR & FLASH_SR_BSY) != 0);        /* (3) Wait for the BSY bit to be cleared */
}

static inline void flash_cmd_disable_erase()
{
    FLASH->PECR &= ~(FLASH_PECR_ERASE);
    FLASH->PECR &= ~(FLASH_PECR_PROG);
}
#endif // STM32L1

// FLASH memory must be already unlocked.
static inline int flash_cmd_erase_page(uint32_t addr)
{
#if defined(STM32F0)
    FLASH->CR |= FLASH_CR_PER;                      /* (1) Set the PER bit in the FLASH_CR register to enable page erasing */
    FLASH->AR = addr;                          /* (2) Program the FLASH_AR register to select a page to erase */
    FLASH->CR |= FLASH_CR_STRT;                     /* (3) Set the STRT bit in the FLASH_CR register to start the erasing */
    while ((FLASH->SR & FLASH_SR_BSY) != 0)         /* (4) Wait until the BSY bit is reset in the FLASH_SR register */
    if ((FLASH->SR & FLASH_SR_EOP) != 0)            /* (5) Check the EOP flag in the FLASH_SR register */
        FLASH->SR = FLASH_SR_EOP;                   /* (6) Clear EOP flag by software by writing EOP at 1 */
    else
    {
        /* Manage the error cases */
    }
    FLASH->CR &= ~FLASH_CR_PER;                     /* (7) Reset the PER Bit to disable the page erase */
#elif defined(STM32L1)
    flash_cmd_enable_erase();

    if(!(FLASH->PECR & FLASH_PECR_ERASE))
        FLASH->PECR |= FLASH_PECR_ERASE;            /* (1) Set the ERASE bit in the FLASH_PECR register */
    if(!(FLASH->PECR & FLASH_PECR_PROG))
        FLASH->PECR |= FLASH_PECR_PROG;             /* (2) Set the PROG bit in the FLASH_PECR register to choose program page */
    while ((FLASH->SR & FLASH_SR_BSY) != 0);        /* (3) Wait for the BSY bit to be cleared */
    *(uint32_t*)addr = 0x00000000;                  /* (4) Write 0x0000 0000 to the first word of the program page to erase */

    flash_cmd_disable_erase();
#endif

    return 0;
}

static inline int flash_cmd_program_word(unsigned int addr, unsigned int data)
{
    flash_cmd_program_prepare();
#if defined(STM32F0)
    *((uint16_t*)addr) = (uint16_t)data;
    *((uint16_t*)(addr + sizeof(uint16_t))) = (uint16_t)(data >> 16);
#elif defined(STM32L1)
    *((uint32_t*)addr) = data;
#endif
    return flash_cmd_program_execute();
}

static inline void flash_program_page(uint32_t dst, uint32_t src, int size)
{
    unsigned int last_word = 0;
    /* program full words of source */
    for (; size >= sizeof(uint32_t); size -= sizeof(uint32_t))
    {
#if (DEBUG)
        printf("Program addr %X from %X\n", dst, src);
#endif // DEBUG
#if (RELEASE)
        flash_cmd_program_word(dst, *(unsigned int*)src);
#endif // RELEASE
        dst += sizeof(uint32_t);
        src += sizeof(uint32_t);
    }

    if(size)
    {
        /* program last word. If word is not full - other bytes will be 0x00 */
        while(size-- > 0)
        {
            last_word <<= 8;
            last_word |= *(uint8_t*)(src + size);
        }
    #if (DEBUG)
        printf("Program last addr %X from %X\n", dst, src);
    #endif // DEBUG
    #if (RELEASE)
        flash_cmd_program_word(dst, last_word);
    #endif // RELEASE
    }
}

// IRQ should be disabled
// addr and size also checked
// size -  in bytes
int flash_update(unsigned int dst_addr, unsigned int src_addr, int bytes_to_copy)
{
    unsigned int start_addr = dst_addr & ~(FLASH_PAGE_SIZE - 1);

    __DSB();
    __ISB();

#if (DEBUG)
    printf("flash update\n");
    printf("start_addr %X\n", start_addr);
    printf("end_addr %X\n", end_addr);
    printf("size: %d\n\n", bytes_to_copy);
#endif // DEBUG

#if(RELEASE)
    flash_cmd_unlock(); /* unlock flash */
#endif // RELEASE

    // program first total pages
    while(bytes_to_copy > FLASH_PAGE_SIZE)
    {
#if (RELEASE)
        flash_cmd_erase_page(start_addr);
#endif // RELEASE
#if (DEBUG)
        printf("Erase page addr: %X\n", start_addr);
#endif // DEBUG
        start_addr += FLASH_PAGE_SIZE;

        flash_program_page(dst_addr, src_addr, FLASH_PAGE_SIZE);
        dst_addr += FLASH_PAGE_SIZE;
        src_addr += FLASH_PAGE_SIZE;
        bytes_to_copy -= FLASH_PAGE_SIZE;
    }

    // program last page
#if (DEBUG)
    printf("Erase page addr: %X\n", start_addr);
#endif // DEBUG
#if (RELEASE)
    flash_cmd_erase_page(start_addr);
#endif // RELEASE

    flash_program_page(dst_addr, src_addr, bytes_to_copy);

#if (RELEASE)
    flash_cmd_lock(); /* lock flash */
    /* Reset core */
    NVIC_SystemReset();
#endif // RELEASE

    return 0;
}


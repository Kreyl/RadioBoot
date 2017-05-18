/*
 * main.c
 *
 *  Created on: 3 мая 2017 г.
 *      Author: RLeonov
 */

#include <stdbool.h>
#include <stdint.h>
#include "../rexos/userspace/stm32/stm32.h"
#include "errors.h"

#if defined(STM32F0)
#define FLASH_SECTOR_SIZE                       2048
#elif defined(STM32L1)
#define FLASH_SECTOR_SIZE                       4096
#endif

#define FLASH_SECTOR_SIZE_WORDS                 (FLASH_SECTOR_SIZE / sizeof(uint32_t))


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
    if(FLASH->PECR & FLASH_PECR_PELOCK)
    {
        FLASH->PEKEYR = FLASH_PEKEY1;
        FLASH->PEKEYR = FLASH_PEKEY2;
        FLASH->SR = FLASH_SR_WRPERR;
        FLASH->PECR &= ~FLASH_PECR_FTDW;
        while((FLASH->PECR & FLASH_PECR_PELOCK) != 0);
    }

    if(FLASH->PECR & FLASH_PECR_PRGLOCK)
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
    return OK;
}

// FLASH memory must be already unlocked.
static inline int flash_cmd_erase_page(uint32_t page_addr)
{
#if defined(STM32F0)
    FLASH->CR |= FLASH_CR_PER;                      /* (1) Set the PER bit in the FLASH_CR register to enable page erasing */
    FLASH->AR = page_addr;                          /* (2) Program the FLASH_AR register to select a page to erase */
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
    if(!(FLASH->PECR & FLASH_PECR_ERASE))
        FLASH->PECR |= FLASH_PECR_ERASE;            /* (1) Set the ERASE bit in the FLASH_PECR register */
    if(!(FLASH->PECR & FLASH_PECR_PROG))
        FLASH->PECR |= FLASH_PECR_PROG;             /* (2) Set the PROG bit in the FLASH_PECR register to choose program page */
    while ((FLASH->SR & FLASH_SR_BSY) != 0);        /* (3) Wait for the BSY bit to be cleared */
    *(uint32_t*)page_addr = 0x00000000;             /* (4) Write 0x0000 0000 to the first word of the program page to erase */
#endif

    return OK;
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

// IRQ should be disabled
// addr and size also checked
// size -  in bytes
int flash_copy(uint32_t dst_addr, uint32_t src_addr, unsigned int bytes_to_copy)
{
//    int res;
    unsigned int i;
    // trim values
    unsigned int start_addr = dst_addr & ~(FLASH_SECTOR_SIZE - 1);
    unsigned int size = (bytes_to_copy + 3) >> 2;
    unsigned int end_addr = (dst_addr + size + FLASH_SECTOR_SIZE) & ~(FLASH_SECTOR_SIZE - 1);

    __DSB();
    __ISB();

    flash_cmd_unlock();

    for(i = start_addr; i < end_addr; i += FLASH_SECTOR_SIZE)
        flash_cmd_erase_page(i);

#if defined(STM32L1)
    // disable ERASE
    FLASH->PECR &= ~(FLASH_PECR_ERASE);
#endif

    for (i = 0; i < size; ++i)
    {
        flash_cmd_program_word(dst_addr, *(unsigned int*)src_addr);
        dst_addr += sizeof(uint32_t);
        src_addr += sizeof(uint32_t);
    }

    flash_cmd_lock();

    return 0;
}


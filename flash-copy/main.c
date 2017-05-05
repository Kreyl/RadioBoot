/*
 * main.c
 *
 *  Created on: 3 мая 2017 г.
 *      Author: RLeonov
 */

#include <stdbool.h>
#include <stdint.h>
#include "stm32.h"
#include "errors.h"

#ifdef STM32F0
#define FLASH_SECTOR_SIZE                       2048
#define FLASH_SECTOR_SIZE_WORDS                 (FLASH_SECTOR_SIZE / sizeof(uint32_t))
#endif

static void flash_cmd_unlock()
{
    /* (1) Wait till no operation is on going */
    /* (2) Check that the Flash is unlocked */
    /* (3) Perform unlock sequence */
#ifdef STM32F0
    while ((FLASH->SR & FLASH_SR_BSY) != 0) { __NOP(); __NOP(); }   /* (1) */
#endif

#ifdef STM32F0
    if ((FLASH->CR & FLASH_CR_LOCK) != 0)                           /* (2) */
    {
        FLASH->KEYR = FLASH_KEY1;                                   /* (3) */
        FLASH->KEYR = FLASH_KEY2;
    }
#endif
}

static inline void flash_cmd_lock()
{
    /* lock FLASH */
#ifdef STM32F0
    FLASH->CR |= FLASH_CR_LOCK;
#endif
}

static void flash_cmd_program_prepare()
{
    /* (1) Set the PG bit in the FLASH_CR register to enable programming */
#ifdef STM32F0
    FLASH->CR |= FLASH_CR_PG;
#endif
    /* (2) Performa program */
}

static int flash_cmd_program_execute()
{
    /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
    /* (4) Check the EOP flag in the FLASH_SR register */
    /* (5) clear it by software by writing it at 1 */
    /* (6) Reset the PG Bit to disable programming */
#ifdef STM32F0
    while ((FLASH->SR & FLASH_SR_BSY) != 0) {}

    if ((FLASH->SR & FLASH_SR_EOP) != 0)
        FLASH->SR = FLASH_SR_EOP;
    else
    {
        /* manage the error cases */
    }
    FLASH->CR &= ~FLASH_CR_PG;
#endif

    return OK;
}

static inline int flash_cmd_erase_page(uint32_t page_addr)
{
    /* (1) Set the PER bit in the FLASH_CR register to enable page erasing */
    /* (2) Program the FLASH_AR register to select a page to erase */
    /* (3) Set the STRT bit in the FLASH_CR register to start the erasing */
    /* (4) Wait until the BSY bit is reset in the FLASH_SR register */
    /* (5) Check the EOP flag in the FLASH_SR register */
    /* (6) Clear EOP flag by software by writing EOP at 1 */
    /* (7) Reset the PER Bit to disable the page erase */
#ifdef STM32F0
    FLASH->CR |= FLASH_CR_PER;                      /* (1) */
    FLASH->AR = page_addr;                          /* (2) */
    FLASH->CR |= FLASH_CR_STRT;                     /* (3) */
    while ((FLASH->SR & FLASH_SR_BSY) != 0) {}      /* (4) */
    if ((FLASH->SR & FLASH_SR_EOP) != 0)            /* (5) */
        FLASH->SR = FLASH_SR_EOP;                   /* (6) */
    else
    {
        /* Manage the error cases */
    }
    FLASH->CR &= ~FLASH_CR_PER;                     /* (7) */
#endif
    return OK;
}

static inline int flash_cmd_program_word(unsigned int addr, unsigned int data)
{
    flash_cmd_program_prepare();
    *((uint16_t*)addr) = (uint16_t)data;
    *((uint16_t*)(addr + sizeof(uint16_t))) = (uint16_t)(data >> 16);
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

    for (i = 0; i < size; ++i)
    {
        flash_cmd_program_word(dst_addr, *(unsigned int*)src_addr);
        dst_addr += sizeof(uint32_t);
        src_addr += sizeof(uint32_t);
    }

    flash_cmd_lock();

    return 0;
}


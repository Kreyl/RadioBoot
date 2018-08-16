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
#elif defined(STM32L0)
#define FLASH_PAGE_SIZE                       128
#define FLASH_HALF_PAGE_SIZE                  64
#elif defined(STM32L1)
#define FLASH_PAGE_SIZE                       256
#define FLASH_HALF_PAGE_SIZE                  128
#endif

#define HALT()                                 for(;;)

static inline void flash_cmd_unlock()
{
    /* (1) Wait till no operation is on going */
    while ((FLASH->SR & FLASH_SR_BSY) != 0);

#if defined(STM32F0)
    /* (2) Check that the Flash is unlocked */
    if ((FLASH->CR & FLASH_CR_LOCK) != 0)
    {
    /* (3) Perform unlock sequence */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
#elif defined(STM32L0) || defined(STM32L1)
    if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0)
    {
        /* Perform unlock sequence */
        FLASH->PEKEYR = FLASH_PEKEY1;
        FLASH->PEKEYR = FLASH_PEKEY2;
#if defined(STM32L1)
        FLASH->SR = FLASH_SR_WRPERR;
        FLASH->PECR &= ~FLASH_PECR_FTDW;
#endif // STM32L1
        while((FLASH->PECR & FLASH_PECR_PELOCK) != 0);
    }

    /* Wait till no operation is on going */
    while ((FLASH->SR & FLASH_SR_BSY) != 0);

    /* Check that the PELOCK is unlocked */
    if ((FLASH->PECR & FLASH_PECR_PELOCK) == 0)
    {
        if((FLASH->PECR & FLASH_PECR_PRGLOCK) != 0)
        {
            FLASH->PRGKEYR = FLASH_PRGKEY1;
            FLASH->PRGKEYR = FLASH_PRGKEY2;
            while((FLASH->PECR & FLASH_PECR_PRGLOCK) != 0);
        }
    }
#endif // STM32L0 || STM32L1
}

static inline void flash_cmd_lock()
{
    /* Wait till no operation is on going */
    while ((FLASH->SR & FLASH_SR_BSY) != 0);
    /* lock FLASH */
#if defined(STM32F0)
    FLASH->CR |= FLASH_CR_LOCK;
#elif defined(STM32L1)
    FLASH->PECR |= FLASH_PECR_PELOCK;
    FLASH->PECR |= FLASH_PECR_PRGLOCK;
#elif defined(STM32L0)
    FLASH->PECR |= FLASH_PECR_PELOCK;
#endif
}

#if defined(STM32F0)
static void flash_cmd_program_prepare()
{
    FLASH->CR |= FLASH_CR_PG;                       /* (1) Set the PG bit in the FLASH_CR register to enable programming */
                                                    /* (2) Perform program */
}

static int flash_cmd_program_execute()
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0) {}      /* (3) Wait until the BSY bit is reset in the FLASH_SR register */

    if ((FLASH->SR & FLASH_SR_EOP) != 0)            /* (4) Check the EOP flag in the FLASH_SR register */
        FLASH->SR = FLASH_SR_EOP;                   /* (5) clear it by software by writing it at 1 */
    else
    {
        /* manage the error cases */
    }
    FLASH->CR &= ~FLASH_CR_PG;                      /* (6) Reset the PG Bit to disable programming */
    return 0;
}
#endif // STM32F0

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


#if defined(STM32L0) || defined(STM32L1)
static inline void flash_cmd_enable_half_page_write()
{
    /* Set the FPRG bit in the FLASH_PECR register (this configures FLASH_PECR to
    perform a data buffer loading sequence) */
    if(!(FLASH->PECR & FLASH_PECR_FPRG))
        FLASH->PECR |= FLASH_PECR_FPRG;
    /* Set the PROG bit in the FLASH_PECR register to access the required program
    memory page */
    if(!(FLASH->PECR & FLASH_PECR_PROG))
        FLASH->PECR |= FLASH_PECR_PROG;
    /* Wait for the BSY bit to be cleared */
    while ((FLASH->SR & FLASH_SR_BSY) != 0);
    /* Wait for the EOP bit to be cleared */
    while ((FLASH->SR & FLASH_SR_EOP) != 0);
}

static inline void flash_cmd_disable_half_page_write()
{
    FLASH->PECR &= ~(FLASH_PECR_PROG | FLASH_PECR_FPRG); /* (6) */
}

#endif //STM32L0
// FLASH memory must be already unlocked.
static inline int flash_cmd_erase(uint32_t addr)
{
#if defined(STM32F0)
    FLASH->CR |= FLASH_CR_PER;                      /* (1) Set the PER bit in the FLASH_CR register to enable page erasing */
    FLASH->AR = addr;                               /* (2) Program the FLASH_AR register to select a page to erase */
    FLASH->CR |= FLASH_CR_STRT;                     /* (3) Set the STRT bit in the FLASH_CR register to start the erasing */
    while ((FLASH->SR & FLASH_SR_BSY) != 0)         /* (4) Wait until the BSY bit is reset in the FLASH_SR register */
    if ((FLASH->SR & FLASH_SR_EOP) != 0)            /* (5) Check the EOP flag in the FLASH_SR register */
        FLASH->SR = FLASH_SR_EOP;                   /* (6) Clear EOP flag by software by writing EOP at 1 */
    else
    {
        /* Manage the error cases */
    }
    FLASH->CR &= ~FLASH_CR_PER;                     /* (7) Reset the PER Bit to disable the page erase */
#elif defined(STM32L0)
    FLASH->PECR |= FLASH_PECR_ERASE | FLASH_PECR_PROG;  /* (1) Set the ERASE and PROG bits in the FLASH_PECR register to enable page erasing */
    *(uint32_t *)addr = (uint32_t)0;                    /* (2) Write a 32-bit word value in an address of the selected page to start the erase sequence */
    while ((FLASH->SR & FLASH_SR_BSY) != 0);            /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
    if ((FLASH->SR & FLASH_SR_EOP) != 0)                /* (4) Check the EOP flag in the FLASH_SR register */
        FLASH->SR = FLASH_SR_EOP;                       /* (5) Clear EOP flag by software by writing EOP at 1 */
    else
    {
        /* Manage the error cases */
    }
    FLASH->PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_PROG);
#elif defined(STM32L1)
    unsigned int aligned_addr = addr & ~(FLASH_PAGE_SIZE - 1);
    flash_cmd_enable_erase();
    *(uint32_t*)aligned_addr = 0x00000000;                  /* (4) Write 0x0000 0000 to the first word of the program page to erase */
    flash_cmd_disable_erase();
#endif

    return 0;
}

#if defined(STM32L0) || defined(STM32L1)
static inline int flash_cmd_program_half_page(unsigned int addr, unsigned int data)
{
#if defined(STM32L1)
    uint32_t *src = (uint32_t*)data;
#endif // STM32L1
    uint32_t *dst = (uint32_t*)addr;
    /* Directly write half a page with 32 different words to the program memory address
    space. The words must be written sequentially starting from word 0 and ending with
    word 31 */
    flash_cmd_enable_half_page_write();

    for(int i = 0; i < (FLASH_HALF_PAGE_SIZE >> 2); i++)
    {
#if defined(STM32L1)
        dst[i] = src[i];
#elif defined(STM32L0)
        *(uint32_t*)(addr) = *dst++;
#endif
    }

    /* wait last operation */
    while ((FLASH->SR & FLASH_SR_BSY) != 0)
    {
        __NOP();
        __NOP();
    }

    flash_cmd_disable_half_page_write();
    return 0;
}
#endif //STM32L1

static inline int flash_cmd_program_word(unsigned int addr, unsigned int data)
{
#if defined(STM32F0)
    flash_cmd_program_prepare();
    *((uint16_t*)addr) = (uint16_t)data;
    *((uint16_t*)(addr + sizeof(uint16_t))) = (uint16_t)(data >> 16);
    return flash_cmd_program_execute();
#elif defined(STM32L0) || defined(STM32L1)
    *((uint32_t*)addr) = data;                  /* (1) Perform the data write (32-bit word) at the desired address */
    while ((FLASH->SR & FLASH_SR_BSY) != 0);    /* (2) Wait until the BSY bit is reset in the FLASH_SR register */
    return 0;
#endif
}

static inline void flash_cmd_program(uint32_t dst, uint32_t src, int size)
{
    unsigned int last_word = 0;
#if defined(STM32L0) || defined(STM32L1)
    uint8_t buffer[FLASH_HALF_PAGE_SIZE] = { 0 };
    /* program half pages of source */
    for(; size >= FLASH_HALF_PAGE_SIZE; size -= FLASH_HALF_PAGE_SIZE, dst += FLASH_HALF_PAGE_SIZE, src += FLASH_HALF_PAGE_SIZE)
    {
        for(int i = 0; i < FLASH_HALF_PAGE_SIZE; i++)
            buffer[i] = ((uint8_t*)src)[i];
        flash_cmd_program_half_page(dst, (unsigned int)buffer);
    }
#endif // STM32L1

    /* program full words of source */
    for (; size >= sizeof(uint32_t); size -= sizeof(uint32_t), dst += sizeof(uint32_t), src += sizeof(uint32_t))
        flash_cmd_program_word(dst, *(unsigned int*)src);

    /* program last word. If word is not full - other bytes will be 0x00 */
    while(size-- != 0)
    {
        last_word <<= 8;
        last_word |= *(uint8_t*)(src + size);
    }
    flash_cmd_program_word(dst, last_word);
}

// IRQ should be disabled
// addresses and bytes_to_copy also had checked
// bytes_to_copy - in bytes
int flash_update(unsigned int dst_addr, unsigned int src_addr, int bytes_to_copy)
{
    __DSB();
    __ISB();

    /* unlock flash */
    flash_cmd_unlock();

    // program first total pages
    for(; bytes_to_copy > FLASH_PAGE_SIZE; dst_addr += FLASH_PAGE_SIZE, src_addr += FLASH_PAGE_SIZE, bytes_to_copy -= FLASH_PAGE_SIZE)
    {
        flash_cmd_erase(dst_addr);
        flash_cmd_program(dst_addr, src_addr, FLASH_PAGE_SIZE);
    }
    // program last page
    flash_cmd_erase(dst_addr);
    flash_cmd_program(dst_addr, src_addr, bytes_to_copy);

    /* lock flash */
    flash_cmd_lock();

    /* Reset core */
    NVIC_SystemReset();
    /* Never return */
    HALT();
    return 0;
}

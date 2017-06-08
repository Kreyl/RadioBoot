/*
 * stm32_power.c
 *
 *  Created on: 8 θώνÿ 2017 γ.
 *      Author: RLeonov
 */

#include <stdbool.h>
#include "stm32.h"
#include "stm32_power.h"


#if defined(STM32F1)
#define MAX_APB2                             72000000
#define MAX_APB1                             36000000
#define MAX_ADC                              14000000
#elif defined(STM32F2)
#define MAX_APB2                             60000000
#define MAX_APB1                             30000000
#elif defined(STM32F401) || defined(STM32F411)
#define MAX_APB2                             100000000
#define MAX_APB1                             50000000
#elif defined(STM32F405) || defined(STM32F407) || defined(STM32F415) || defined(STM32F417)
#define MAX_APB2                             84000000
#define MAX_APB1                             42000000
#elif defined(STM32F427) || defined(STM32F429) || defined(STM32F437) || defined(STM32F439)
#define MAX_APB2                             90000000
#define MAX_APB1                             45000000
#elif defined(STM32L0)
#define MAX_APB2                             32000000
#define MAX_APB1                             32000000
#define HSI_VALUE                            16000000
#elif defined(STM32L1)
#define MAX_AHB                              32000000
#define MAX_APB2                             32000000
#define MAX_APB1                             32000000
#define HSI_VALUE                            16000000
#elif defined(STM32F0)
#define MAX_APB2                             0
#define MAX_APB1                             48000000
#define HSI_VALUE                            8000000
#endif

#if defined(STM32F1) || defined(STM32L1) || defined(STM32L0) || defined(STM32F0)
#define PPRE1_POS                            8
#define PPRE2_POS                            11
#elif defined(STM32F0)
#define PPRE1_POS                            8
#elif defined(STM32F2) || defined(STM32F4)
#define PPRE1_POS                            10
#define PPRE2_POS                            13
#endif

#ifndef RCC_CSR_WDGRSTF
#define RCC_CSR_WDGRSTF RCC_CSR_IWDGRSTF
#endif

#ifndef RCC_CSR_PADRSTF
#define RCC_CSR_PADRSTF RCC_CSR_PINRSTF
#endif


typedef enum {
    //only if HSE value is set
    STM32_CLOCK_SOURCE_HSE = 0,
    STM32_CLOCK_SOURCE_HSI,
#if defined(STM32L0) || defined(STM32L1)
    STM32_CLOCK_SOURCE_MSI,
#endif //STM32L0
    STM32_CLOCK_SOURCE_PLL,
} STM32_CLOCK_SOURCE_TYPE;


static inline int stm32_power_get_pll_clock()
{
    unsigned int pllsrc = HSI_VALUE;
    unsigned int mul = (1 << ((RCC->CFGR >> 19) & 7)) * (3 + ((RCC->CFGR >> 18) & 1));
    unsigned int div = ((RCC->CFGR >> 22) & 3) + 1;

#if (HSE_VALUE)
    if (RCC->CFGR & (1 << 16))
        pllsrc = HSE_VALUE;
#endif
    return pllsrc * mul / div;
}

static int get_msi_clock()
{
    return 65536 * (1 << ((RCC->ICSCR >> 13) & 7));
}

static void stm32_power_set_vrange(int vrange)
{
    // wait stable
    while((PWR->CSR & PWR_CSR_VOSF) != 0);
    PWR->CR = ((PWR->CR) & ~(3 << 11)) | ((vrange & 3) << 11);
    // wait stable
    while((PWR->CSR & PWR_CSR_VOSF) != 0);
}

static inline void stm32_power_hsi_on()
{
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0) {}
}

static bool stm32_power_pll_on(STM32_CLOCK_SOURCE_TYPE src)
{
    unsigned int pow, mul;
    mul = PLL_MUL;
    for (pow = 0; mul > 4; mul /= 2, pow++) {}

    RCC->CFGR &= ~((0xf << 18) | (3 << 22));
    RCC->CFGR |= (((pow << 1) | (mul - 3)) << 18);
    RCC->CFGR |= (((PLL_DIV - 1) << 22) | ((pow << 1) | (mul - 3)) << 18);

    stm32_power_set_vrange(1);

#if (HSE_VALUE)
    if (src == STM32_CLOCK_SOURCE_HSE)
        RCC->CFGR |= (1 << 16);
    else
#endif
    {
        stm32_power_hsi_on();
        RCC->CFGR &= ~(1 << 16);
    }
    //turn PLL on
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}
    return true;
}

#if (HSE_VALUE)
static bool stm32_power_hse_on()
{
    if ((RCC->CR & RCC_CR_HSEON) == 0)
    {
#if defined(STM32L0) && !(LSE_VALUE)
        RCC->CR &= ~(3 << 20);
        RCC->CR |= (30 - __builtin_clz(HSE_VALUE / 1000000)) << 20;
#endif //STM32L0 && !LSE_VALUE

#if (HSE_BYPASS)
        RCC->CR |= RCC_CR_HSEON | RCC_CR_HSEBYP;
#else
        RCC->CR |= RCC_CR_HSEON;
#endif //HSE_BYPAS
#if defined(HSE_STARTUP_TIMEOUT)
        int i;
        for (i = 0; i < HSE_STARTUP_TIMEOUT; ++i)
            if (RCC->CR & RCC_CR_HSERDY)
                return true;
        return false;
#else
        while ((RCC->CR & RCC_CR_HSERDY) == 0) {}
#endif //HSE_STARTUP_TIMEOUT
    }
    return true;
}
#endif //HSE_VALUE

static unsigned int stm32_power_get_bus_prescaller(unsigned int clock, unsigned int max)
{
    int i;
    for (i = 0; i <= 4; ++i)
        if ((clock >> i) <= max)
            break;
    return i ? i + 3 : 0;
}

static void power_set_clock_source(STM32_CLOCK_SOURCE_TYPE src)
{
    unsigned int sw, core_clock, pll_src;
    pll_src = STM32_CLOCK_SOURCE_HSI;

#if (HSE_VALUE)
    if ((src == STM32_CLOCK_SOURCE_HSE) && !stm32_power_hse_on())
        src = STM32_CLOCK_SOURCE_HSI;
    if ((src == STM32_CLOCK_SOURCE_PLL) && stm32_power_hse_on())
        pll_src = STM32_CLOCK_SOURCE_HSE;
#endif //HSE_VALUE

    switch (src)
    {
#if defined (STM32L0) || defined (STM32L1)
    case STM32_CLOCK_SOURCE_MSI:
        sw = RCC_CFGR_SW_MSI;
        core_clock = get_msi_clock();
        break;
#endif
#if (HSE_VALUE)
    case STM32_CLOCK_SOURCE_HSE:
        sw = RCC_CFGR_SW_HSE;
        core_clock = HSE_VALUE;
        break;
#endif
    case STM32_CLOCK_SOURCE_PLL:
        if (stm32_power_pll_on(pll_src))
        {
            sw = RCC_CFGR_SW_PLL;
            core_clock = stm32_power_get_pll_clock();
            break;
        }
        //follow down
    default:
        sw = RCC_CFGR_SW_HSI;
        core_clock = HSI_VALUE;
    }

    //setup bases
    //AHB. Can operates at maximum clock
#if (MAX_APB2)
    //APB2
    RCC->CFGR = (RCC->CFGR & ~(7 << PPRE2_POS)) | (stm32_power_get_bus_prescaller(core_clock, MAX_APB2) << PPRE2_POS);
#endif //MAX_APB2
    //APB1
    RCC->CFGR = (RCC->CFGR & ~(7 << PPRE1_POS)) | (stm32_power_get_bus_prescaller(core_clock, MAX_APB1) << PPRE1_POS);

#if defined(STM32F1)
#if (STM32_ADC_DRIVER)
    //APB2 can operates at maximum in F1
    unsigned int adc_psc = (((core_clock / MAX_ADC) + 1) & ~1) >> 1;
    if (adc_psc)
        --adc_psc;
    RCC->CFGR = (RCC->CFGR & ~(3 << 14)) | (adc_psc << 14);
#endif // STM32_ADC_DRIVER
#endif //STM32F1

    //tune flash latency
#if defined(STM32F1) && !defined(STM32F100)
    FLASH->ACR = FLASH_ACR_PRFTBE | ((core_clock - 1) / 24000000);
#elif defined(STM32F2) || defined(STM32F4)
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | ((core_clock - 1) / 30000000);
#elif defined(STM32L0)
    FLASH->ACR = FLASH_ACR_PRE_READ | ((core_clock - 1) / 16000000);
#elif defined(STM32L1)
    FLASH->ACR |= FLASH_ACR_ACC64;
    FLASH->ACR |= FLASH_ACR_PRFTEN;

    unsigned int range = ((PWR->CR >> 11) & 3);
    if( (range == 3 && core_clock > 2000000) ||
        (range == 2 && core_clock > 8000000) ||
        (range == 1 && core_clock > 16000000))
    {
        FLASH->ACR |= FLASH_ACR_LATENCY;
    }
    else
        FLASH->ACR &= ~FLASH_ACR_LATENCY;

#elif defined(STM32F0)
    FLASH->ACR = FLASH_ACR_PRFTBE | ((core_clock / 24000000) << 0);
#endif
    __DSB();
    __DMB();
    __NOP();
    __NOP();

    //switch to source
    RCC->CFGR |= sw;
    while (((RCC->CFGR >> 2) & 3) != sw) {}
    __NOP();
    __NOP();
}

int get_ahb_clock()
{
    int div = 1;
    if (RCC->CFGR & (1 << 7))
        div = 1 << (((RCC->CFGR >> 4) & 7) + 1);
    if (div >= 32)
        div <<= 1;
    return power_get_core_clock() / div;
}

#if (MAX_APB2)
int get_apb2_clock()
{
    int div = 1;
    if (RCC->CFGR & (1 << (PPRE2_POS + 2)))
        div = 1 << (((RCC->CFGR >> PPRE2_POS) & 3) + 1);
    return get_ahb_clock() /div;
}
#endif //MAX_APB2

int get_apb1_clock()
{
    int div = 1;
    if (RCC->CFGR & (1 << (PPRE1_POS + 2)))
        div = 1 << (((RCC->CFGR >> PPRE1_POS) & 3) + 1);
    return get_ahb_clock() / div;
}

#if defined(STM32F1)
int get_adc_clock()
{
    return get_apb2_clock() / ((((RCC->CFGR >> 14) & 3) + 1) * 2);
}
#endif

unsigned int power_get_clock(POWER_CLOCK_TYPE clock_type)
{
    unsigned int res = 0;
    switch (clock_type)
    {
    case POWER_CORE_CLOCK:
        res = power_get_core_clock();
        break;
    case POWER_BUS_CLOCK:
        res = get_ahb_clock();
        break;
    case POWER_CLOCK_APB1:
#if (MAX_APB2)
        res = get_apb2_clock();
        break;
#endif //MAX_APB2
    case POWER_CLOCK_APB2:
        res = get_apb1_clock();
        break;
#if defined(STM32F1)
    case POWER_CLOCK_ADC:
        res = get_adc_clock();
        break;
#endif
    default:
        break;
    }
    return res;
}

unsigned int power_get_core_clock()
{
    switch (RCC->CFGR & (3 << 2))
    {
    case RCC_CFGR_SWS_HSI:
        return HSI_VALUE;
        break;
#if defined(STM32L0) || defined(STM32L1)
    case RCC_CFGR_SWS_MSI:
        return get_msi_clock();
        break;
#endif
    case RCC_CFGR_SWS_HSE:
        return HSE_VALUE;
        break;
    case RCC_CFGR_SWS_PLL:
        return stm32_power_get_pll_clock();
    }
    return 0;
}

void power_init()
{
    RCC->APB1ENR = 0;
    RCC->APB2ENR = 0;

#if defined(STM32F1) || defined(STM32L1) || defined(STM32L0) || defined(STM32F0)
    RCC->AHBENR = 0;
#else
    RCC->AHB1ENR = 0;
    RCC->AHB2ENR = 0;
    RCC->AHB3ENR = 0;
#endif

    RCC->CFGR = 0;

#if defined(STM32F10X_CL) || defined(STM32F100)
    RCC->CFGR2 = 0;
#elif defined(STM32F0)
    RCC->CFGR2 = 0;
    RCC->CFGR3 = 0;
#endif

    power_set_clock_source(STM32_CLOCK_SOURCE_HSI);
}

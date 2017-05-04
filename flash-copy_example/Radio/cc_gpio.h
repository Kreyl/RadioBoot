/*
 * cc_gpio.h
 *
 *  Created on: Nov 28, 2013
 *      Author: kreyl
 */

#pragma once

// Pins
#define CC_GPIO     GPIOA
#define CC_GDO2     2
#define CC_GDO0     3
#define CC_SCK      5
#define CC_MISO     6
#define CC_MOSI     7
#define CC_CS       4

#if defined STM32L1XX
    #if CC_GDO0 == 0
    #define GDO0_IRQ_HANDLER     Vector58
    #elif CC_GDO0 == 1
    #define GDO0_IRQ_HANDLER     Vector5C
    #elif CC_GDO0 == 2
    #define GDO0_IRQ_HANDLER     Vector60
    #elif CC_GDO0 == 3
    #define GDO0_IRQ_HANDLER     Vector64
    #elif CC_GDO0 == 4
    #define GDO0_IRQ_HANDLER     Vector68
    #endif

#elif defined STM32F072xB
    #if (CC_GDO0 == 0) || (CC_GDO0 == 1)
    #define GDO0_IRQ_HANDLER     Vector54
    #elif (CC_GDO0 == 2) || (CC_GDO0 == 3)
    #define GDO0_IRQ_HANDLER     Vector58
    #else
    #define GDO0_IRQ_HANDLER     Vector5C
    #endif
#endif

// SPI
#define CC_SPI      SPI1
#if defined STM32L1XX
#define CC_SPI_AF   AF5
#elif defined STM32F030 || defined STM32F072xB
#define CC_SPI_AF   AF0
#endif

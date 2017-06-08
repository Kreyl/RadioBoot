/*
 * stm32_power.h
 *
 *  Created on: 8 θώνÿ 2017 γ.
 *      Author: RLeonov
 */

#ifndef STM32_POWER_H_
#define STM32_POWER_H_

typedef enum {
    POWER_CORE_CLOCK = 0,
    POWER_BUS_CLOCK,
    POWER_CLOCK_APB1,
    POWER_CLOCK_APB2,
    POWER_CLOCK_ADC,
    POWER_CLOCK_MAX
} POWER_CLOCK_TYPE;

void power_init();
unsigned int power_get_clock(POWER_CLOCK_TYPE clock_type);
unsigned int power_get_core_clock();


#endif /* STM32_POWER_H_ */

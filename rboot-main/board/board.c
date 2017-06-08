/*
 * board.c
 *
 *  Created on: 5 θώνÿ 2017 γ.
 *      Author: RLeonov
 */

#include "stm32_power.h"
#include "stm32.h"

void board_init()
{
    power_init();
//    flash_init();
}

void board_reset()
{
    NVIC_SystemReset();
}

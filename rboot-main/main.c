/*
 * main.c
 *
 *  Created on: 4 θώνÿ 2017 γ.
 *      Author: RomaJam
 */

#include "board.h"
#include "system.h"
#include "config.h"
#include "dbg.h"

#include "board/ST/stm32_pin.h"


int main(void)
{
//    SYSTEM system;
    board_init();

    gpio_enable(B0, GPIO_MODE_OUT);
    pin_reset(B0);


#if (DFU_DEBUG)
    board_dbg_init();
    printf("BOOT APP, v %d.%d \n", VERSION >> 8, VERSION & 0xff);
    printf("CPU %d MHz\n", power_get_core_clock() / 1000000);
#endif // DFU_DEBUG

    for(;;) {}

}

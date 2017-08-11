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
    SYSTEM system;
    board_init();

#if (DFU_DEBUG)
    board_dbg_init();
    printf("RBOOT main, v %d.%d \n", VERSION >> 8, VERSION & 0xff);
    printf("CPU %d MHz\n", power_get_core_clock() / 1000000);
#endif // DFU_DEBUG

    system.reboot = false;

    // Enable LED
    gpio_enable(B1, GPIO_MODE_OUT);
    pin_set(B1);

    while(!system.reboot)
    {
//        radio_request(system);
    }

    /* reset system */
    board_reset();

    for(;;) {}
}

/*
 * main.c
 *
 *  Created on: 4 θώνÿ 2017 γ.
 *      Author: RomaJam
 */

#include "board/ST/stm32_pin.h"
#include "board.h"
#include "system.h"
#include "config.h"

//#include "f_upd_test.h"

//#include "test.h"
#include "flash_update.h"

#if (DFU_DEBUG)
#include "dbg.h"
#endif

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

    //flash_update(FLASH_BASE, 0x08001000, 2048 + 102);

    //flash_update(FLASH_BASE, 0x08001000, 27444);

    uint8_t ram[FLASH_UPD_SIZE] = { 0 };
    memcpy(ram, __FLASH_UPD, FLASH_UPD_SIZE);
#if (DFU_DEBUG)
    printf("Update firmware...\n");
#endif

    __disable_irq();
    flash_upd_sram(ram, FLASH_BASE, 0x08001000, 27444);


    while(!system.reboot)
    {
//        radio_request(system);
    }

    /* reset system */
    board_reset();

    for(;;) {}
}

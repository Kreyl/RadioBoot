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

#define RAM

#ifdef RAM
#include "flash_update_STM32L052K8.h"
#else
#include "f_upd_test.h"
#endif

//#include "test.h"
#if (DFU_DEBUG)
#include "dbg.h"
#endif
#define FLASH_TARGET

int main(void)
{
    SYSTEM system;
    board_init();

#if (DFU_DEBUG)
    board_dbg_init();
    printf("RBOOT main, v %d.%d \n", VERSION >> 8, VERSION & 0xff);
    printf("CPU %d MHz\n", power_get_core_clock() / 1000000);
#endif // DFU_DEBUG

    delay_ms(999);

    system.reboot = false;

    // Enable LED
    gpio_enable(B0, GPIO_MODE_OUT);
    pin_set(B0);

#ifdef RAM
    uint8_t ram[FLASH_UPD_SIZE] = { 0 };
    memcpy(ram, __FLASH_UPD, FLASH_UPD_SIZE);
#endif

#if (DFU_DEBUG)
#if defined(RAM)
    printf("Update firmware from RAM...\n");
#else
    printf("Update firmware...\n");
#endif
#endif
    __disable_irq();

#ifdef RAM
    flash_upd_sram(ram, 0x08007000, 0x08000000, 128, false);
#else
    flash_update(0x08007000, 0x08000000, 2048);
#endif

    __enable_irq();

#if (DFU_DEBUG)
    printf("OK\n");
#endif

    while(!system.reboot)
    {
//        radio_request(system);
    }

    for(;;) {}
}

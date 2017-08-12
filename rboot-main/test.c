/*
 * test.c
 *
 *  Created on: 12 рту. 2017 у.
 *      Author: RomaJam
 */

#include "test.h"
#include "flash_update.h"
#include "dbg.h"

#define FLASH_TARGET            0x08007000

#define TEST_FROM_FLASH         0
#define TEST_FROM_SRAM          1

void dump(char* s, uint8_t* buf, int size)
{
    printf("%s: ", s);
    for(int i = 0; i < size; i++)
        printf("%0.2X ", buf[i]);
    printf("\n");
}

void flash_update_test()
{

    uint8_t null[32] = { 0 };
    uint8_t buffer[32] =
    {
            0xA1, 0xA2, 0xA3, 0xA4,  0xA5, 0xA6, 0xA7, 0xA8,  0xB1, 0xB2, 0xB3, 0xB4,  0xB5, 0xB6, 0xB7, 0xB8,
            0xC1, 0xC2, 0xC3, 0xC4,  0xC5, 0xC6, 0xC7, 0xC8,  0xD1, 0xD2, 0xD3, 0xD4,  0xD5, 0xD6, 0xD7, 0xD8
    };

    printf("Target FLASH 0x%08X\n", FLASH_TARGET);

#if (TEST_FROM_FLASH)
    for (int i = 0; i <= 32; i++)
    {
        printf("==========================================\n");
        printf("Test execute from FLASH: copy %u bytes\n", i);
        printf("Erase target flash...");
        flash_upd(FLASH_TARGET, (unsigned int)null, 32);
        printf("OK\n");

        dump("Before: ", (uint8_t*)FLASH_TARGET, 32);

        flash_upd(FLASH_TARGET, (unsigned int)buffer, i);

        dump("After: ", (uint8_t*)FLASH_TARGET, 32);
    }
#endif // TEST_FROM_FLASH

#if (TEST_FROM_SRAM)
    uint8_t ram[FLASH_UPD_SIZE] = { 0 };
    memcpy(ram, __FLASH_UPD, FLASH_UPD_SIZE);
    for (int i = 0; i <= 32; i++)
    {
        printf("==========================================\n");
        printf("Test execute from SRAM: copy %u bytes\n", i);
        printf("Erase target flash...");
        flash_upd_sram(ram, FLASH_TARGET, (unsigned int)null, 32);
        printf("OK\n");

        dump("Before: ", (uint8_t*)FLASH_TARGET, 32);

        flash_upd_sram(ram, FLASH_TARGET, (unsigned int)buffer, i);

        dump("After: ", (uint8_t*)FLASH_TARGET, 32);
    }
#endif // TEST_FROM_SRAM

}

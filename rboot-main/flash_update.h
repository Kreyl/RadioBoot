/*
*      Author: RL
*/

#ifndef FLASH_UPDATE_H
#define FLASH_UPDATE_H

#include <stdint.h>
#include <string.h>

/*
* Automatically composed file - DO NOT CHANGE
* Target MCU: STM32L151C8
* IMPORTANT: 
* For update FLASH from SRAM, function __FLASH_UPD 
* had to be placed in "sram_func" before using macros "flash_upd_sram".
*/

#define flash_upd(dst, src, size)                   ((FLASH_UPD_TYPE)((unsigned int)__FLASH_UPD + 1))(dst, src, size)
#define flash_upd_sram(sram_func, dst, src, size)   ((FLASH_UPD_TYPE)((unsigned int)sram_func + 1))(dst, src, size)

#define FLASH_UPD_SIZE                   708

extern const uint8_t __FLASH_UPD[FLASH_UPD_SIZE];

typedef int (*FLASH_UPD_TYPE)(unsigned int, unsigned int, int);

#endif // FLASH_UPDATE_H

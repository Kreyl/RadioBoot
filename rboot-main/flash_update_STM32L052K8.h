/*
*      Author: RL
*/

#ifndef FLASH_UPDATE_STM32L052K8_H
#define FLASH_UPDATE_STM32L052K8_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*
* Automatically composed file - DO NOT CHANGE
* Target MCU: STM32L052K8
* IMPORTANT: 
* For update FLASH from SRAM, function __FLASH_UPD 
* had to be placed in "sram_func" before using macros "flash_upd_sram".
*/

#define flash_upd(dst, src, size, reset)                   ((FLASH_UPD_TYPE)((unsigned int)__FLASH_UPD + 1))(dst, src, size, reset)
#define flash_upd_sram(sram_func, dst, src, size, reset)   ((FLASH_UPD_TYPE)((unsigned int)sram_func + 1))(dst, src, size, reset)

#define FLASH_UPD_SIZE                   624

extern const uint8_t __FLASH_UPD[FLASH_UPD_SIZE];

typedef int (*FLASH_UPD_TYPE)(unsigned int, unsigned int, int, bool);

#endif // FLASH_UPDATE_STM32L052K8_H

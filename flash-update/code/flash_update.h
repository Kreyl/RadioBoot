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
* Call usage:
* *((FLASH_COPY_FN_TYPE)((unsigned int)__FLASH_COPY_FN + 1))(src_addr, dest_addr, size);
*/

#define FLASH_UPD_FN_SIZE                   276

extern const uint8_t __FLASH_UPD_FN[FLASH_UPD_FN_SIZE];

typedef int (*FLASH_UPD_FN_TYPE)(uint32_t, uint32_t, unsigned int);

#endif // FLASH_UPDATE_H

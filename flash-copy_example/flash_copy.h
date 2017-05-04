/*
*      Author: Roma Jam
*/

#ifndef FLASH_COPY_H
#define FLASH_COPY_H

#include <stdint.h>

#define FLASH_COPY_FN_SIZE                   204

extern const uint8_t __FLASH_COPY_FN[FLASH_COPY_FN_SIZE];

typedef int (*FLASH_COPY_FN_TYPE)(uint32_t, uint32_t, unsigned int);

#endif // FLASH_COPY_H

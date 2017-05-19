/*
 * flash_copy_int.h
 *
 *  Created on: 19 мая 2017 г.
 *      Author: RLeonov
 */

#ifndef FLASH_COPY_INT_H_
#define FLASH_COPY_INT_H_

#include "../rexos/userspace/types.h"

// Return values
#define OK                      0
#define FAILURE                 1
#define TIMEOUT                 2
#define BUSY                    3
#define NEW                     4
#define IN_PROGRESS             5
#define LAST                    6
#define CMD_ERROR               7
#define WRITE_PROTECT           8
#define CMD_UNKNOWN             9
#define EMPTY                   10
#define NOT_A_NUMBER            11
#define OVERFLOW                12

int flash_copy(uint32_t dst_addr, uint32_t src_addr, unsigned int bytes_to_copy);


#endif /* FLASH_COPY_INT_H_ */

/*
 * board.h
 *
 *  Created on: 7 θώνÿ 2017 γ.
 *      Author: RomaJam
 */

#ifndef BOARD_H_
#define BOARD_H_


#include <stdbool.h>
#include "sys.h"
#include "board/ST/stm32_power.h"

//main
extern void board_init();
extern void board_reset();

//dbg
extern void board_dbg_init();
extern void board_dbg(const char *const buf, unsigned int size);

//delay
extern void delay_us(unsigned int us);
extern void delay_ms(unsigned int ms);


#endif /* BOARD_H_ */

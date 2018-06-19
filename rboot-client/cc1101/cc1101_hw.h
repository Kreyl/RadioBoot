/*
 * cc1101_hw.h
 *
 *  Created on: 19 θώνÿ 2018 γ.
 *      Author: RLeonov
 */

#ifndef CC1101_CC1101_HW_H_
#define CC1101_CC1101_HW_H_


#include "app.h"
#include "../rexos/userspace/io.h"
#include "cc1101_config.h"
#include "cc1101_defines.h"

typedef enum {
    CC1101_STATE_OFF = 0,
    CC1101_STATE_SLEEP,
    CC1101_STATE_IDLE,
    CC1101_STATE_BUSY,
    CC1101_STATE_TX,
    CC1101_STATE_TX_ACK,
    CC1101_STATE_RX,
} CC1101_HW_STATE;

typedef struct {
    CC1101_HW_STATE state;
    HANDLE process;
    IO* io;
    uint8_t status;
    uint8_t channel;
    uint8_t packet_size;
} CC1101_HW;

void cc1101_hw_init(CC1101_HW* cc1101);
void cc1101_hw_deinit(CC1101_HW* cc1101);

void cc1101_hw_reset(CC1101_HW* cc1101);
void cc1101_hw_calibrate(CC1101_HW* cc1101);
void cc1101_hw_set_channel(CC1101_HW* cc1101, uint8_t channel_num);
void cc1101_hw_set_tx_power(CC1101_HW* cc1101, uint8_t power);
void cc1101_hw_set_radio_pkt_size(CC1101_HW* cc1101, uint8_t size);
void cc1101_hw_tx(CC1101_HW* cc1101, HANDLE process, IO* io, unsigned int size);
void cc1101_hw_rx(CC1101_HW* cc1101, HANDLE process, IO* io, unsigned int size);

int cc1101_hw_receive_packet(CC1101_HW* cc1101, IO* io);

#endif /* CC1101_CC1101_HW_H_ */

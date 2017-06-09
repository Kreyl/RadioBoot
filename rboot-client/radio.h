/*
 * radio.h
 *
 *  Created on: 17 мая 2017 г.
 *      Author: RLeonov
 */

#ifndef RADIO_H_
#define RADIO_H_

#include "../rexos/userspace/sys.h"

extern const REX __RADIO;

typedef enum {
    RADIO_RESET = IPC_USER,
    RADIO_CALIBRATE,
    RADIO_SET_CHANNEL,
    RADIO_SET_POWER,
    RADIO_SET_PACKET_SIZE,
    RADIO_TX,
    RADIO_RX,
    RADIO_RX_ASYNC,
} RADIO_IPCS;

void radio_init(APP* app);

void radio_tx_sync(APP* app, uint8_t* data, unsigned int data_size);
unsigned int radio_rx_sync(APP* app, uint8_t* data);


#endif /* RADIO_H_ */

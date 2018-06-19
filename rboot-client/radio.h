/*
 * radio.h
 *
 *  Created on: 17 ��� 2017 �.
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
    RADIO_GET_PACKET,
} RADIO_IPCS;

void radio_init(APP* app);
void radio_tx_sync(APP* app, uint8_t* data, unsigned int data_size);
bool radio_rx_sync(APP* app, uint8_t* data, uint8_t data_size);


#endif /* RADIO_H_ */

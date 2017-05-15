/*
 * app_radio.h
 *
 *  Created on: 14 мая 2017 г.
 *      Author: RomaJam
 */

#ifndef APP_RADIO_H_
#define APP_RADIO_H_

#include "app.h"
#include "../rexos/userspace/process.h"

extern const REX __RADIO;

typedef enum {
    RADIO_RESET = IPC_USER,
    RADIO_CALIBRATE,
    RADIO_SET_CHANNEL,
    RADIO_SET_POWER,
    RADIO_SET_PACKET_SIZE,
    RADIO_TX,
    RADIO_RX
} RADIO_IPCS;

void app_radio_init(APP* app);

void app_radio_tx_sync(APP* app, uint8_t* data, unsigned int data_size);
void app_radio_rx(APP* app, uint8_t* data);


#endif /* APP_RADIO_H_ */

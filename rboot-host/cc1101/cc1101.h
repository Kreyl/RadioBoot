/*
 * cc1101.h
 *
 *  Created on: 19 θώνÿ 2018 γ.
 *      Author: RLeonov
 */

#ifndef CC1101_H_
#define CC1101_H_

#include "../rexos/userspace/io.h"
#include "../rexos/userspace/sys.h"
#include "cc1101_config.h"

extern const REX __CC1101;

#define CC1101_FLAGS_EMPTY                      (0 << 0)
#define CC1101_FLAGS_TRANSMIT_ACK               (1 << 0)
#define CC1101_FLAGS_NO_TIMEOUT                 (1 << 1)

typedef enum {
    CC1101_RESET = IPC_USER,
    CC1101_CALIBRATE,
    CC1101_SET_CHANNEL,
    CC1101_SET_POWER,
    CC1101_SET_PACKET_SIZE,
    CC1101_READ_FIFO,
} CC1101_IPCS;

#pragma pack(push, 1)
typedef struct {
    unsigned int size;
    unsigned int flags;
} CC1101_STACK;

typedef struct {
    int8_t RSSI;
    uint8_t LQI;
} CC1101_PKT_STATUS;
#pragma pack(pop)

// External
HANDLE cc1101_open();
void cc1101_set_packet_size(HANDLE process, unsigned int packet_size);
void cc1101_set_channel(HANDLE process, unsigned int channel);
void cc1101_set_power(HANDLE process, uint8_t CC_Pwr);
void cc1101_close(HANDLE process);
bool cc1101_transmit(HANDLE process, const uint8_t* data, unsigned int data_size);
void cc1101_transmit_with_ack(HANDLE process, IO* io);
int cc1101_receive(HANDLE process, uint8_t* data, unsigned int data_size, unsigned int flags, int* RSSI);

int8_t RSSI_dBm(uint8_t raw);



#endif /* CC1101_H_ */

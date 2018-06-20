/*
 * cc1101.c
 *
 *  Created on: 19 ���� 2018 �.
 *      Author: RLeonov
 */


/*
 * radio.c
 *
 *  Created on: 17 ��� 2017 �.
 *      Author: RLeonov
 */

#include <string.h>
#include "app_private.h"
#include "../rexos/userspace/stdio.h"
#include "cc1101_hw.h"
#include "cc1101.h"

void cc1101();

const REX __CC1101 = {
    //name
    "CC1101",
    //size
    CC1101_PROCESS_SIZE,
    //priority
    CC1101_PROCESS_PRIORITY,
    //flags
    PROCESS_FLAGS_ACTIVE | REX_FLAG_PERSISTENT_NAME,
    //function
    cc1101
};

HANDLE cc1101_open()
{
    HANDLE process = process_create(&__CC1101);
    if(process == INVALID_HANDLE)
        return INVALID_HANDLE;

    ack(process, HAL_REQ(HAL_CC1101, IPC_OPEN), 0, 0, 0);
    return process;
}

void cc1101_set_packet_size(HANDLE process, unsigned int packet_size)
{
    ipc_post_inline(process, HAL_REQ(HAL_CC1101, CC1101_SET_PACKET_SIZE), packet_size, 0, 0);
}

void cc1101_set_channel(HANDLE process, unsigned int channel)
{
    ipc_post_inline(process, HAL_REQ(HAL_CC1101, CC1101_SET_CHANNEL), channel, 0, 0);
}

void cc1101_set_power(HANDLE process, uint8_t CC_Pwr)
{
    ipc_post_inline(process, HAL_REQ(HAL_CC1101, CC1101_SET_POWER), CC_Pwr, 0, 0);
}

static bool cc1101_transmit_sync_internal(HANDLE process, IO* io, unsigned int size, unsigned int flags)
{
    CC1101_STACK* stack = io_push(io, sizeof(CC1101_STACK));
    stack->size = size;
    stack->flags = flags;
    return (size == io_read_sync(process, HAL_IO_REQ(HAL_CC1101, IPC_WRITE), 0, io, size));
}

static int cc1101_receive_internal(HANDLE process, IO* io, unsigned int size, unsigned int flags)
{
    CC1101_STACK* stack = io_push(io, sizeof(CC1101_STACK));
    stack->size = size;
    stack->flags = flags;
    int res = io_write_sync(process, HAL_IO_REQ(HAL_CC1101, IPC_READ), 0, io);
    return io_read_sync(process, HAL_IO_REQ(HAL_CC1101, CC1101_READ_FIFO), 0, io, res);
}

bool cc1101_transmit(HANDLE process, uint8_t* data, unsigned int data_size)
{
    IO* io = io_create(data_size + sizeof(CC1101_STACK));
    bool res = false;
    if(io == NULL)
    {
        error(ERROR_OUT_OF_MEMORY);
        return false;
    }
    io_data_append(io, data, data_size);
    res = cc1101_transmit_sync_internal(process, io, io->data_size, CC1101_FLAGS_EMPTY);
    io_destroy(io);
    return res;
}

//void cc1101_transmit_with_ack(HANDLE process, IO* io)
//{
//    cc1101_transmit_sync_internal(process, io, io->data_size, CC1101_FLAGS_TRANSMIT_ACK);
//}

int cc1101_receive(HANDLE process, uint8_t* data, unsigned int data_size, unsigned int flags, int* RSSI)
{
    IO* io = io_create(sizeof(CC1101_STACK) + data_size + sizeof(CC1101_PKT_STATUS));
    int res = false;
    if(io == NULL)
    {
        error(ERROR_OUT_OF_MEMORY);
        return get_last_error();
    }

    res = cc1101_receive_internal(process, io, data_size, flags);

    if(res < sizeof(CC1101_PKT_STATUS))
    {
        io_destroy(io);
        return res;
    }

    memcpy(data, io_data(io), data_size);
    *RSSI = RSSI_dBm(*(uint8_t*)(io_data(io) + data_size));

    io_destroy(io);
    return res - sizeof(CC1101_PKT_STATUS);
}

// Return RSSI in dBm
int8_t RSSI_dBm(uint8_t raw)
{
    int16_t RSSI = raw;
    if (RSSI >= 128) RSSI -= 256;
    RSSI = (RSSI / 2) - 74;    // now it is in dBm
    return RSSI;
}

// ========================== CC1101 PROCESS ====================================
static void cc1101_request(CC1101_HW* cc1101, IPC* ipc)
{
    switch (HAL_ITEM(ipc->cmd))
    {
        case IPC_OPEN:
            cc1101_hw_init(cc1101);
            break;
        case IPC_CLOSE:
            cc1101_hw_deinit(cc1101);
            break;
        case IPC_WRITE:
            cc1101_hw_tx(cc1101, ipc->process, (IO*)ipc->param2, ipc->param3);
            break;
        case IPC_READ:
            cc1101_hw_rx(cc1101, ipc->process, (IO*)ipc->param2, ipc->param3);
            break;
        case CC1101_RESET:
            cc1101_hw_reset(cc1101);
            break;
        case CC1101_CALIBRATE:
            cc1101_hw_calibrate(cc1101);
            break;
        case CC1101_SET_CHANNEL:
            cc1101_hw_set_channel(cc1101, ipc->param1);
            break;
        case CC1101_SET_POWER:
            cc1101_hw_set_tx_power(cc1101, ipc->param1);
            break;
        case CC1101_SET_PACKET_SIZE:
            cc1101_hw_set_radio_pkt_size(cc1101, ipc->param1);
            break;
//            cc1101_rx(cc1101, (IO*)ipc->param2);
        case CC1101_READ_FIFO:
            ipc->param3 = cc1101_hw_read_fifo(cc1101, (IO*)ipc->param2);
            break;
        default:
            error(ERROR_NOT_SUPPORTED);
            break;
    }
}


void cc1101()
{
    CC1101_HW cc1101;
    IPC ipc;

#if (CC1101_DEBUG)
    open_stdout();
#endif // CC1101_DEBUG

    for (;;)
    {
        ipc_read(&ipc);
        switch (HAL_GROUP(ipc.cmd))
        {
        case HAL_CC1101:
            cc1101_request(&cc1101, &ipc);
            break;
        default:
#if (CC1101_DEBUG)
            printf("CC1101: Unhandled ipc\n");
#endif // CC1101_DEBUG
            error(ERROR_NOT_SUPPORTED);
            break;
        }
        ipc_write(&ipc);
    }

}


//

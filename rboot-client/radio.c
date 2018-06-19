/*
 * radio.c
 *
 *  Created on: 17 мая 2017 г.
 *      Author: RLeonov
 */

#include <string.h>
#include "app_private.h"
#include "../rexos/userspace/stdio.h"
#include "cc11xx/cc1101.h"
#include "radio.h"

void radio();

const REX __RADIO = {
    //name
    "Radio",
    //size
    600,
    //priority
    201,
    //flags
    PROCESS_FLAGS_ACTIVE | REX_FLAG_PERSISTENT_NAME,
    //function
    radio
};

void radio_init(APP* app)
{
    app->radio = process_create(&__RADIO);
    if(app->radio == INVALID_HANDLE)
        return;

    ack(app->radio, HAL_REQ(HAL_RADIO, IPC_OPEN), 0, 0, 0);
    ack(app->radio, HAL_REQ(HAL_RADIO, RADIO_RESET), 0, 0, 0);
    ack(app->radio, HAL_REQ(HAL_RADIO, RADIO_SET_POWER), CC_PwrMinus10dBm, 0, 0);
    ack(app->radio, HAL_REQ(HAL_RADIO, RADIO_SET_CHANNEL), 0, 0, 0);
    ack(app->radio, HAL_REQ(HAL_RADIO, RADIO_SET_PACKET_SIZE), 5, 0, 0);

    led_mode(app, LED_COLOR_BLUE, LED_MODE_ON);
}

void radio_tx_sync(APP* app, uint8_t* data, unsigned int data_size)
{
    led_mode(app, LED_COLOR_BLUE, LED_MODE_BLINK);
    ack(app->radio, HAL_REQ(HAL_RADIO, RADIO_TX), (unsigned int)data, data_size, 0);
}

bool radio_rx_sync(APP* app, uint8_t* data, uint8_t data_size)
{
    IO* io = io_create(data_size);
    if(io == NULL)
    {
        error(ERROR_OUT_OF_MEMORY);
        return false;
    }

    if(data_size == io_read_sync(app->radio, HAL_IO_REQ(HAL_RADIO, RADIO_RX), 0, io, data_size))
        memcpy(data, io_data(io), io->data_size);

    io_destroy(io);
    return true;
}

// ========================== RADIO PROCESS ====================================
static void radio_request(CC1101* cc1101, IPC* ipc)
{
    switch (HAL_ITEM(ipc->cmd))
    {
        case IPC_OPEN:
            cc1101_hw_init(cc1101);
            break;
        case IPC_CLOSE:
            cc1101_hw_deinit(cc1101);
            break;
        case RADIO_RESET:
            cc1101_reset(cc1101);
            break;
        case RADIO_CALIBRATE:
            cc1101_calibrate(cc1101);
            break;
        case RADIO_SET_CHANNEL:
            cc1101_set_channel(cc1101, ipc->param1);
            break;
        case RADIO_SET_POWER:
            cc1101_set_tx_power(cc1101, ipc->param1);
            break;
        case RADIO_SET_PACKET_SIZE:
            cc1101_set_radio_pkt_size(cc1101, ipc->param1);
            break;
        case RADIO_TX:
            cc1101_tx(cc1101, (uint8_t*)ipc->param1, ipc->param2);
            break;
        case RADIO_RX:
            cc1101_rx(cc1101, (IO*)ipc->param2);
            break;
        case RADIO_GET_PACKET:
            cc1101_receive_packet(cc1101, (IO*)ipc->param2);
            break;
        default:
            error(ERROR_NOT_SUPPORTED);
            break;
    }
}


void radio()
{
    CC1101 cc1101;
    IPC ipc;

#if (APP_RADIO_DEBUG)
    open_stdout();
#endif // APP_RADIO_DEBUG

    for (;;)
    {
        ipc_read(&ipc);
        switch (HAL_GROUP(ipc.cmd))
        {
        case HAL_RADIO:
            radio_request(&cc1101, &ipc);
            break;
        default:
#if (APP_RADIO_DEBUG)
            printf("RADIO: Unhandled ipc\n");
#endif // APP_RADIO_DEBUG
            error(ERROR_NOT_SUPPORTED);
            break;
        }
        ipc_write(&ipc);
    }

}

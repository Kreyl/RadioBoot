/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2016, Alexey Kramarenko
    All rights reserved.
*/

#include <string.h>
#include "../rexos/userspace/stm32/stm32_driver.h"
#include "../rexos/userspace/stdio.h"
#include "../rexos/userspace/stdlib.h"
#include "../rexos/userspace/usb.h"
#include "../rexos/userspace/stream.h"
#include "../rexos/userspace/gpio.h"
#include "../rexos/midware/usbd/usbd.h"
#include "../rexos/drv/stm32/stm32_usb.h"
#include "comm.h"
#include "led.h"
#include "usb_desc.h"
#include "sys_config.h"
#include "stm32_config.h"
#include "app_private.h"
#include "config.h"


static inline void comm_reply_err(IO* io, const char* c)
{
    io_reset(io);
    io_data_append(io, c, strlen(c));
}

static inline void comm_dispatch_command(APP* app, IO* io)
{
    COMM_HEADER* header = (COMM_HEADER*)io_data(io);
//    uint8_t* data = (uint8_t*)(io_data(io) + sizeof(COMM_HEADER));
//    unsigned int data_size = io->data_size - sizeof(COMM_HEADER);

    if(header->start_byte != COMM_START_BYTE)
        return comm_reply_err(io, "SB ERROR\n");

    switch(header->id)
    {
        case COMM_CMD_ID_PING:
            return comm_reply_err(io, "PING OK\n");
        default:
            return comm_reply_err(io, "UNKNOWN ID\n");
    }
    // should never rich this point
}

static inline void comm_collect_data(APP* app, uint8_t byte)
{
    IO* io = app->comm.io;
    if((byte == COMM_CR_BYTE) || (byte == COMM_LF_BYTE))
    {
#if (APP_COMM_DEBUG)
        printf("COMM: dispatch\n");
        for(int i = 0; i < io->data_size; i++)
            printf("%02X ", ((uint8_t*)io_data(io))[i]);
        printf("\n");
#endif // APP_COMM_DEBUG

        if(io->data_size == 0)
            return;
        comm_dispatch_command(app, io);
        stream_write(app->comm.tx, (const char*)io_data(io), io->data_size);
        io_reset(app->comm.io);
        return;
    }

    if(app->comm.io->data_size < (sizeof(COMM_HEADER) + COMM_DATA_SIZE))
        io_data_append(app->comm.io, &byte, 1);
}

static inline void comm_usb_start(APP* app)
{
    HANDLE tx_stream;

#if (APP_COMM_DEBUG)
    printf("COMM: start\n");
#endif // APP_COMM_DEBUG

    tx_stream = get_handle(app->usbd, HAL_REQ(HAL_USBD_IFACE, IPC_GET_TX_STREAM), USBD_IFACE(0, 0), 0, 0);
    app->comm.rx_stream = get_handle(app->usbd, HAL_REQ(HAL_USBD_IFACE, IPC_GET_RX_STREAM), USBD_IFACE(0, 0), 0, 0);

    app->comm.tx = stream_open(tx_stream);
    app->comm.rx = stream_open(app->comm.rx_stream);
    app->comm.active = true;

    stream_listen(app->comm.rx_stream, 0, HAL_USBD);
    led_mode(app, LED_COLOR_WHITE, LED_MODE_ON);
}

static void comm_usb_stop(APP* app)
{
    if (app->comm.active)
    {
#if (APP_COMM_DEBUG)
        printf("COMM: stop\n");
#endif // APP_COMM_DEBUG

        stream_stop_listen(app->comm.rx_stream);
        stream_close(app->comm.tx);
        stream_close(app->comm.rx);
        app->comm.rx = app->comm.tx = app->comm.rx_stream = INVALID_HANDLE;
        app->comm.active = false;
        led_mode(app, LED_COLOR_WHITE, LED_MODE_OFF);
    }
}

static inline void comm_usbd_alert(APP* app, USBD_ALERTS alert)
{
    switch (alert)
    {
    case USBD_ALERT_CONFIGURED:
    case USBD_ALERT_RESUME:
        comm_usb_start(app);
        break;
    case USBD_ALERT_RESET:
    case USBD_ALERT_SUSPEND:
        comm_usb_stop(app);
        break;
    default:
        break;
    }
}

static inline void comm_usbd_stream_rx(APP* app, unsigned int size)
{
    char c;
    unsigned int i;
    for (i = 0; i < size; ++i)
    {
        stream_read(app->comm.rx, &c, sizeof(char));
        comm_collect_data(app, c);
    }
    stream_listen(app->comm.rx_stream, 0, HAL_USBD);
    led_mode(app, LED_COLOR_WHITE, LED_MODE_BLINK);

        // echo
//        stream_write(app->comm.tx, &c, sizeof(char));
}

void comm_init(APP *app)
{
    app->comm.io = io_create(sizeof(COMM_HEADER) + COMM_DATA_SIZE);
    if(app->comm.io == NULL)
    {
        error(ERROR_OUT_OF_MEMORY);
        return;
    }

    app->comm.rx = app->comm.tx = app->comm.rx_stream = INVALID_HANDLE;
    app->usbd = usbd_create(USB_PORT_NUM, USBD_PROCESS_SIZE, USBD_PROCESS_PRIORITY);

    ack(app->usbd, HAL_REQ(HAL_USBD, USBD_REGISTER_HANDLER), 0, 0, 0);

    usbd_register_const_descriptor(app->usbd, &__DEVICE_DESCRIPTOR, 0, 0);
    usbd_register_const_descriptor(app->usbd, &__CONFIGURATION_DESCRIPTOR, 0, 0);
    usbd_register_const_descriptor(app->usbd, &__STRING_WLANGS, 0, 0);
    usbd_register_const_descriptor(app->usbd, &__STRING_MANUFACTURER, 1, 0x0409);
    usbd_register_const_descriptor(app->usbd, &__STRING_PRODUCT, 2, 0x0409);
    usbd_register_const_descriptor(app->usbd, &__STRING_SERIAL, 3, 0x0409);
    usbd_register_const_descriptor(app->usbd, &__STRING_DEFAULT, 4, 0x0409);

    sleep_ms(9);
    ack(app->usbd, HAL_REQ(HAL_USBD, IPC_OPEN), USB_PORT_NUM, 0, 0);

#if (APP_COMM_DEBUG)
    printf("COMM: init\n");
#endif // APP_COMM_DEBUG
}

void comm_request(APP* app, IPC* ipc)
{
    switch (HAL_ITEM(ipc->cmd))
    {
    case USBD_ALERT:
        comm_usbd_alert(app, ipc->param1);
        break;
    case IPC_STREAM_WRITE:
        comm_usbd_stream_rx(app, ipc->param3);
        break;
    default:
        error(ERROR_NOT_SUPPORTED);
        break;
    }
}

void comm_interface_request(APP* app, IPC* ipc)
{
    led_mode(app, LED_COLOR_WHITE, LED_MODE_BLINK);
    switch(HAL_ITEM(ipc->cmd))
    {
        case USB_CDC_ACM_SET_BAUDRATE:
#if (APP_COMM_DEBUG)
            printf("COMM: set baud\n");
#endif // APP_COMM_DEBUG
            break;
        case USB_CDC_ACM_GET_BAUDRATE:
#if (APP_COMM_DEBUG)
            printf("COMM: get baud\n");
#endif // APP_COMM_DEBUG
            break;
        case USB_CDC_ACM_BAUDRATE_REQUEST:
#if (APP_COMM_DEBUG)
            printf("COMM: baud req\n");
#endif // APP_COMM_DEBUG
            break;
        case USB_CDC_ACM_SEND_BREAK:
#if (APP_COMM_DEBUG)
            printf("COMM: send break\n");
#endif // APP_COMM_DEBUG
            break;
        case USB_CDC_ACM_BREAK_REQUEST:
#if (APP_COMM_DEBUG)
            printf("COMM: break req\n");
#endif // APP_COMM_DEBUG
            break;
        default:
#if (APP_COMM_DEBUG)
            printf("COMM: unhandled ipc, ITEM %X\n", HAL_ITEM(ipc->cmd));
#endif // APP_COMM_DEBUG
            error(ERROR_NOT_SUPPORTED);
            break;

    }
}

/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2016, Alexey Kramarenko
    All rights reserved.
*/

#include "comm.h"
#include "../rexos/userspace/stdio.h"
#include "../rexos/userspace/stdlib.h"
#include "../rexos/userspace/usb.h"
#include "../rexos/userspace/stream.h"
#include "../rexos/userspace/gpio.h"
#include "../rexos/userspace/stm32/stm32_driver.h"
#include "../rexos/midware/usbd/usbd.h"
#include "../rexos/drv/stm32/stm32_usb.h"
#include "led.h"
#include "usb_desc.h"
#include "sys_config.h"
#include "stm32_config.h"
#include "app_private.h"
#include "config.h"

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
        // echo
        stream_write(app->comm.tx, &c, sizeof(char));
    }
    stream_listen(app->comm.rx_stream, 0, HAL_USBD);
}

void comm_init(APP *app)
{
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

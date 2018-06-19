/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2016, Alexey Kramarenko
    All rights reserved.
*/

#include "../rexos/userspace/stdio.h"
#include "../rexos/userspace/stdlib.h"
#include "../rexos/userspace/process.h"
#include "../rexos/userspace/sys.h"
#include "../rexos/userspace/stm32/stm32_driver.h"
#include "../rexos/userspace/ipc.h"
#include "../rexos/userspace/uart.h"
#include "../rexos/userspace/process.h"
#include "../rexos/userspace/power.h"
#include "../rexos/userspace/pin.h"
#include "../rexos/userspace/gpio.h"
#include "../rexos/userspace/irq.h"
#include "app_private.h"
#include "cc1101/cc1101.h"
#include "comm.h"
#include "led.h"
#include "config.h"

void app();

const REX __APP = {
    //name
    "App main",
    //size
    900,
    //priority
    200,
    //flags
    PROCESS_FLAGS_ACTIVE | REX_FLAG_PERSISTENT_NAME,
    //function
    app
};


static inline void app_setup_dbg()
{
    BAUD baudrate;
    pin_enable(DBG_CONSOLE_TX_PIN, STM32_GPIO_MODE_AF, AF7);
    uart_open(DBG_CONSOLE, UART_MODE_STREAM | UART_TX_STREAM);
    baudrate.baud = DBG_CONSOLE_BAUD;
    baudrate.data_bits = 8;
    baudrate.parity = 'N';
    baudrate.stop_bits= 1;
    uart_set_baudrate(DBG_CONSOLE, &baudrate);
    uart_setup_printk(DBG_CONSOLE);
    uart_setup_stdout(DBG_CONSOLE);
    open_stdout();
}

static inline void app_init(APP* app)
{
    process_create(&__STM32_CORE);
#if (APP_DEBUG)
    app_setup_dbg();
    printf("RadioBoot Host, CPU %d MHz\n", power_get_core_clock()/1000000);
#endif
}

const uint8_t packets[6][5] = {
        {0x00, 0x01, 0xFF, 0x00, 0x00},
        {0x00, 0x01, 0x00, 0x00, 0x00},
        {0x00, 0x01, 0x00, 0xFF, 0x00},
        {0x00, 0x01, 0x00, 0x00, 0x00},
        {0x00, 0x01, 0x00, 0x00, 0xFF},
        {0x00, 0x01, 0x00, 0x00, 0x00}
};

void app()
{
    APP app;
    IPC ipc;

    app_init(&app);
    led_init(&app);

    app.cc1101 = cc1101_open();

    printf("cc1101 handle: %X\n", app.cc1101);

//    comm_init(&app);

    uint8_t pkt_id = 0;
//    uint8_t data[64];

    uint32_t timeout = 1000;
    app.timer = timer_create(0, HAL_APP);
    timer_start_ms(app.timer, timeout);

    sleep_ms(200);
    process_info();

    IO* io = io_create(5);


    for (;;)
    {
        ipc_read(&ipc);
        switch (HAL_GROUP(ipc.cmd))
        {

        case HAL_APP:
            printf("TO\n");
            io_reset(io);
            io_data_append(io, (uint8_t*)packets[pkt_id], 5);

            if(!cc1101_transmit(app.cc1101, (uint8_t*)packets[pkt_id], 5))
                printf("TX failure\n");

            //printf("rx: %d\n", radio_rx_sync(&app, data));

            if(pkt_id++ >= 5)
                pkt_id = 0;

            timer_start_ms(app.timer, timeout);
        break;

        case HAL_USBD:
            comm_request(&app, &ipc);
            break;
        default:
            error(ERROR_NOT_SUPPORTED);
            break;
        }
        ipc_write(&ipc);
    }
}

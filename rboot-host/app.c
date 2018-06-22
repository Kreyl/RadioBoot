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
#include "cc1101/cc1101_defines.h"
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

//    uint8_t pkt_id = 0;
//    app.cc1101 = cc1101_open();
//    cc1101_set_channel(app.cc1101, 0);
//    cc1101_set_power(app.cc1101, CC_PwrMinus10dBm);
//    uint32_t timeout = 100;
//    app.timer = timer_create(0, HAL_APP);
//    timer_start_ms(app.timer, timeout);

    comm_init(&app);


    sleep_ms(20);
    process_info();

    for (;;)
    {
        ipc_read(&ipc);
        switch (HAL_GROUP(ipc.cmd))
        {

        case HAL_APP:
            if(HAL_ITEM(ipc.cmd) == IPC_TIMEOUT)
            {
                led_mode(&app, LED_COLOR_BLUE, LED_MODE_BLINK);
//                if(cc1101_transmit(app.cc1101, packets[pkt_id], 5))
//                {
//                    printf("TX ok\n");
//                    for(int i = 0; i < 5; i++)
//                        printf("%02X ", packets[pkt_id][i]);
//                    printf("\n");
//
//                }
//                else
//                {
//                    printf("TX failure\n");
//                }
//
//                if(pkt_id++ >= 5)
//                    pkt_id = 0;
//
//                //printf("rx: %d\n", radio_rx_sync(&app, data));
//                timer_start_ms(app.timer, timeout);
            }
        break;

        case HAL_USBD:
            comm_request(&app, &ipc);
            break;
        case HAL_USBD_IFACE:
            comm_interface_request(&app, &ipc);
            break;
        default:
            printf("APP: Unhandled IPC, HAL_GROUP %X, ITEM %X\n", HAL_GROUP(ipc.cmd), HAL_ITEM(ipc.cmd));
            error(ERROR_NOT_SUPPORTED);
            break;
        }
        ipc_write(&ipc);
    }
}

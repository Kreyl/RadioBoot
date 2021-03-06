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
#include "cc1101.h"
#include "cc1101_defines.h"
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
    printf("RadioBoot Client, CPU %d MHz\n", power_get_core_clock()/1000000);
#endif
}

void app()
{
    APP app;
    IPC ipc;

    app_init(&app);
    led_init(&app);

    app.cc1101 = cc1101_open();

    uint32_t timeout = 20;
    app.timer = timer_create(0, HAL_APP);
//    timer_start_ms(app.timer, timeout);

    sleep_ms(200);
    process_info();

    uint8_t data[100];
    int RSSI;
    cc1101_set_packet_size(app.cc1101, 5);
    cc1101_set_channel(app.cc1101, 0);
//    cc1101_set_power(app.cc1101, CC_PwrMinus10dBm);

    int res = 0;

    for (;;)
    {
        ipc_read(&ipc);
        switch (HAL_GROUP(ipc.cmd))
        {
            case HAL_APP:
                res = cc1101_receive(app.cc1101, data, 5, CC1101_FLAGS_NO_TIMEOUT, &RSSI);

                if(res > 0)
                {
                    printf("RX: ");
                    for(int i = 0; i < res; i++)
                        printf("%02X ", data[i]);
                    printf(", RSSI %d\n", RSSI);
                }
                else
                    printf("RX failure\n");

                timer_start_ms(app.timer, timeout);
            break;

            default:
                error(ERROR_NOT_SUPPORTED);
                break;
        }
        ipc_write(&ipc);
    }
}

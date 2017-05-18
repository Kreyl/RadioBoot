/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2017, Alexey Kramarenko
    All rights reserved.
*/

#ifndef LPC_CORE_PRIVATE_H
#define LPC_CORE_PRIVATE_H

#include "lpc_config.h"
#include "lpc_core.h"
#include "lpc_power.h"
#include "lpc_timer.h"

#include "lpc_uart.h"
#include "lpc_i2c.h"
#ifdef LPC11Uxx
#include "lpc_usb.h"
#else //LPC18xx
#include "lpc_otg.h"
#endif //LPC11Uxx
#include "lpc_eep.h"
#include "lpc_sdmmc.h"
#include "lpc_flash.h"


typedef struct _CORE {
    POWER_DRV power;
    TIMER_DRV timer;
#if (LPC_UART_DRIVER)
    UART_DRV uart;
#endif //LPC_UART_DRIVER
#if (LPC_I2C_DRIVER)
    I2C_DRV i2c;
#endif
#if (LPC_USB_DRIVER)
#ifdef LPC11Uxx
    USB_DRV usb;
#else //LPC18xx
    OTG_DRV otg;
#endif //LPC11Uxx
#endif //LPC_USB_DRIVER
#if (LPC_SDMMC_DRIVER)
    SDMMC_DRV sdmmc;
#endif
#if (LPC_FLASH_DRIVER)
    FLASH_DRV flash;
#endif //LPC_FLASH_DRIVER
}CORE;


#endif // LPC_CORE_PRIVATE_H

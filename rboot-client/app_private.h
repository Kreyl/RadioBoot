/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2016, Alexey Kramarenko
    All rights reserved.
*/

#ifndef APP_PRIVATE_H
#define APP_PRIVATE_H

#include "app.h"
#include "led.h"
#include "../rexos/userspace/ipc.h"
#include <stdint.h>

typedef enum {
    HAL_RADIO = HAL_APP
} HAL_APP_GROUPS;

typedef struct _APP {
    HANDLE timer;
    HANDLE radio;
    LEDS leds;
} APP;

#endif // APP_PRIVATE_H

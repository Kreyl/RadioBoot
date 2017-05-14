/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2016, Alexey Kramarenko
    All rights reserved.
*/

#ifndef APP_PRIVATE_H
#define APP_PRIVATE_H

#include "app.h"
#include "comm.h"
#include "led.h"
#include <stdint.h>

typedef struct _APP {
    HANDLE timer;
    HANDLE usbd;

    COMM comm;
    LEDS leds;
} APP;

#endif // APP_PRIVATE_H

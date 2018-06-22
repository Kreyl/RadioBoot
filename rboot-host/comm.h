/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2016, Alexey Kramarenko
    All rights reserved.
*/

#ifndef COMM_H
#define COMM_H

// USB communication process

#include "../rexos/userspace/types.h"
#include "../rexos/userspace/sys.h"
#include "../rexos/userspace/usb.h"
#include "../rexos/userspace/io.h"
#include "app.h"

#define COMM_START_BYTE             '#'
#define COMM_CR_BYTE                0x0D
#define COMM_LF_BYTE                0x0A

#define COMM_DATA_SIZE              40

typedef enum {
    COMM_CMD_ID_PING = 0x30,
    COMM_CMD_ID_MAX
} COMM_CMD_ID;

#pragma pack(push, 1)
typedef struct
{
    uint8_t start_byte;
    uint8_t id;
    uint8_t length;
} COMM_HEADER;
#pragma pack(pop)


typedef struct {
    HANDLE tx, rx, rx_stream;
    bool active;
    IO* io;
} COMM;

void comm_connect(APP* app);
void comm_disconnect(APP* app);
void comm_init(APP* app);
void comm_request(APP* app, IPC* ipc);
void comm_interface_request(APP* app, IPC* ipc);

#endif // COMM_H

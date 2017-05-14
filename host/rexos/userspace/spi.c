/*
 * spi.c
 *
 *  Created on: 1 апр. 2017 г.
 *      Author: RomaJam
 */


#include "spi.h"
#include "object.h"
#include "sys_config.h"
#include "core/core.h"

bool spi_open(int port, unsigned int settings)
{
    return get_handle(object_get(SYS_OBJ_CORE), HAL_REQ(HAL_SPI, IPC_OPEN), port, settings, 0) != INVALID_HANDLE;
}

void spi_close(int port)
{
    ack(object_get(SYS_OBJ_CORE), HAL_REQ(HAL_SPI, IPC_CLOSE), port, 0, 0);
}

int spi_data(int port, IO* io, unsigned int max_size)
{
    return io_read_sync(object_get(SYS_OBJ_CORE), HAL_IO_REQ(HAL_SPI, IPC_READ), port, io, max_size);
}

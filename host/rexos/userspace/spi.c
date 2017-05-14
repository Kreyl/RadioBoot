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

unsigned int spi_byte(int port, uint8_t byte)
{
    return get(object_get(SYS_OBJ_CORE), HAL_REQ(HAL_SPI, SPI_BYTE), port, byte, 0);
}

unsigned int spi_send_data(int port, uint8_t* data, unsigned int data_size)
{
    return io_read_sync(object_get(SYS_OBJ_CORE), HAL_REQ(HAL_SPI, SPI_SEND_DATA), port, data, data_size);
}

unsigned int spi_get_data(int port, uint8_t* data)
{
    return io_read_sync(object_get(SYS_OBJ_CORE), HAL_REQ(HAL_SPI, SPI_GET_DATA), port, data, 0);
}

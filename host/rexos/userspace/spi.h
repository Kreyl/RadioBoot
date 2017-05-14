/*
 * spi.h
 *
 *  Created on: 1 апр. 2017 г.
 *      Author: RomaJam
 */

#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include "io.h"

typedef enum {
    SPI_MODE_MASTER,
    SPI_MODE_SLAVE
} MODE;

#define SPI_DATA_FIRST_EDGE         (0 << 0)
#define SPI_DATA_SECOND_EDGE        (1 << 0)

#define SPI_DATA_CK_IDLE_LOW        (0 << 1)
#define SPI_DATA_CK_IDLE_HIGH       (1 << 1)

#define SPI_MODE_SLAVE              (0 << 2)
#define SPI_MODE_MASTER             (1 << 2)

#define SPI_BAUDRATE_DIV2           (0 << 3)
#define SPI_BAUDRATE_DIV4           (1 << 3)
#define SPI_BAUDRATE_DIV8           (2 << 3)
#define SPI_BAUDRATE_DIV16          (3 << 3)
#define SPI_BAUDRATE_DIV32          (4 << 3)
#define SPI_BAUDRATE_DIV64          (5 << 3)
#define SPI_BAUDRATE_DIV128         (6 << 3)
#define SPI_BAUDRATE_DIV256         (7 << 3)

#define SPI_DATA_BO_MSB             (0 << 7)
#define SPI_DATA_BO_LSB             (1 << 7)

#define SPI_SSI_OFF                 (0 << 8)
#define SPI_SSI_ON                  (1 << 8)

#define SPI_NSS_HARDWARE            (0 << 9)
#define SPI_NSS_SOFTWARE            (1 << 9)

#define SPI_MODE_FULLDUPLEX         (0 << 10)
#define SPI_MODE_RECIEVE            (1 << 10)


bool spi_open(int port, unsigned int settings);
void spi_close(int port);

int spi_data(int port, IO* io, unsigned int max_size);

#endif /* SPI_H */

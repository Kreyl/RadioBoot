/*
 * cc1101.c
 *
 *  Created on: 2 апр. 2017 г.
 *      Author: RomaJam
 */

#include <string.h>
#include "../rexos/userspace/stdio.h"
#include "../rexos/userspace/stdlib.h"
#include "../rexos/userspace/process.h"
#include "../rexos/userspace/spi.h"
#include "../rexos/userspace/pin.h"
#include "../rexos/userspace/gpio.h"
#include "../rexos/userspace/systime.h"
#include "app_private.h"
#include "cc11xx/cc1101.h"

static inline void cc1101_cs_lo()
{
    gpio_reset_pin(CC1101_CS_PIN);
}

static inline void cc1101_cs_hi()
{
    gpio_set_pin(CC1101_CS_PIN);
}

static inline bool cc1101_busy()
{
    return gpio_get_pin(CC1101_MISO_PIN);
}

static inline bool cc1101_write_register(CC1101* cc1101, uint8_t addr, uint8_t data)
{
    cc1101_cs_lo();
    while(cc1101_busy());
    spi_byte(CC1101_SPI, addr);
    spi_byte(CC1101_SPI, data);
    cc1101_cs_hi();
    return true;
}

static inline uint8_t cc1101_read_register(CC1101* cc1101, uint8_t addr)
{
    uint8_t res = 0;
    cc1101_cs_lo();
    while(cc1101_busy());
    spi_byte(CC1101_SPI, addr | CC_READ_FLAG);
    res = spi_byte(CC1101_SPI, 0);
    cc1101_cs_hi();
    return res;
}

static inline void cc1101_write_strobe(CC1101* cc1101, uint8_t strobe)
{
    cc1101_cs_lo();
    while(cc1101_busy());
    cc1101->state = (uint8_t)spi_byte(CC1101_SPI, strobe);
    cc1101->state &= 0b01110000; // TODO: ?
    cc1101_cs_hi();
}

//static inline bool cc1101_prepare_tx(CC1101* cc1101, uint8_t* data, unsigned int data_size)
//{
//    CC1101_PKT* pkt = (CC1101_PKT*)io_data(cc1101->io);
//    if(data_size + sizeof(uint8_t) > CC1101_IO_SIZE)
//    {
//        error(ERROR_OVERFLOW);
//        return false;
//    }
//
//    pkt->status = CC_FIFO | CC_WRITE_FLAG | CC_BURST_FLAG;
//    memcpy(&pkt->data, data, data_size);
//
//    cc1101->io->data_size = data_size + sizeof(uint8_t);
//    cc1101_cs_lo();
//    while(cc1101_busy());
//
//    if(data_size + sizeof(uint8_t) != spi_data(CC1101_SPI, cc1101->io, CC1101_IO_SIZE))
//    {
//#if (CC1101_DEBUG_ERRORS)
//        printf("CC1101: prepare tx failure\n");
//#endif // CC1101_DEBUG_ERRORS
//        cc1101_cs_hi();
//        return false;
//    }
//    cc1101_cs_hi();
//    return true;
//}

static inline void cc1101_chip_reset(CC1101* cc1101)
{
    cc1101_write_strobe(cc1101, CC_SRES);
}

//static inline void cc1101_flush_tx_fifo(CC1101* cc1101)
//{
//    cc1101_write_strobe(cc1101, CC_SFTX);
//}

static inline void cc1101_flush_rx_fifo(CC1101* cc1101)
{
    cc1101_write_strobe(cc1101, CC_SFRX);
}

//static inline bool cc1101_start_tx(CC1101* cc1101)
//{
//    return cc1101_write_strobe(cc1101, CC_STX);
//}
//
//static inline bool cc1101_start_rx(CC1101* cc1101)
//{
//    return cc1101_write_strobe(cc1101, CC_SRX);
//}
//
//

//
//static inline bool cc1101_get_state(CC1101* cc1101)
//{
//    return cc1101_write_strobe(cc1101, CC_SNOP);
//}
//
static inline void cc1101_rf_config(CC1101* cc1101)
{
    cc1101_write_register(cc1101, CC_FSCTRL1,  CC_FSCTRL1_VALUE);    // Frequency synthesizer control.
    cc1101_write_register(cc1101, CC_FSCTRL0,  CC_FSCTRL0_VALUE);    // Frequency synthesizer control.
    cc1101_write_register(cc1101, CC_FREQ2,    CC_FREQ2_VALUE);      // Frequency control word, high byte.
    cc1101_write_register(cc1101, CC_FREQ1,    CC_FREQ1_VALUE);      // Frequency control word, middle byte.
    cc1101_write_register(cc1101, CC_FREQ0,    CC_FREQ0_VALUE);      // Frequency control word, low byte.
    cc1101_write_register(cc1101, CC_MDMCFG4,  CC_MDMCFG4_VALUE);    // Modem configuration.
    cc1101_write_register(cc1101, CC_MDMCFG3,  CC_MDMCFG3_VALUE);    // Modem configuration.
    cc1101_write_register(cc1101, CC_MDMCFG2,  CC_MDMCFG2_VALUE);    // Modem configuration.
    cc1101_write_register(cc1101, CC_MDMCFG1,  CC_MDMCFG1_VALUE);    // Modem configuration.
    cc1101_write_register(cc1101, CC_MDMCFG0,  CC_MDMCFG0_VALUE);    // Modem configuration.
    cc1101_write_register(cc1101, CC_CHANNR,   CC_CHANNR_VALUE);     // Channel number.
    cc1101_write_register(cc1101, CC_DEVIATN,  CC_DEVIATN_VALUE);    // Modem deviation setting (when FSK modulation is enabled).
    cc1101_write_register(cc1101, CC_FREND1,   CC_FREND1_VALUE);     // Front end RX configuration.
    cc1101_write_register(cc1101, CC_FREND0,   CC_FREND0_VALUE);     // Front end RX configuration.
    cc1101_write_register(cc1101, CC_MCSM0,    CC_MCSM0_VALUE);      // Main Radio Control State Machine configuration.
    cc1101_write_register(cc1101, CC_FOCCFG,   CC_FOCCFG_VALUE);     // Frequency Offset Compensation Configuration.
    cc1101_write_register(cc1101, CC_BSCFG,    CC_BSCFG_VALUE);      // Bit synchronization Configuration.
    cc1101_write_register(cc1101, CC_AGCCTRL2, CC_AGCCTRL2_VALUE);   // AGC control.
    cc1101_write_register(cc1101, CC_AGCCTRL1, CC_AGCCTRL1_VALUE);   // AGC control.
    cc1101_write_register(cc1101, CC_AGCCTRL0, CC_AGCCTRL0_VALUE);   // AGC control.
    cc1101_write_register(cc1101, CC_FSCAL3,   CC_FSCAL3_VALUE);     // Frequency synthesizer calibration.
    cc1101_write_register(cc1101, CC_FSCAL2,   CC_FSCAL2_VALUE);     // Frequency synthesizer calibration.
    cc1101_write_register(cc1101, CC_FSCAL1,   CC_FSCAL1_VALUE);     // Frequency synthesizer calibration.
    cc1101_write_register(cc1101, CC_FSCAL0,   CC_FSCAL0_VALUE);     // Frequency synthesizer calibration.
    cc1101_write_register(cc1101, CC_TEST2,    CC_TEST2_VALUE);      // Various test settings.
    cc1101_write_register(cc1101, CC_TEST1,    CC_TEST1_VALUE);      // Various test settings.
    cc1101_write_register(cc1101, CC_TEST0,    CC_TEST0_VALUE);      // Various test settings.
    cc1101_write_register(cc1101, CC_FIFOTHR,  CC_FIFOTHR_VALUE);    // fifo threshold
//    cc1101_write_register(cc1101, CC_IOCFG2,   CC_IOCFG2_VALUE);     // GDO2 output pin configuration.
    cc1101_write_register(cc1101, CC_IOCFG2,   0x35);
    cc1101_write_register(cc1101, CC_IOCFG0,   CC_IOCFG0_VALUE);     // GDO0 output pin configuration.
//    cc1101_write_register(cc1101, CC_IOCFG0,   0x35);                // GDO0 is 27MHz/192 = 4.5MHz clock XOSC
    cc1101_write_register(cc1101, CC_PKTCTRL1, CC_PKTCTRL1_VALUE);   // Packet automation control.
    cc1101_write_register(cc1101, CC_PKTCTRL0, CC_PKTCTRL0_VALUE);   // Packet automation control.
//    cc1101_write_register(cc1101, CC_PKTLEN,   7);                   // Packet length, dummy

    cc1101_write_register(cc1101, CC_PATABLE, CC_Pwr0dBm);

    cc1101_write_register(cc1101, CC_MCSM2, CC_MCSM2_VALUE);
    cc1101_write_register(cc1101, CC_MCSM1, CC_MCSM1_VALUE);
}

#if (CC1101_DEBUG_INFO)
void cc1101_info(CC1101* cc1101)
{
    printf("CC1101: info\n");
    printf("version: %X\n", cc1101_read_register(cc1101, CC_VERSION));
    printf("RSSI: %X\n", cc1101_read_register(cc1101, CC_RSSI));
}
#endif // CC1101_DEBUG_INFO

void cc1101_hw_init(CC1101* cc1101)
{
    cc1101->active = false;

    gpio_enable_pin(CC1101_CS_PIN, GPIO_MODE_OUT);
    pin_enable(CC1101_CLK_PIN, STM32_GPIO_MODE_AF, AF5);
    pin_enable(CC1101_MISO_PIN, STM32_GPIO_MODE_AF, AF5);
    pin_enable(CC1101_MOSI_PIN, STM32_GPIO_MODE_AF, AF5);
    pin_enable(CC1101_GDO0_PIN, STM32_GPIO_MODE_INPUT, false);
    pin_enable(CC1101_GDO2_PIN, STM32_GPIO_MODE_INPUT, false);

    cc1101_cs_hi();

    if(!spi_open(CC1101_SPI, SPI_MODE_MASTER | SPI_DATA_BO_MSB | SPI_NSS_SOFTWARE | SPI_SSI_ON | SPI_DATA_CK_IDLE_LOW | SPI_DATA_FIRST_EDGE | SPI_BAUDRATE_DIV256))
    {
#if (CC1101_DEBUG_ERRORS)
    printf("CC1101: spi open failure\n");
#endif // CC1101_DEBUG_ERRORS
        return;
    }

//    cc1101_chip_reset(cc1101);
//    cc1101_flush_rx_fifo(cc1101);
//    cc1101_rf_config(cc1101);

#if (CC1101_DEBUG_INFO)
    cc1101_info(cc1101);
#endif // CC1101_DEBUG_INFO

    cc1101->active = true;

#if (CC1101_DEBUG_INFO)
    printf("CC1101: init\n");
#endif // CC1101_DEBUG_INFO
}


void cc1101_hw_deinit(CC1101* cc1101)
{
#if (CC1101_DEBUG_INFO)
    printf("CC1101: deinit\n");
#endif // CC1101_DEBUG_INFO
    cc1101->active = false;

    spi_close(CC1101_SPI);

    pin_disable(CC1101_CS_PIN);
    pin_disable(CC1101_CLK_PIN);
    pin_disable(CC1101_MISO_PIN);
    pin_disable(CC1101_MOSI_PIN);
    pin_disable(CC1101_GDO0_PIN);
    pin_disable(CC1101_GDO2_PIN);
}


void cc1101_set_channel(CC1101* cc1101, uint8_t channel_num)
{
#if (CC1101_DEBUG_INFO)
    printf("CC1101: set channel\n");
#endif // CC1101_DEBUG_INFO
}

void cc1101_set_tx_power(CC1101* cc1101, uint8_t power)
{
#if (CC1101_DEBUG_INFO)
    printf("CC1101: set tx power\n");
#endif // CC1101_DEBUG_INFO
    cc1101_write_register(cc1101, CC_PATABLE, power);
}

void cc1101_set_radio_pkt_size(CC1101* cc1101, uint8_t size)
{
#if (CC1101_DEBUG_INFO)
    printf("CC1101: set pkt size\n");
#endif // CC1101_DEBUG_INFO
}

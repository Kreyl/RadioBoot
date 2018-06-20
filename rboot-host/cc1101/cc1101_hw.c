/*
 * cc1101_hw.c
 *
 *  Created on: 19 θώνÿ 2018 γ.
 *      Author: RLeonov
 */


#include <string.h>
#include "../rexos/userspace/stdio.h"
#include "../rexos/userspace/stdlib.h"
#include "../rexos/userspace/process.h"
#include "../rexos/userspace/spi.h"
#include "../rexos/userspace/irq.h"
#include "../rexos/userspace/pin.h"
#include "../rexos/userspace/gpio.h"
#include "../rexos/userspace/systime.h"
#include "app_private.h"
#include "cc1101_hw.h"
#include "cc1101.h"

#if (CC1101_DEBUG)
void cc1101_dump(const uint8_t* buf, unsigned int size, const char* header)
{
    int i;
    printf("%s: ", header);
    for (i = 0; i < size; ++i)
        printf("%02X ", buf[i]);
    printf("\n");
}
#endif // CC1101_DEBUG

static inline bool cc1101_busy()
{
    return gpio_get_pin(CC1101_MISO_PIN);
}

// Return RSSI in dBm
static inline int8_t RSSI_dBm(uint8_t raw)
{
    int16_t RSSI = raw;
    if (RSSI >= 128) RSSI -= 256;
    RSSI = (RSSI / 2) - 74;    // now it is in dBm
    return RSSI;
}

static inline bool cc1101_write_register(uint8_t addr, uint8_t data)
{
    gpio_reset_pin(CC1101_CS_PIN);
    while(cc1101_busy());
    spi_byte(CC1101_SPI, addr);
    spi_byte(CC1101_SPI, data);
    sleep_ms(1);
    gpio_set_pin(CC1101_CS_PIN);
    return true;
}

static inline uint8_t cc1101_read_register(uint8_t addr)
{
    uint8_t res = 0;
    gpio_reset_pin(CC1101_CS_PIN);
    sleep_ms(2);
    while(cc1101_busy());
    spi_byte(CC1101_SPI, addr | CC_READ_FLAG);
    res = spi_byte(CC1101_SPI, 0);
    sleep_ms(2);
    gpio_set_pin(CC1101_CS_PIN);
    return res;
}

static inline void cc1101_write_strobe(CC1101_HW* cc1101, uint8_t strobe)
{
    gpio_reset_pin(CC1101_CS_PIN);
    while(cc1101_busy());
    cc1101->status = (uint8_t)spi_byte(CC1101_SPI, strobe);
    cc1101->status &= 0b01110000; // Mask needed bits
    sleep_ms(2);
    gpio_set_pin(CC1101_CS_PIN);

#if (CC1101_DEBUG_FLOW)
    printf("CC1101 status: %X\n", cc1101->status);
#endif // CC1101_DEBUG_FLOW
}

static inline void cc1101_prepare_tx(CC1101_HW* cc1101, uint8_t* data, unsigned int data_size)
{

#if (CC1101_DEBUG_FLOW)
    cc1101_dump(data, data_size, "TX PACKET");
    sleep_ms(20);
#endif // CC1101_DEBUG_FLOW

    gpio_reset_pin(CC1101_CS_PIN);
    while(cc1101_busy());
    spi_byte(CC1101_SPI, CC_FIFO | CC_WRITE_FLAG | CC_BURST_FLAG);

    for(uint8_t i = 0; i < data_size; i++)
        spi_byte(CC1101_SPI, data[i]);

    gpio_set_pin(CC1101_CS_PIN);
}

static inline void cc1101_chip_reset(CC1101_HW* cc1101)
{
#if (CC1101_DEBUG)
    printf("CC1101: reset\n");
#endif // CC1101_DEBUG_INFO
    cc1101_write_strobe(cc1101, CC_SRES);
}

static inline void cc1101_flush_tx_fifo(CC1101_HW* cc1101)
{
    cc1101_write_strobe(cc1101, CC_SFTX);
}

static inline void cc1101_flush_rx_fifo(CC1101_HW* cc1101)
{
#if (CC1101_DEBUG)
    printf("CC1101: flush RX\n");
#endif // CC1101_DEBUG_INFO
    cc1101_write_strobe(cc1101, CC_SFRX);
}

static inline void cc1101_start_tx(CC1101_HW* cc1101)
{
    cc1101_write_strobe(cc1101, CC_STX);
}

static inline void cc1101_start_rx(CC1101_HW* cc1101)
{
    cc1101_write_strobe(cc1101, CC_SRX);
}

//static inline void cc1101_get_state(CC1101* cc1101)
//{
//    cc1101_write_strobe(cc1101, CC_SNOP);
//}

static inline void cc1101_go_idle(CC1101_HW* cc1101)
{
    while(cc1101->status != CC_STB_IDLE)
    {
        cc1101_write_strobe(cc1101, CC_SIDLE);
        sleep_ms(1000);
#if (CC1101_DEBUG_ERRORS)
        printf("CC1101: status: %X\n", cc1101->status);
        if(cc1101->status & CC_STATUS_CARIER_SENSE)
            printf("Carrier sense\n");
        if(cc1101->status & CC_STATUS_PQT_REACHED)
            printf("PQT reached\n");
        if(cc1101->status & CC_STATUS_CHANNEL_CLEAR)
            printf("Channel clear\n");
        if(cc1101->status & CC_STATUS_SFD)
            printf("Sync word has been reveived\n");

        //Note: the reading gives the non-inverted value irrespective of what IOCFG0.GDO0_INV is programmed to.
        if(cc1101->status & CC_STATUS_GDO2)
            printf("GDO2 low value\n");
        if(cc1101->status & CC_STATUS_GDO0)
            printf("GDO0 low value\n");
#endif // CC1101_DEBUG_ERRORS
    }

    cc1101->state = CC1101_STATE_IDLE;
}

static inline void cc1101_go_sleep(CC1101_HW* cc1101)
{
    while(cc1101->state != CC_STB_IDLE)
    {
        cc1101_write_strobe(cc1101, CC_SIDLE);
    }

    cc1101->state = CC1101_STATE_IDLE;
}


static inline void cc1101_rf_config(CC1101_HW* cc1101)
{
#if (CC1101_DEBUG)
    printf("CC1101: RF config\n");
#endif // CC1101_DEBUG_INFO

    cc1101_write_register(CC_FSCTRL1,  CC_FSCTRL1_VALUE);    // Frequency synthesizer control.
    cc1101_write_register(CC_FSCTRL0,  CC_FSCTRL0_VALUE);    // Frequency synthesizer control.
    cc1101_write_register(CC_FREQ2,    CC_FREQ2_VALUE);      // Frequency control word, high byte.
    cc1101_write_register(CC_FREQ1,    CC_FREQ1_VALUE);      // Frequency control word, middle byte.
    cc1101_write_register(CC_FREQ0,    CC_FREQ0_VALUE);      // Frequency control word, low byte.
    cc1101_write_register(CC_MDMCFG4,  CC_MDMCFG4_VALUE);    // Modem configuration.
    cc1101_write_register(CC_MDMCFG3,  CC_MDMCFG3_VALUE);    // Modem configuration.
    cc1101_write_register(CC_MDMCFG2,  CC_MDMCFG2_VALUE);    // Modem configuration.
    cc1101_write_register(CC_MDMCFG1,  CC_MDMCFG1_VALUE);    // Modem configuration.
    cc1101_write_register(CC_MDMCFG0,  CC_MDMCFG0_VALUE);    // Modem configuration.
    cc1101_write_register(CC_CHANNR,   CC_CHANNR_VALUE);     // Channel number.
    cc1101_write_register(CC_DEVIATN,  CC_DEVIATN_VALUE);    // Modem deviation setting (when FSK modulation is enabled).
    cc1101_write_register(CC_FREND1,   CC_FREND1_VALUE);     // Front end RX configuration.
    cc1101_write_register(CC_FREND0,   CC_FREND0_VALUE);     // Front end RX configuration.
    cc1101_write_register(CC_MCSM0,    CC_MCSM0_VALUE);      // Main Radio Control State Machine configuration.
    cc1101_write_register(CC_FOCCFG,   CC_FOCCFG_VALUE);     // Frequency Offset Compensation Configuration.
    cc1101_write_register(CC_BSCFG,    CC_BSCFG_VALUE);      // Bit synchronization Configuration.
    cc1101_write_register(CC_AGCCTRL2, CC_AGCCTRL2_VALUE);   // AGC control.
    cc1101_write_register(CC_AGCCTRL1, CC_AGCCTRL1_VALUE);   // AGC control.
    cc1101_write_register(CC_AGCCTRL0, CC_AGCCTRL0_VALUE);   // AGC control.
    cc1101_write_register(CC_FSCAL3,   CC_FSCAL3_VALUE);     // Frequency synthesizer calibration.
    cc1101_write_register(CC_FSCAL2,   CC_FSCAL2_VALUE);     // Frequency synthesizer calibration.
    cc1101_write_register(CC_FSCAL1,   CC_FSCAL1_VALUE);     // Frequency synthesizer calibration.
    cc1101_write_register(CC_FSCAL0,   CC_FSCAL0_VALUE);     // Frequency synthesizer calibration.
    cc1101_write_register(CC_TEST2,    CC_TEST2_VALUE);      // Various test settings.
    cc1101_write_register(CC_TEST1,    CC_TEST1_VALUE);      // Various test settings.
    cc1101_write_register(CC_TEST0,    CC_TEST0_VALUE);      // Various test settings.
    cc1101_write_register(CC_FIFOTHR,  CC_FIFOTHR_VALUE);    // fifo threshold
    cc1101_write_register(CC_IOCFG2,   CC_IOCFG2_VALUE);     // GDO2 output pin configuration.
    cc1101_write_register(CC_IOCFG0,   CC_IOCFG0_VALUE);     // GDO0 output pin configuration.

    cc1101_write_register(CC_PKTCTRL1, CC_PKTCTRL1_VALUE);   // Packet automation control.
    cc1101_write_register(CC_PKTCTRL0, CC_PKTCTRL0_VALUE);   // Packet automation control.

    cc1101_write_register(CC_PATABLE, CC_Pwr0dBm);

    cc1101_write_register(CC_MCSM2, CC_MCSM2_VALUE);
    cc1101_write_register(CC_MCSM1, CC_MCSM1_VALUE);
}

#if (CC1101_DEBUG_INFO)
void cc1101_info(CC1101_HW* cc1101)
{
    printf("CC1101:\n");
    printf("Version %X\n", cc1101_read_register(CC_VERSION));
    printf("Patable %X\n", cc1101_read_register(CC_PATABLE));
}
#endif // CC1101_DEBUG_INFO

static inline void cc1101_gdo0_irq(int vector, void* param)
{
    CC1101_HW* cc1101 = (CC1101_HW*)param;
    EXTI->PR = 1ul << GPIO_PIN(CC1101_GDO0_PIN);
    NVIC_DisableIRQ(CC1101_GDO0_EXTI_IRQ);

//    iprintd("CC1101: exti irq, state: %d\n", cc1101->state);
    switch(cc1101->state)
    {
        case CC1101_STATE_RX:
            /* go to process for read FIFO */
//            ipc_ipost_inline(cc1101->user, HAL_CMD(HAL_RADIO, RADIO_GET_PACKET), (unsigned int)cc1101->io, 0, 0);
            cc1101->state = CC1101_STATE_IDLE;
            cc1101->process = INVALID_HANDLE;
            cc1101->io = NULL;
            break;

        case CC1101_STATE_TX:
            iio_complete(cc1101->process, HAL_IO_CMD(HAL_CC1101, IPC_WRITE), 0, cc1101->io);
            cc1101->state = CC1101_STATE_IDLE;
            cc1101->process = INVALID_HANDLE;
            cc1101->io = NULL;
            break;

        case CC1101_STATE_TX_ACK:
            cc1101->state = CC1101_STATE_IDLE;
            break;

        case CC1101_STATE_OFF:
        case CC1101_STATE_IDLE:
        case CC1101_STATE_SLEEP:
            break;
    }
}

static inline void cc1101_gdo2_irq(int vector, void* param)
{
    CC1101_HW* cc1101 = (CC1101_HW*)param;
    EXTI->PR = 1ul << GPIO_PIN(CC1101_GDO2_PIN);
    iprintd("CC1101: exti irq gdo2, state: %d\n", cc1101->state);
    NVIC_DisableIRQ(CC1101_GDO2_EXTI_IRQ);
}

void cc1101_hw_init(CC1101_HW* cc1101)
{
    cc1101->state = CC1101_STATE_OFF;

    pin_enable(CC1101_CLK_PIN, STM32_GPIO_MODE_AF, AF5);
    pin_enable(CC1101_MISO_PIN, STM32_GPIO_MODE_AF, AF5);
    pin_enable(CC1101_MOSI_PIN, STM32_GPIO_MODE_AF, AF5);
    gpio_enable_pin(CC1101_CS_PIN, GPIO_MODE_OUT);
    gpio_enable_pin(CC1101_GDO0_PIN, GPIO_MODE_IN_FLOAT);
    gpio_enable_pin(CC1101_GDO2_PIN, GPIO_MODE_IN_FLOAT);


    pin_enable_exti(CC1101_GDO0_PIN, EXTI_FLAGS_FALLING);
    pin_enable_exti(CC1101_GDO2_PIN, EXTI_FLAGS_FALLING);
    irq_register(CC1101_GDO0_EXTI_IRQ, cc1101_gdo0_irq, (void*)cc1101);
    irq_register(CC1101_GDO2_EXTI_IRQ, cc1101_gdo2_irq, (void*)cc1101);

    NVIC_SetPriority(CC1101_GDO0_EXTI_IRQ, 14);
    NVIC_SetPriority(CC1101_GDO2_EXTI_IRQ, 15);

    gpio_set_pin(CC1101_CS_PIN);

    if(!spi_open(CC1101_SPI, SPI_MODE_MASTER | SPI_DATA_BO_MSB | SPI_NSS_SOFTWARE | SPI_SSI_ON | SPI_DATA_CK_IDLE_LOW | SPI_DATA_FIRST_EDGE | SPI_BAUDRATE_DIV256))
    {
#if (CC1101_DEBUG_ERRORS)
        printf("CC1101: spi open failure\n");
#endif // CC1101_DEBUG_ERRORS
        return;
    }

    cc1101->process = INVALID_HANDLE;
    cc1101->state = CC1101_STATE_IDLE;

#if (CC1101_DEBUG)
    printf("CC1101: init\n");
#endif // CC1101_DEBUG_INFO
}

void cc1101_hw_deinit(CC1101_HW* cc1101)
{
#if (CC1101_DEBUG)
    printf("CC1101: deinit\n");
#endif // CC1101_DEBUG_INFO
    cc1101->state = CC1101_STATE_OFF;
    spi_close(CC1101_SPI);
    pin_disable(CC1101_CS_PIN);
    pin_disable(CC1101_CLK_PIN);
    pin_disable(CC1101_MISO_PIN);
    pin_disable(CC1101_MOSI_PIN);
    pin_disable(CC1101_GDO0_PIN);
    pin_disable(CC1101_GDO2_PIN);
}

void cc1101_hw_reset(CC1101_HW* cc1101)
{
    cc1101_chip_reset(cc1101);
    cc1101_flush_rx_fifo(cc1101);
    cc1101_rf_config(cc1101);

    sleep_ms(21);
#if (CC1101_DEBUG_INFO)
    cc1101_info(cc1101);
#endif // CC1101_DEBUG_INFO
}

void cc1101_hw_calibrate(CC1101_HW* cc1101)
{
#if (CC1101_DEBUG)
    printf("CC1101: calibrate\n");
#endif // CC1101_DEBUG_INFO
}

void cc1101_hw_set_channel(CC1101_HW* cc1101, uint8_t channel_num)
{
#if (CC1101_DEBUG)
    printf("CC1101: set channel %u\n", channel_num);
#endif // CC1101_DEBUG_INFO
    cc1101->channel = channel_num;
    cc1101_go_idle(cc1101);
    cc1101_write_register(CC_CHANNR, cc1101->channel);
}

void cc1101_hw_set_tx_power(CC1101_HW* cc1101, uint8_t power)
{
#if (CC1101_DEBUG)
    printf("CC1101: set tx power %X\n", power);
#endif // CC1101_DEBUG_INFO
    cc1101_write_register(CC_PATABLE, power);

    printf("tx power %X\n", cc1101_read_register(CC_PATABLE));
}

void cc1101_hw_set_radio_pkt_size(CC1101_HW* cc1101, uint8_t size)
{
#if (CC1101_DEBUG)
    printf("CC1101: set packet size to %d\n", size);
#endif // CC1101_DEBUG_INFO
    cc1101->packet_size = size;
    cc1101_write_register(CC_PKTLEN, size);
}

void cc1101_hw_tx(CC1101_HW* cc1101, HANDLE process, IO* io, unsigned int size)
{
#if (CC1101_DEBUG)
    printf("CC1101: TX\n");
#endif // CC1101_DEBUG_INFO
    CC1101_STACK* stack = io_stack(io);
    io_pop(io, sizeof(CC1101_STACK));

    if(cc1101->packet_size != io->data_size)
        cc1101_hw_set_radio_pkt_size(cc1101, io->data_size);

    cc1101->process = process;
    cc1101->io = io;

    cc1101_go_idle(cc1101);
    cc1101_prepare_tx(cc1101, io_data(io), io->data_size);
    cc1101_start_tx(cc1101);

    if(stack->flags & CC1101_FLAGS_TRANSMIT_ACK)
        cc1101->state = CC1101_STATE_TX_ACK;
    else
        cc1101->state = CC1101_STATE_TX;

    NVIC_EnableIRQ(CC1101_GDO0_EXTI_IRQ);
    error(ERROR_SYNC);
}

void cc1101_hw_rx(CC1101_HW* cc1101, IO* io)
{
#if (CC1101_DEBUG)
    printf("CC1101: start RX\n");
#endif // CC1101_DEBUG_INFO
    cc1101_go_idle(cc1101);
    cc1101_flush_rx_fifo(cc1101);
    cc1101_start_rx(cc1101);
    cc1101->state = CC1101_STATE_RX;
    cc1101->io = io;
    NVIC_EnableIRQ(CC1101_GDO2_EXTI_IRQ);
}

int cc1101_hw_receive_packet(CC1101_HW* cc1101, IO* io)
{
    unsigned int res = 0;
    uint8_t status = cc1101_read_register(CC_PKTSTATUS);
    uint8_t* data = io_data(io);

    if(status & CC_STATUS_CRC_OK)
    {
        gpio_reset_pin(CC1101_CS_PIN);
        while(cc1101_busy());

        spi_byte(CC1101_SPI, CC_FIFO | CC_READ_FLAG | CC_BURST_FLAG);

        for(uint8_t i = 0; i < cc1101->packet_size; i++)
             data[res++] = spi_byte(CC1101_SPI, 0);

        // RSSI
        data[res++] = spi_byte(CC1101_SPI, 0);
        // LQI
        data[res++] = spi_byte(CC1101_SPI, 0);
        gpio_set_pin(CC1101_CS_PIN);

        // TODO: RSSI transform
        int Rssi = RSSI_dBm(data[cc1101->packet_size]);

#if (CC1101_DEBUG_REQUESTS)
    cc1101_dump(data, cc1101->packet_size, "RX PACKET");
    printf("RSSI: %d\n", Rssi);
#endif // CC1101_DEBUG_REQUESTS
        return res;
    }
    else
    {
#if (CC1101_DEBUG_ERRORS)
        printf("CC1101: status: %X\n", status);
        if(status & CC_STATUS_CARIER_SENSE)
            printf("Carrier sense\n");
        if(status & CC_STATUS_PQT_REACHED)
            printf("PQT reached\n");
        if(status & CC_STATUS_CHANNEL_CLEAR)
            printf("Channel clear\n");
        if(status & CC_STATUS_SFD)
            printf("Sync word has been reveived\n");

        //Note: the reading gives the non-inverted value irrespective of what IOCFG0.GDO0_INV is programmed to.
        if(status & CC_STATUS_GDO2)
            printf("GDO2 low value\n");
        if(status & CC_STATUS_GDO0)
            printf("GDO0 low value\n");
#endif // CC1101_DEBUG_ERRORS
        return res;
    }
}

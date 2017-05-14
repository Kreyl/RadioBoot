/*
 * stm32_spi.c
 *
 *  Created on: 1 ���. 2017 �.
 *      Author: RomaJam
 */

#include "stm32_spi.h"
#include "stm32_core_private.h"
#include "../../userspace/stdlib.h"
#include "../../userspace/stdio.h"
#include "../../userspace/irq.h"
#include "../../userspace/stm32/stm32_driver.h"


typedef SPI_TypeDef* SPI_TypeDef_P;
#if (SPI_COUNT > 1)
static const SPI_TypeDef_P __SPI_REGS[] =                               {SPI1, SPI2};
static const uint8_t __SPI_VECTORS[] =                                  {35, 36};
#else
static const SPI_TypeDef_P __SPI_REGS[] =                               {SPI1};
static const uint8_t __SPI_VECTORS[] =                                  {35};
static const uint32_t __SPI_CLOCK[] =                                   {RCC_APB2ENR_SPI1EN};
#endif


void stm32_spi_init(CORE* core)
{
    int i;
    for (i = 0; i < SPI_COUNT; ++i)
        core->spi.spis[i] = NULL;
}

static inline void stm32_spi_enable(SPI_TypeDef* SPI, bool enable)
{
    if(enable)
        SPI->CR1 |=  SPI_CR1_SPE;
    else
        SPI->CR1 &= ~SPI_CR1_SPE;
}

static void stm32_spi_on_status_isr(SPI* spi, SPI_PORT port)
{
    if(__SPI_REGS[port]->SR & SPI_SR_OVR)
        iprintd("spi %d overrun\n", port);
    if(__SPI_REGS[port]->SR & SPI_SR_MODF)
        iprintd("spi %d mode fault\n", port);
    if(__SPI_REGS[port]->SR & SPI_SR_BSY)
        iprintd("spi %d busy\n", port);
    if(__SPI_REGS[port]->SR & SPI_SR_UDR)
        iprintd("spi %d underrun\n", port);
    if(__SPI_REGS[port]->SR & SPI_SR_TXE)
        iprintd("spi %d txe\n", port);
    if(__SPI_REGS[port]->SR & SPI_SR_RXNE)
        iprintd("spi %d rxne\n", port);
}

static inline void stm32_spi_on_rx_isr(SPI* spi, SPI_PORT port)
{

    *(uint8_t*)(io_data(spi->io) + spi->rx_length) = __SPI_REGS[port]->DR;
//    iprintd("spi rx isr %d\n", spi->rx_length);
    spi->rx_length++;

    if(spi->io->data_size == spi->rx_length)
    {
//        iprintd("spi tx done\n");
        spi->io->data_size = spi->rx_length;
        iio_complete(spi->process, HAL_IO_CMD(HAL_SPI, IPC_READ), port, spi->io);
    }
}

static inline void stm32_spi_on_tx_isr(SPI* spi, SPI_PORT port)
{
//    iprintd("spi tx isr %d\n", spi->tx_length);
    if(spi->tx_length-- > 0)
        __SPI_REGS[port]->DR = *(uint8_t*)(io_data(spi->io) + (spi->io->data_size - spi->tx_length));
}

void stm32_spi_on_isr(int vector, void* param)
{
    SPI_PORT port;
    SPI* spi;
    CORE* core = param;
    port = SPI_1;
#if (SPI_COUNT > 1)
    if (vector != __SPI_VECTORS[0])
        port = SPI_2;
#endif
    spi = core->spi.spis[port];

    if (__SPI_REGS[port]->SR & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_BSY))
    {
        stm32_spi_on_status_isr(spi, port);
    }

    if(__SPI_REGS[port]->SR & SPI_SR_TXE)
    {
        stm32_spi_on_tx_isr(spi, port);
    }

    if(__SPI_REGS[port]->SR & SPI_SR_RXNE)
    {
        stm32_spi_on_rx_isr(spi, port);
    }
}

void stm32_spi_open(CORE* core, SPI_PORT port, unsigned int settings)
{
    SPI* spi = core->spi.spis[port];
    if (spi)
    {
        error(ERROR_ALREADY_CONFIGURED);
        return;
    }
    spi = malloc(sizeof(SPI));
    if (spi == NULL)
    {
        error(ERROR_OUT_OF_MEMORY);
        return;
    }
    core->spi.spis[port] = spi;

    spi->io = NULL;
    stm32_spi_enable(__SPI_REGS[port], false);
    // Enable clocking
    switch(port)
    {
        case SPI_1:
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
            break;
        case SPI_2:
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
            break;
        default:
        {
            error(ERROR_HARDWARE);
            return;
        }
    }

    __SPI_REGS[port]->SR = SPI_SR_TXE;
    __SPI_REGS[port]->CR1 = settings;
    __SPI_REGS[port]->CR2 = SPI_CR2_RXNEIE | SPI_CR2_ERRIE;
    __SPI_REGS[port]->I2SCFGR &= ~((uint16_t)SPI_I2SCFGR_I2SMOD); // Disable I2S
    stm32_spi_enable(__SPI_REGS[port], true);

    irq_register(__SPI_VECTORS[port], stm32_spi_on_isr, (void*)core);
    NVIC_EnableIRQ(__SPI_VECTORS[port]);
    NVIC_SetPriority(__SPI_VECTORS[port], 13);
}

void stm32_spi_close(CORE* core, SPI_PORT port)
{
    printd("spi close\n");
}

static void stm32_spi_data_io(CORE* core, IPC* ipc)
{
    printd("spi io\n");
    SPI_PORT port = (SPI_PORT)ipc->param1;
    unsigned int max_size = ipc->param3;
    SPI* spi = core->spi.spis[port];
    if (spi == NULL)
    {
        error(ERROR_NOT_CONFIGURED);
        return;
    }
    spi->process = ipc->process;
    spi->io = (IO*)ipc->param2;
    spi->tx_length = spi->io->data_size;
    spi->rx_length = 0;

    if(spi->tx_length > max_size)
        spi->tx_length = max_size;

    __SPI_REGS[port]->DR = *(uint8_t*)io_data(spi->io);
    spi->tx_length--;
    //all rest in isr
    error(ERROR_SYNC);
}

void stm32_spi_request(CORE* core, IPC* ipc)
{
    SPI_PORT port = (SPI_PORT)ipc->param1;
    if (port >= SPI_COUNT)
    {
        error(ERROR_INVALID_PARAMS);
        return;
    }

    switch (HAL_ITEM(ipc->cmd))
    {
        case IPC_OPEN:
            stm32_spi_open(core, port, ipc->param2);
            break;
        case IPC_CLOSE:
            stm32_spi_close(core, port);
            break;
        case IPC_READ:
            stm32_spi_data_io(core, ipc);
            break;
        default:
            error(ERROR_NOT_SUPPORTED);
            break;
    }
}

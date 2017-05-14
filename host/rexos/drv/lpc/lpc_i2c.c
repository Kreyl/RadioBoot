/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2017, Alexey Kramarenko
    All rights reserved.
*/

#include "lpc_i2c.h"
#include "lpc_pin.h"
#include "lpc_core_private.h"
#include "lpc_power.h"
#include "../../userspace/stdlib.h"
#include "../../userspace/irq.h"
#include "../../userspace/systime.h"

#if defined(LPC11U6x)
static const uint8_t __I2C_VECTORS[] =              {15, 10};
typedef LPC_I2C_Type* LPC_I2C_Type_P;
static const LPC_I2C_Type_P __I2C_REGS[] =          {LPC_I2C};
static const uint8_t __I2C_POWER_PINS[] =           {SYSCON_SYSAHBCLKCTRL_I2C0_POS, SYSCON_SYSAHBCLKCTRL_I2C1_POS};
static const uint8_t __I2C_RESET_PINS[] =           {SYSCON_PRESETCTRL_I2C0_RST_N_POS, SYSCON_PRESETCTRL_I2C1_RST_N_POS};
#elif defined(LPC11Uxx)
typedef LPC_I2C0_Type* LPC_I2C_Type_P;
static const LPC_I2C_Type_P __I2C_REGS[] =          {LPC_I2C0, LPC_I2C1};
static const uint8_t __I2C_VECTORS[] =              {15};
static const uint8_t __I2C_POWER_PINS[] =           {SYSCON_SYSAHBCLKCTRL_I2C0_POS};
static const uint8_t __I2C_RESET_PINS[] =           {SYSCON_PRESETCTRL_I2C0_RST_N_POS};
#else //LPC18xx
typedef LPC_I2Cn_Type* LPC_I2C_Type_P;
static const LPC_I2C_Type_P __I2C_REGS[] =          {LPC_I2C0, LPC_I2C1};
static const uint8_t __I2C_VECTORS[] =              {18, 19};
#endif //LPC11U6x

#define I2C_CLEAR (I2C0_CONCLR_AAC_Msk | I2C0_CONCLR_SIC_Msk | I2C0_CONCLR_STAC_Msk)

#if (LPC_I2C_TIMEOUT_MS)
static void lpc_i2c_timer_istop(CORE* core, I2C_PORT port)
{
    ipc_ipost_inline(process_iget_current(), HAL_CMD(HAL_I2C, IPC_TIMEOUT), port, 0, 0);
    core->i2c.i2cs[port]->timer_need_stop = true;
}
#endif //LPC_I2C_TIMEOUT_MS

static void lpc_i2c_isr_error(CORE* core, I2C_PORT port, int error)
{
    I2C* i2c = core->i2c.i2cs[port];
    __I2C_REGS[port]->CONSET = I2C0_CONSET_STO_Msk;
#if (LPC_I2C_TIMEOUT_MS)
    lpc_i2c_timer_istop(core, port);
#endif
    iio_complete_ex(i2c->process, HAL_IO_CMD(HAL_I2C, (i2c->io_mode == I2C_IO_MODE_TX) ? IPC_WRITE : IPC_READ), port, i2c->io, error);
    i2c->io_mode = I2C_IO_MODE_IDLE;
}

static inline void lpc_i2c_isr_tx(CORE* core, I2C_PORT port)
{
    I2C* i2c = core->i2c.i2cs[port];
    switch(__I2C_REGS[port]->STAT)
    {
    case I2C0_STAT_START:
        //transmit sla
        __I2C_REGS[port]->DAT = (i2c->stack->sla << 1) | 0;
        __I2C_REGS[port]->CONCLR = I2C0_CONCLR_STAC_Msk;
        break;
    case I2C0_STAT_SLAW_NACK:
        lpc_i2c_isr_error(core, port, ERROR_NAK);
        break;
    case I2C0_STAT_DATW_NACK:
        //only acceptable for last byte
        if ((i2c->processed < i2c->size) || (i2c->state != I2C_STATE_DATA))
        {
            lpc_i2c_isr_error(core, port, ERROR_NAK);
            break;
        }
        //follow down
    case I2C0_STAT_SLAW_ACK:
    case I2C0_STAT_DATW_ACK:
        switch (i2c->state)
        {
        case I2C_STATE_ADDR:
            __I2C_REGS[port]->DAT = i2c->stack->addr;
            i2c->state = (i2c->stack->flags & I2C_FLAG_LEN) ? I2C_STATE_LEN : I2C_STATE_DATA;
            break;
        case I2C_STATE_LEN:
            __I2C_REGS[port]->DAT = (uint8_t)i2c->io->data_size;
            i2c->state = I2C_STATE_DATA;
            break;
            break;
        default:
            if (i2c->processed < i2c->size)
                __I2C_REGS[port]->DAT = ((uint8_t*)io_data(i2c->io))[i2c->processed++];
            //last byte
            else
            {
                __I2C_REGS[port]->CONSET = I2C0_CONSET_STO_Msk;
#if (LPC_I2C_TIMEOUT_MS)
                lpc_i2c_timer_istop(core, port);
#endif
                iio_complete_ex(i2c->process, HAL_IO_CMD(HAL_I2C, IPC_WRITE), port, i2c->io, i2c->processed);
                i2c->io_mode = I2C_IO_MODE_IDLE;
            }
        }
        break;
    default:
         lpc_i2c_isr_error(core, port, ERROR_INVALID_STATE);
         break;
    }
}

static inline void lpc_i2c_isr_rx(CORE* core, I2C_PORT port)
{
    I2C* i2c = core->i2c.i2cs[port];
    switch(__I2C_REGS[port]->STAT)
    {
    case I2C0_STAT_START:
        //transmit address W
        if (i2c->state == I2C_STATE_ADDR)
        {
            __I2C_REGS[port]->DAT = (i2c->stack->sla << 1) | 0;
            __I2C_REGS[port]->CONCLR = I2C0_CONCLR_STAC_Msk;
            i2c->state = (i2c->stack->flags & I2C_FLAG_LEN) ? I2C_STATE_LEN : I2C_STATE_DATA;
            break;
        }
    case I2C0_STAT_REPEATED_START:
        //transmit address R
        __I2C_REGS[port]->CONCLR = I2C0_CONCLR_STAC_Msk;
        __I2C_REGS[port]->DAT = (i2c->stack->sla << 1) | 1;
        break;
    case I2C0_STAT_SLAR_NACK:
    case I2C0_STAT_SLAW_NACK:
        lpc_i2c_isr_error(core, port, ERROR_NAK);
        break;
    case I2C0_STAT_SLAW_ACK:
        //transmit address
        __I2C_REGS[port]->DAT = i2c->stack->addr;
        break;
    case I2C0_STAT_DATW_NACK:
    case I2C0_STAT_DATW_ACK:
        //adress transmitted (NACK also acceptable), rS
        __I2C_REGS[port]->CONSET = I2C0_CONSET_STA_Msk;
        break;
    case I2C0_STAT_SLAR_ACK:
        //more than 1 byte
        if (i2c->size || (i2c->stack->flags & I2C_FLAG_LEN))
            __I2C_REGS[port]->CONSET = I2C0_CONSET_AA_Msk;
        else
            __I2C_REGS[port]->CONCLR = I2C0_CONCLR_AAC_Msk;
        break;
    case I2C0_STAT_DATR_ACK:
        switch (i2c->state)
        {
        case I2C_STATE_LEN:
            if (__I2C_REGS[port]->DAT < i2c->size)
                i2c->size = __I2C_REGS[port]->DAT;
            i2c->state = I2C_STATE_DATA;
            break;
        case I2C_STATE_DATA:
            ((uint8_t*)io_data(i2c->io))[i2c->processed++] = __I2C_REGS[port]->DAT;
            break;
        //data state
        default:
            lpc_i2c_isr_error(core, port, ERROR_INVALID_STATE);
        }
        //need more? send ACK
        if (i2c->processed + 1 < i2c->size)
            __I2C_REGS[port]->CONSET = I2C0_CONSET_AA_Msk;
        //received all - send NAK
        else
            __I2C_REGS[port]->CONCLR = I2C0_CONCLR_AAC_Msk;
        break;
    case I2C0_STAT_DATR_NACK:
        //last byte, if required
        if (i2c->processed < i2c->size)
            ((uint8_t*)io_data(i2c->io))[i2c->processed++] = __I2C_REGS[port]->DAT;
        //stop transmission
        __I2C_REGS[port]->CONSET = I2C0_CONSET_STO_Msk;
#if (LPC_I2C_TIMEOUT_MS)
        lpc_i2c_timer_istop(core, port);
#endif
        i2c->io->data_size = i2c->processed;
        iio_complete(i2c->process, HAL_IO_CMD(HAL_I2C, IPC_READ), port, i2c->io);
        i2c->io_mode = I2C_IO_MODE_IDLE;
        break;
    default:
        lpc_i2c_isr_error(core, port, ERROR_INVALID_STATE);
        break;
    }
}

void lpc_i2c_on_isr(int vector, void* param)
{
    CORE* core = (CORE*)param;
    I2C_PORT port = I2C_0;
#if defined(LPC11U6x) || defined(LPC18xx)
    if (vector != __I2C_VECTORS[0])
        port = I2C_1;
#endif //defined(LPC11U6x) || defined(LPC18xx)
    switch (core->i2c.i2cs[port]->io_mode)
    {
    case I2C_IO_MODE_TX:
        lpc_i2c_isr_tx(core, port);
        break;
    case I2C_IO_MODE_RX:
        lpc_i2c_isr_rx(core, port);
        break;
    default:
        break;
    }
    __I2C_REGS[port]->CONCLR = I2C0_CONCLR_SIC_Msk;
}

void lpc_i2c_open(CORE* core, I2C_PORT port, unsigned int mode, unsigned int speed)
{
    I2C* i2c = core->i2c.i2cs[port];
    if (i2c)
    {
        error(ERROR_ALREADY_CONFIGURED);
        return;
    }
    i2c = malloc(sizeof(I2C));
    core->i2c.i2cs[port] = i2c;
    if (i2c == NULL)
    {
        error(ERROR_OUT_OF_MEMORY);
        return;
    }
#if (LPC_I2C_TIMEOUT_MS)
    i2c->timer = timer_create(port, HAL_I2C);
    if (i2c->timer == INVALID_HANDLE)
    {
        free(i2c);
        core->i2c.i2cs[port] = NULL;
        return;
    }
    i2c->timer_need_stop = false;
#endif
    i2c->io = NULL;
    i2c->io_mode = I2C_IO_MODE_IDLE;

#if defined(LPC11Uxx)
    //power up
    LPC_SYSCON->SYSAHBCLKCTRL |= 1 << __I2C_POWER_PINS[port];
    //remove reset state
    LPC_SYSCON->PRESETCTRL |= 1 << __I2C_RESET_PINS[port];
#else //LPC18xx
    if (port == I2C_0)
    {
        //connect APB1 to PLL1
        LPC_CGU->BASE_APB1_CLK = CGU_BASE_APB1_CLK_PD_Pos;
        LPC_CGU->BASE_APB1_CLK |= CGU_CLK_PLL1;
        LPC_CGU->BASE_APB1_CLK &= ~CGU_BASE_APB1_CLK_PD_Pos;
        LPC_SCU->SFSI2C0 = SCU_SFSI2C0_SCL_EZI | SCU_SFSI2C0_SDA_EZI;
    }
    else
    {
        //connect APB3 to PLL1
        LPC_CGU->BASE_APB3_CLK = CGU_BASE_APB3_CLK_PD_Pos;
        LPC_CGU->BASE_APB3_CLK |= CGU_CLK_PLL1;
        LPC_CGU->BASE_APB3_CLK &= ~CGU_BASE_APB3_CLK_PD_Pos;
        //pin configured by user
    }

#endif //LPC11Uxx
    //setup clock
    __I2C_REGS[port]->SCLL = __I2C_REGS[port]->SCLH = lpc_power_get_clock_inside(POWER_BUS_CLOCK) / (speed) / 2;
    //reset state machine
    __I2C_REGS[port]->CONCLR = I2C_CLEAR;
    //enable interrupt
    irq_register(__I2C_VECTORS[port], lpc_i2c_on_isr, (void*)core);
    NVIC_EnableIRQ(__I2C_VECTORS[port]);
    NVIC_SetPriority(__I2C_VECTORS[port], 2);
}

void lpc_i2c_close(CORE* core, I2C_PORT port)
{
    I2C* i2c = core->i2c.i2cs[port];
    if (i2c == NULL)
    {
        error(ERROR_NOT_CONFIGURED);
        return;
    }
    //disable interrupt
    NVIC_DisableIRQ(__I2C_VECTORS[port]);
    irq_unregister(__I2C_VECTORS[port]);

#if defined(LPC11Uxx)
    //set reset state
    LPC_SYSCON->PRESETCTRL &= ~(1 << __I2C_RESET_PINS[port]);
    //power down
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << __I2C_POWER_PINS[port]);
#else //LPC18xx
    if (port == I2C_0)
        LPC_SCU->SFSI2C0 = 0;
#endif //LPC11Uxx

#if (LPC_I2C_TIMEOUT_MS)
    timer_destroy(i2c->timer);
#endif
    free(i2c);
    core->i2c.i2cs[port] = NULL;
}

static void lpc_i2c_io(CORE* core, IPC* ipc, bool read)
{
    I2C_PORT port = (I2C_PORT)ipc->param1;
    I2C* i2c = core->i2c.i2cs[port];
    if (i2c == NULL)
    {
        error(ERROR_NOT_CONFIGURED);
        return;
    }
    if (i2c->io_mode != I2C_IO_MODE_IDLE)
    {
        error(ERROR_IN_PROGRESS);
        return;
    }
    i2c->process = ipc->process;
    i2c->io = (IO*)ipc->param2;
    i2c->processed = 0;
    i2c->stack = io_stack(i2c->io);
    if (i2c->stack->flags & I2C_FLAG_ADDR)
        i2c->state = I2C_STATE_ADDR;
    else if (i2c->stack->flags & I2C_FLAG_LEN)
        i2c->state = I2C_STATE_LEN;
    else
        i2c->state = I2C_STATE_DATA;

    if (read)
    {
        i2c->io->data_size = 0;
        i2c->size = ipc->param3;
        i2c->io_mode = I2C_IO_MODE_RX;
    }
    else
    {
        i2c->size = i2c->io->data_size;
        i2c->io_mode = I2C_IO_MODE_TX;
    }

    //reset
    __I2C_REGS[port]->CONCLR = I2C_CLEAR;
#if (LPC_I2C_TIMEOUT_MS)
    timer_start_ms(i2c->timer, LPC_I2C_TIMEOUT_MS);
#endif

    //set START
    __I2C_REGS[port]->CONSET = I2C0_CONSET_I2EN_Msk | I2C0_CONSET_STA_Msk;
    //all rest in isr
    error(ERROR_SYNC);
}

#if (LPC_I2C_TIMEOUT_MS)
static inline void lpc_i2c_timeout(CORE* core, I2C_PORT port)
{
    I2C_IO_MODE io_mode;
    I2C* i2c = core->i2c.i2cs[port];
    if (i2c->timer_need_stop)
    {
        timer_stop(i2c->timer, port, HAL_I2C);
        i2c->timer_need_stop = false;
        return;
    }
    __disable_irq();
    io_mode = i2c->io_mode;
    i2c->io_mode = I2C_IO_MODE_IDLE;
    __enable_irq();
    __I2C_REGS[port]->CONSET = I2C0_CONSET_STO_Msk;
    io_complete_ex(i2c->process, HAL_IO_CMD(HAL_I2C, (io_mode == I2C_IO_MODE_TX) ? IPC_WRITE : IPC_READ), port, i2c->io, ERROR_TIMEOUT);
}
#endif

void lpc_i2c_init(CORE* core)
{
    int i;
    for (i = 0; i < I2C_COUNT; ++i)
        core->i2c.i2cs[i] = NULL;
}

void lpc_i2c_request(CORE* core, IPC* ipc)
{
    I2C_PORT port = (I2C_PORT)ipc->param1;
    if (port >= I2C_COUNT)
    {
        error(ERROR_INVALID_PARAMS);
        return;
    }
    switch (HAL_ITEM(ipc->cmd))
    {
    case IPC_OPEN:
        lpc_i2c_open(core, port, ipc->param2, ipc->param3);
        break;
    case IPC_CLOSE:
        lpc_i2c_close(core, port);
        break;
    case IPC_WRITE:
        lpc_i2c_io(core, ipc, false);
        break;
    case IPC_READ:
        lpc_i2c_io(core, ipc, true);
        //async message, no write
        break;
#if (LPC_I2C_TIMEOUT_MS)
    case IPC_TIMEOUT:
        lpc_i2c_timeout(core, port);
        break;
#endif
    default:
        error(ERROR_NOT_SUPPORTED);
        break;
    }
}

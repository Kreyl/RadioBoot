/*
 * kl_lib_f0.cpp
 *
 *  Created on: 10.12.2012
 *      Author: kreyl
 */

#include <kl_lib.h>
#include <stdarg.h>
#include <string.h>
#include <uart.h>
#include "main.h"   // App is there

#if 1 // ============================= Timer ===================================
void Timer_t::Init() {
#if defined STM32L1XX
    if(ANY_OF_3(ITmr, TIM9, TIM10, TIM11)) PClk = &Clk.APB2FreqHz;
    else PClk = &Clk.APB1FreqHz;
    if     (ITmr == TIM2)  { rccEnableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
    else if(ITmr == TIM4)  { rccEnableTIM4(FALSE); }
    else if(ITmr == TIM6)  { rccEnableAPB1(RCC_APB1ENR_TIM6EN,  FALSE); }
    else if(ITmr == TIM7)  { rccEnableAPB1(RCC_APB1ENR_TIM7EN,  FALSE); }
    else if(ITmr == TIM9)  { rccEnableAPB2(RCC_APB2ENR_TIM9EN,  FALSE); }
    else if(ITmr == TIM10) { rccEnableAPB2(RCC_APB2ENR_TIM10EN, FALSE); }
    else if(ITmr == TIM11) { rccEnableAPB2(RCC_APB2ENR_TIM11EN, FALSE); }
#elif defined STM32F0XX
    if     (ITmr == TIM1)  { rccEnableTIM1(FALSE); }
    else if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
#ifdef TIM6
    else if(ITmr == TIM6)  { rccEnableAPB1(RCC_APB1ENR_TIM6EN,  FALSE); }
#endif
    else if(ITmr == TIM14) { RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; }
#ifdef TIM15
    else if(ITmr == TIM15) { RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; }
#endif
    else if(ITmr == TIM16) { RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; }
    else if(ITmr == TIM17) { RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; }
    // Clock src
    PClk = &Clk.APBFreqHz;
#elif defined STM32F2XX || defined STM32F4XX
    if(ANY_OF_5(ITmr, TIM1, TIM8, TIM9, TIM10, TIM11)) PClk = &Clk.APB2FreqHz;
    else PClk = &Clk.APB1FreqHz;
    if     (ITmr == TIM1)  { rccEnableTIM1(FALSE); }
    else if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
    else if(ITmr == TIM4)  { rccEnableTIM4(FALSE); }
    else if(ITmr == TIM5)  { rccEnableTIM5(FALSE); }
    else if(ITmr == TIM6)  { rccEnableTIM6(FALSE); }
    else if(ITmr == TIM7)  { rccEnableTIM7(FALSE); }
    else if(ITmr == TIM8)  { rccEnableTIM8(FALSE); }
    else if(ITmr == TIM9)  { rccEnableTIM9(FALSE); }
    else if(ITmr == TIM10)  { RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; }
    else if(ITmr == TIM11)  { rccEnableTIM11(FALSE); }
    else if(ITmr == TIM12)  { rccEnableTIM12(FALSE); }
    else if(ITmr == TIM13)  { RCC->APB1ENR |= RCC_APB1ENR_TIM13EN; }
    else if(ITmr == TIM14)  { rccEnableTIM14(FALSE); }
#elif defined STM32F10X_LD_VL
    if(ANY_OF_4(ITmr, TIM1, TIM15, TIM16, TIM17)) PClk = &Clk.APB2FreqHz;
    else PClk = &Clk.APB1FreqHz;
    if     (ITmr == TIM1)  { rccEnableTIM1(FALSE); }
    else if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
    else if(ITmr == TIM15) { RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; }
    else if(ITmr == TIM16) { RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; }
    else if(ITmr == TIM17) { RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; }
#endif
}

void Timer_t::Deinit() {
    TMR_DISABLE(ITmr);
#if defined STM32F0XX
    if     (ITmr == TIM1)  { rccDisableTIM1(); }
    else if(ITmr == TIM2)  { rccDisableTIM2(); }
    else if(ITmr == TIM3)  { rccDisableTIM3(); }
#ifdef TIM6
    else if(ITmr == TIM6)  { rccDisableTIM6(); }
#endif
    else if(ITmr == TIM14) { rccDisableTIM14(); }
#ifdef TIM15
    else if(ITmr == TIM15) { rccDisableTIM15(); }
#endif
    else if(ITmr == TIM16) { rccDisableTIM16(); }
    else if(ITmr == TIM17) { rccDisableTIM17(); }
#endif
}

void Timer_t::InitPwm(GPIO_TypeDef *GPIO, uint16_t N, uint8_t Chnl, uint32_t ATopValue, Inverted_t Inverted, PinOutMode_t OutputType) {
    // GPIO
#if defined STM32L1XX
    if              (ITmr == TIM2)              PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF1);
    else if(ANY_OF_2(ITmr, TIM3, TIM4))         PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF2);
    else if(ANY_OF_3(ITmr, TIM9, TIM10, TIM11)) PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF3);
#elif defined STM32F0XX
    if     (ITmr == TIM1)  PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF2);
    else if(ITmr == TIM3)  PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF1);
    else if(ITmr == TIM14) {
        if(GPIO == GPIOA) PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF4);
        else PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF0);
    }
#ifdef TIM15
    else if(ITmr == TIM15) {
        if(GPIO == GPIOA) PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF0);
        else PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF1);
    }
#endif
    else if(ITmr == TIM16 or ITmr == TIM17) {
        if(GPIO == GPIOA) PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF5);
        else PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF2);
    }
#elif defined STM32F2XX || defined STM32F4XX
    if(ANY_OF_2(ITmr, TIM1, TIM2)) PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF1);
    else if(ANY_OF_3(ITmr, TIM3, TIM4, TIM5)) PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF2);
    else if(ANY_OF_4(ITmr, TIM8, TIM9, TIM10, TIM11)) PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF3);
    else if(ANY_OF_3(ITmr, TIM12, TIM13, TIM14)) PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF9);
#elif defined STM32F100_MCUCONF
    PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF0);   // Alternate function is dummy
//    ITmr->BDTR = 0xC000;   // Main output Enable
#endif
#if !defined STM32L151xB
    ITmr->BDTR = 0xC000;   // Main output Enable
#endif
    ITmr->ARR = ATopValue;
    // Output
    uint16_t tmp = (Inverted == invInverted)? 0b111 : 0b110; // PWM mode 1 or 2
    switch(Chnl) {
        case 1:
            ITmr->CCMR1 |= (tmp << 4);
            ITmr->CCER  |= TIM_CCER_CC1E;
            break;
        case 2:
            ITmr->CCMR1 |= (tmp << 12);
            ITmr->CCER  |= TIM_CCER_CC2E;
            break;
        case 3:
            ITmr->CCMR2 |= (tmp << 4);
            ITmr->CCER  |= TIM_CCER_CC3E;
            break;
        case 4:
            ITmr->CCMR2 |= (tmp << 12);
            ITmr->CCER  |= TIM_CCER_CC4E;
            break;
        default: break;
    }
}

void Timer_t::SetUpdateFrequency(uint32_t FreqHz) {
#if defined STM32F2XX || defined STM32F4XX
    if(ANY_OF_5(ITmr, TIM1, TIM8, TIM9, TIM10, TIM11))  // APB2 is clock src
    	SetTopValue((*PClk * Clk.TimerAPB2ClkMulti) / FreqHz);
    else // APB1 is clock src
    	SetTopValue((*PClk * Clk.TimerAPB1ClkMulti) / FreqHz);
#elif defined STM32L1XX
    uint32_t TopVal;
    if(ANY_OF_3(ITmr, TIM9, TIM10, TIM11)) // APB2 is clock src
        TopVal  = ((*PClk * Clk.Timer9_11ClkMulti) / FreqHz) - 1;
    else TopVal = ((*PClk * Clk.Timer2_7ClkMulti) / FreqHz) - 1;
//    Uart.Printf("Topval = %u\r", TopVal);
    SetTopValue(TopVal);
#else
//    uint32_t UpdFreqMax = *PClk / (ITmr->ARR + 1);
#endif
	ITmr->CNT = 0;  // Reset counter to start from scratch
//#if defined STM32F2XX || defined STM32F4XX
//    uint32_t UpdFreqMax;
//    if(ANY_OF_5(ITmr, TIM1, TIM8, TIM9, TIM10, TIM11))  // APB2 is clock src
//        UpdFreqMax = (*PClk) * Clk.TimerAPB2ClkMulti / (ITmr->ARR + 1);
//    else // APB1 is clock src
//        UpdFreqMax = (*PClk) * Clk.TimerAPB1ClkMulti / (ITmr->ARR + 1);
//#else
//    uint32_t UpdFreqMax = *PClk / (ITmr->ARR + 1);
//#endif
//    uint32_t div = UpdFreqMax / FreqHz;
//    if(div != 0) div--;
//    ITmr->PSC = div;
//    Uart.Printf("\r  FMax=%u; div=%u", UpdFreqMax, div);
}
#endif

#if VIRTUAL_TIMER_KL // =================== Virtual Timers =====================
// Universal VirtualTimer callback
void TmrVirtualCallback(void *p) {
    reinterpret_cast<TmrVirtual_t*>(p)->CallbackHandler();
}
#endif

#if CH_DBG_ENABLED // ========================= DEBUG ==========================
void chDbgPanic(const char *msg1) {
#if CH_USE_REGISTRY
    Uart.PrintfNow("\r%S @ %S\r", msg1, chThdSelf()->p_name);
#else
    Uart.PrintfNow("\r%S\r", msg1);
#endif
}
#endif

#if I2C_REQUIRED // ============================= I2C ==========================
void i2cDmaIrqHandler(void *p, uint32_t flags) {
    chSysLockFromISR();
    i2c_t *pi2c = (i2c_t*)p;
//    Uart.PrintfNow("\r===T===");
    chThdResumeI(&pi2c->ThdRef, (msg_t)0);
    chSysUnlockFromISR();
}

void i2c_t::Init() {
    Standby();
    Resume();
    // ==== DMA ====
    // Here only unchanged parameters of the DMA are configured.
#ifdef STM32F2XX
    if      (ii2c == I2C1) DmaChnl = 1;
    else if (ii2c == I2C2) DmaChnl = 7;
    else                   DmaChnl = 3;   // I2C3
#endif
    dmaStreamAllocate(PDmaTx, IRQ_PRIO_MEDIUM, i2cDmaIrqHandler, this);
    dmaStreamSetPeripheral(PDmaTx, &ii2c->DR);
    dmaStreamAllocate(PDmaRx, IRQ_PRIO_MEDIUM, i2cDmaIrqHandler, this);
    dmaStreamSetPeripheral(PDmaRx, &ii2c->DR);
}

void i2c_t::Standby() {
    if(ii2c == I2C1) { rccResetI2C1(); rccDisableI2C1(FALSE); }
#ifdef RCC_APB1ENR_I2C2EN
    else             { rccResetI2C2(); rccDisableI2C2(FALSE); }
#endif
#if 0
    else if (ii2c == I2C3) { rccResetI2C3(); rccDisableI2C3(FALSE); }
#endif
    // Disable GPIOs
    PinSetupAnalog(IPGpio, ISclPin);
    PinSetupAnalog(IPGpio, ISdaPin);
}

void i2c_t::Resume() {
    Error = false;
    // ==== GPIOs ====
    PinSetupAlterFunc(IPGpio, ISclPin, omOpenDrain, pudNone, AF4);
    PinSetupAlterFunc(IPGpio, ISdaPin, omOpenDrain, pudNone, AF4);
    // ==== Clock and reset ====
    if(ii2c == I2C1) { rccEnableI2C1(FALSE); rccResetI2C1(); }
#ifdef I2C2
    else if (ii2c == I2C2) { rccEnableI2C2(FALSE); rccResetI2C2(); }
#endif
#ifdef I2C3
    else if (ii2c == I2C3) { rccEnableI2C3(FALSE); rccResetI2C3(); }
#endif

    // Minimum clock is 2 MHz
    uint32_t ClkMhz = Clk.APB1FreqHz / 1000000;
    uint16_t tmpreg = ii2c->CR2;
    tmpreg &= (uint16_t)~I2C_CR2_FREQ;
    if(ClkMhz < 2)  ClkMhz = 2;
    if(ClkMhz > 32) ClkMhz = 32;
    tmpreg |= ClkMhz;
    ii2c->CR2 = tmpreg;
    ii2c->CR1 &= (uint16_t)~I2C_CR1_PE; // Disable i2c to setup TRise & CCR
    ii2c->TRISE = (uint16_t)(((ClkMhz * 300) / 1000) + 1);
    // 16/9
    tmpreg = (uint16_t)(Clk.APB1FreqHz / (IBitrateHz * 25));
    if(tmpreg == 0) tmpreg = 1; // minimum allowed value
    tmpreg |= I2C_CCR_FS | I2C_CCR_DUTY;
    ii2c->CCR = tmpreg;
    ii2c->CR1 |= I2C_CR1_PE;    // Enable i2c back
    // ==== DMA ====
    ii2c->CR2 |= I2C_CR2_DMAEN;
}

void i2c_t::Reset() {
    Standby();
    Resume();
}

uint8_t i2c_t::WriteRead(uint8_t Addr,
        uint8_t *WPtr, uint8_t WLength,
        uint8_t *RPtr, uint8_t RLength) {
    if(IBusyWait() != OK) return FAILURE;
    // Clear flags
    ii2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)ii2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != OK) return FAILURE;
    SendAddrWithWrite(Addr);
    if(WaitEv6() != OK) { SendStop(); return FAILURE; }
    ClearAddrFlag();
    // Start TX DMA if needed
    if(WLength != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr);
        dmaStreamSetMode   (PDmaTx, I2C_DMATX_MODE);
        dmaStreamSetTransactionSize(PDmaTx, WLength);
        chSysLock();
        dmaStreamEnable(PDmaTx);
        chThdSuspendS(&ThdRef);    // Wait IRQ
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    // Read if needed
    if(RLength != 0) {
        if(WaitEv8() != OK) return FAILURE;
        // Send repeated start
        SendStart();
        if(WaitEv5() != OK) return FAILURE;
        SendAddrWithRead(Addr);
        if(WaitEv6() != OK) { SendStop(); return FAILURE; }
        // If single byte is to be received, disable ACK before clearing ADDR flag
        if(RLength == 1) AckDisable();
        else AckEnable();
        ClearAddrFlag();
        dmaStreamSetMemory0(PDmaRx, RPtr);
        dmaStreamSetMode   (PDmaRx, I2C_DMARX_MODE);
        dmaStreamSetTransactionSize(PDmaRx, RLength);
        SignalLastDmaTransfer(); // Inform DMA that this is last transfer => do not ACK last byte
        chSysLock();
        dmaStreamEnable(PDmaRx);
        chThdSuspendS(&ThdRef);    // Wait IRQ
        chSysUnlock();
        dmaStreamDisable(PDmaRx);
    } // if != 0
    else WaitBTF(); // if nothing to read, just stop
    SendStop();
    return OK;
}

uint8_t i2c_t::WriteWrite(uint8_t Addr,
        uint8_t *WPtr1, uint8_t WLength1,
        uint8_t *WPtr2, uint8_t WLength2) {
    if(IBusyWait() != OK) return FAILURE;
    // Clear flags
    ii2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)ii2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != OK) return FAILURE;
    SendAddrWithWrite(Addr);
    if(WaitEv6() != OK) { SendStop(); return FAILURE; }
    ClearAddrFlag();
    // Start TX DMA if needed
    if(WLength1 != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr1);
        dmaStreamSetMode   (PDmaTx, I2C_DMATX_MODE);
        dmaStreamSetTransactionSize(PDmaTx, WLength1);
        chSysLock();
        dmaStreamEnable(PDmaTx);
        chThdSuspendS(&ThdRef);    // Wait IRQ
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    if(WLength2 != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr2);
        dmaStreamSetMode   (PDmaTx, I2C_DMATX_MODE);
        dmaStreamSetTransactionSize(PDmaTx, WLength2);
        chSysLock();
        dmaStreamEnable(PDmaTx);
        chThdSuspendS(&ThdRef);    // Wait IRQ
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    WaitBTF();
    SendStop();
    return OK;
}

uint8_t i2c_t::Write(uint8_t Addr, uint8_t *WPtr1, uint8_t WLength1) {
    if(IBusyWait() != OK) return FAILURE;
    // Clear flags
    ii2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)ii2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != OK) return FAILURE;
    SendAddrWithWrite(Addr);
    if(WaitEv6() != OK) { SendStop(); return FAILURE; }
    ClearAddrFlag();
    // Start TX DMA if needed
    if(WLength1 != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr1);
        dmaStreamSetMode   (PDmaTx, I2C_DMATX_MODE);
        dmaStreamSetTransactionSize(PDmaTx, WLength1);
        chSysLock();
        dmaStreamEnable(PDmaTx);
        chThdSuspendS(&ThdRef);    // Wait IRQ
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    WaitBTF();
    SendStop();
    return OK;
}

void i2c_t::BusScan() {
    Uart.Printf("\r     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    uint8_t AddrHi, Addr;
    for(AddrHi = 0; AddrHi < 0x80; AddrHi += 0x10) {
        Uart.Printf("\r%02X: ", AddrHi);
        for(uint8_t n=0; n<0x10; n++) {
            Addr = AddrHi + n;
            if(Addr <= 0x01 or Addr > 0x77) Uart.Printf("   ");
            else {
                // Try to get response from addr
                if(IBusyWait() != OK) return;
                // Clear flags
                ii2c->SR1 = 0;
                while(RxIsNotEmpty()) (void)ii2c->DR;   // Read DR until it empty
                ClearAddrFlag();
                // Start transmission
                SendStart();
                if(WaitEv5() != OK) continue;
                SendAddrWithWrite(Addr);
                if(WaitEv6() == OK) Uart.Printf("%02X ", Addr);
                else Uart.Printf("__ ");
                SendStop();
            }
        } // for n
    } // for AddrHi
}

// ==== Flag operations ====
// Busy flag
uint8_t i2c_t::IBusyWait() {
    uint8_t RetryCnt = 4;
    while(RetryCnt--) {
        if(!(ii2c->SR2 & I2C_SR2_BUSY)) return OK;
        chThdSleepMilliseconds(1);
    }
    Error = true;
    return TIMEOUT;
}

// BUSY, MSL & SB flags
uint8_t i2c_t::WaitEv5() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--) {
        uint16_t Flag1 = ii2c->SR1;
        uint16_t Flag2 = ii2c->SR2;
        if((Flag1 & I2C_SR1_SB) and (Flag2 & (I2C_SR2_MSL | I2C_SR2_BUSY))) return OK;
    }
    Error = true;
    return FAILURE;
}

uint8_t i2c_t::WaitEv6() {
    uint32_t RetryCnt = 45;
    uint16_t Flag1;
    do {
        Flag1 = ii2c->SR1;
        if((RetryCnt-- == 0) or (Flag1 & I2C_SR1_AF)) return FAILURE;   // Fail if timeout or NACK
    } while(!(Flag1 & I2C_SR1_ADDR)); // ADDR set when Address is sent and ACK received
    return OK;
}

uint8_t i2c_t::WaitEv8() {
    uint32_t RetryCnt = 45;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_TXE) return OK;
    Error = true;
    return TIMEOUT;
}

uint8_t i2c_t::WaitRx() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_RXNE) return OK;
    return TIMEOUT;
}

uint8_t i2c_t::WaitStop() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->CR1 & I2C_CR1_STOP) return OK;
    return TIMEOUT;
}

uint8_t i2c_t::WaitBTF() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_BTF) return OK;
    return TIMEOUT;
}
#endif

#ifdef FLASH_LIB_KL // ==================== FLASH & EEPROM =====================
// Here not-fast write is used. I.e. interface will erase the word if it is not the same.
uint8_t Eeprom_t::Write32(uint32_t Addr, uint32_t W) {
    Addr += EEPROM_BASE_ADDR;
//    Uart.Printf("EAdr=%u\r", Addr);
    UnlockEE();
    // Wait for last operation to be completed
    uint8_t status = WaitForLastOperation();
    if(status == OK) {
        *(volatile uint32_t*)Addr = W;
        status = WaitForLastOperation();
    }
    LockEE();
    return status;
}

void Eeprom_t::ReadBuf(void *PDst, uint32_t Sz, uint32_t Addr) {
    uint32_t *p32 = (uint32_t*)PDst;
    Sz = Sz / 4;  // Size in words32
    while(Sz--) {
        *p32 = Read32(Addr);
        p32++;
        Addr += 4;
    }
}

uint8_t Eeprom_t::WriteBuf(void *PSrc, uint32_t Sz, uint32_t Addr) {
    uint32_t *p32 = (uint32_t*)PSrc;
    Addr += EEPROM_BASE_ADDR;
    Sz = (Sz + 3) / 4;  // Size in words32
    UnlockEE();
    // Wait for last operation to be completed
    uint8_t status = WaitForLastOperation();
    while((status == OK) and (Sz > 0))  {
        *(volatile uint32_t*)Addr = *p32;
        status = WaitForLastOperation();
        p32++;
        Addr += 4;
        Sz--;
    }
    LockEE();
    return status;
}

#endif

namespace Convert { // ============== Conversion operations ====================
void U16ToArrAsBE(uint8_t *PArr, uint16_t N) {
    uint8_t *p8 = (uint8_t*)&N;
    *PArr++ = *(p8 + 1);
    *PArr   = *p8;
}
void U32ToArrAsBE(uint8_t *PArr, uint32_t N) {
    uint8_t *p8 = (uint8_t*)&N;
    *PArr++ = *(p8 + 3);
    *PArr++ = *(p8 + 2);
    *PArr++ = *(p8 + 1);
    *PArr   = *p8;
}
uint16_t ArrToU16AsBE(uint8_t *PArr) {
    uint16_t N;
    uint8_t *p8 = (uint8_t*)&N;
    *p8++ = *(PArr + 1);
    *p8 = *PArr;
    return N;
}
uint32_t ArrToU32AsBE(uint8_t *PArr) {
    uint32_t N;
    uint8_t *p8 = (uint8_t*)&N;
    *p8++ = *(PArr + 3);
    *p8++ = *(PArr + 2);
    *p8++ = *(PArr + 1);
    *p8 = *PArr;
    return N;
}
void U16ChangeEndianness(uint16_t *p) { *p = __REV16(*p); }
uint8_t TryStrToUInt32(char* S, uint32_t *POutput) {
    if(*S == '\0') return EMPTY;
    char *p;
    *POutput = strtoul(S, &p, 0);
    return (*p == 0)? OK : NOT_A_NUMBER;
}
uint8_t TryStrToInt32(char* S, int32_t *POutput) {
    if(*S == '\0') return EMPTY;
    char *p;
    *POutput = strtol(S, &p, 0);
    return (*p == '\0')? OK : NOT_A_NUMBER;
}

uint16_t BuildUint16(uint8_t Lo, uint8_t Hi) {
    uint16_t r = Hi;
    r <<= 8;
    r |= Lo;
    return r;
}

uint32_t BuildUint32(uint8_t Lo, uint8_t MidLo, uint8_t MidHi, uint8_t Hi) {
    uint32_t r = Hi;
    r <<= 8;
    r |= MidHi;
    r <<= 8;
    r |= MidLo;
    r <<= 8;
    r |= Lo;
    return r;
}

// ==== Float ====
uint8_t TryStrToFloat(char* S, float *POutput) {
    if(*S == '\0') return EMPTY;
    char *p;
    *POutput = strtof(S, &p);
    return (*p == '\0')? OK : NOT_A_NUMBER;
}
}; // namespace

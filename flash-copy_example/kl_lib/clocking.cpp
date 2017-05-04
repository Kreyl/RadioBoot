/*
 * clocking.cpp
 *
 *  Created on: 20.01.2013
 *      Author: kreyl
 */

#include "clocking.h"
#include "stm32_rcc.h"
#include "kl_lib.h"
#include "uart.h"

Clk_t Clk;

#define CLK_STARTUP_TIMEOUT     200007

#if defined STM32L1XX
// ==== Inner use ====
uint8_t Clk_t::EnableHSE() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return 0;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::EnableHSI() {
    RCC->CR |= RCC_CR_HSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSIRDY) return 0;   // HSI is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::EnablePLL() {
    RCC->CR |= RCC_CR_PLLON;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return 0;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::EnableMSI() {
    RCC->CR |= RCC_CR_MSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_MSIRDY) return 0;   // MSI is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

void Clk_t::UpdateFreqValues() {
    uint32_t tmp, PllMul, PllDiv;
    uint32_t SysClkHz;
    // Tables
    const uint32_t MSIClk[8] = {65536, 131072, 262144, 524188, 1048000, 2097000, 4194000};
    const uint8_t PllMulTable[9] = {3, 4, 6, 8, 12, 16, 24, 32, 48};
    const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    // Figure out SysClk
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    tmp >>= 2;
    switch(tmp) {
        case 0b00: // MSI
            tmp = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> 13;
            SysClkHz = MSIClk[tmp];
            break;

        case 0b01: // HSI
            SysClkHz = HSI_FREQ_HZ;
            break;

        case 0b10: // HSE
            SysClkHz = CRYSTAL_FREQ_HZ;
            break;

        case 0b11: // PLL used as system clock source
            // Get different PLL dividers
            tmp = (RCC->CFGR & RCC_CFGR_PLLMUL) >> 18;
            PllMul = PllMulTable[tmp];
            PllDiv = ((RCC->CFGR & RCC_CFGR_PLLDIV) >> 22) +1;
            // Which src is used as pll input?
            SysClkHz = ((RCC->CFGR & RCC_CFGR_PLLSRC) == RCC_CFGR_PLLSRC_HSI)? HSI_FREQ_HZ : CRYSTAL_FREQ_HZ;
            SysClkHz = (SysClkHz * PllMul) / PllDiv;
            break;
    } // switch

    // AHB freq
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    AHBFreqHz = SysClkHz >> tmp;
    // APB freq
    uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
    uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 8;
    tmp = APBPrescTable[APB1prs];
    APB1FreqHz = AHBFreqHz >> tmp;
    tmp = APBPrescTable[APB2prs];
    APB2FreqHz = AHBFreqHz >> tmp;
    // Timer multi
    Timer2_7ClkMulti = (APB1prs < 4)? 1 : 2;
    Timer9_11ClkMulti = (APB2prs < 4)? 1 : 2;
}

// ==== Common use ====
// AHB, APB
void Clk_t::SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div) {
    // Setup dividers
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);  // Clear bits
    tmp |= ((uint32_t)AHBDiv)  << 4;
    tmp |= ((uint32_t)APB1Div) << 8;
    tmp |= ((uint32_t)APB2Div) << 11;
    RCC->CFGR = tmp;
}

// Enables HSI, switches to HSI
uint8_t Clk_t::SwitchToHSI() {
    if(EnableHSI() != 0) return 1;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |=  RCC_CFGR_SW_HSI;  // Select HSI as system clock src
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Wait till ready
    return 0;
}

// Enables HSE, switches to HSE
uint8_t Clk_t::SwitchToHSE() {
    if(EnableHSE() != 0) return 1;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |=  RCC_CFGR_SW_HSE;  // Select HSE as system clock src
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE); // Wait till ready
    return 0;
}

// Enables HSE, enables PLL, switches to PLL
uint8_t Clk_t::SwitchToPLL() {
    if(EnableHSE() != 0) return 1;
    if(EnablePLL() != 0) return 2;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |=  RCC_CFGR_SW_PLL;      // Select PLL as system clock src
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until ready
    return 0;
}

// Enables MSI, switches to MSI
uint8_t Clk_t::SwitchToMSI() {
    if(EnableMSI() != 0) return 1;
    uint32_t tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_SW;
    tmp |=  RCC_CFGR_SW_MSI;      // Select MSI as system clock src
    RCC->CFGR = tmp;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI); // Wait until ready
    return 0;
}

// Disable PLL first!
// HsePreDiv: 1...16; PllMul: pllMul[]
uint8_t Clk_t::SetupPLLMulDiv(PllMul_t PllMul, PllDiv_t PllDiv) {
    if(RCC->CR & RCC_CR_PLLON) return 1;    // PLL must be disabled to change dividers
    uint32_t tmp = RCC->CFGR;
    tmp &= RCC_CFGR_PLLDIV | RCC_CFGR_PLLMUL;
    tmp |= ((uint32_t)PllDiv) << 22;
    tmp |= ((uint32_t)PllMul) << 18;
    tmp |= RCC_CFGR_PLLSRC_HSE;
    RCC->CFGR = tmp;
    return 0;
}

void Clk_t::SetupFlashLatency(uint8_t AHBClk_MHz) {
    FLASH->ACR |= FLASH_ACR_ACC64;  // Enable 64-bit access
    FLASH->ACR |= FLASH_ACR_PRFTEN; // May be written only when ACC64 is already set
    if(     ((VCore == vcore1V2) and (AHBClk_MHz > 2)) or
            ((VCore == vcore1V5) and (AHBClk_MHz > 8)) or
            ((VCore == vcore1V8) and (AHBClk_MHz > 16))
            ) {
        FLASH->ACR |= FLASH_ACR_LATENCY;
    }
    else FLASH->ACR &= ~FLASH_ACR_LATENCY;
}

//void Clk_t::SetupAdcClk(ADCDiv_t ADCDiv) {
//    uint32_t tmp = RCC->CFGR;
//    tmp &= ~RCC_CFGR_ADCPRE;
//    tmp |= (uint32_t)ADCDiv;
//    RCC->CFGR = tmp;
//}

void Clk_t::PrintFreqs() {
    Uart.Printf(
            "AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz; Tim 2...7 Multi=%u; Tim 9...11 Multi=%u\r",
            Clk.AHBFreqHz/1000000, Clk.APB1FreqHz/1000000, Clk.APB2FreqHz/1000000,
            Timer2_7ClkMulti, Timer9_11ClkMulti);
}

// =============================== V Core ======================================
VCore_t VCore;
void SetupVCore(VCore_t AVCore) {
    // PWR clock enable
    RCC->APB1ENR = RCC_APB1ENR_PWREN;
    // Core voltage setup
    while((PWR->CSR & PWR_CSR_VOSF) != 0); // Wait until regulator is stable
    uint32_t tmp = PWR->CR;
    tmp &= ~PWR_CR_VOS;
    tmp |= ((uint32_t)AVCore) << 11;
    PWR->CR = tmp;
    while((PWR->CSR & PWR_CSR_VOSF) != 0); // Wait until regulator is stable
    VCore = AVCore;
}

#elif defined STM32F0XX
#include "CRS_defins.h"
// ==== Inner use ====
uint8_t Clk_t::EnableHSE() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return 0;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::EnableHSI() {
    RCC->CR |= RCC_CR_HSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSIRDY) return 0;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::EnablePLL() {
    RCC->CR |= RCC_CR_PLLON;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return 0;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::EnableHSI48() {
    RCC->CR2 |= RCC_CR2_HSI48ON;
    for(volatile uint32_t i=0; i<999; i++); // Let it to stabilize. Otherwise program counter flies to space with Ozzy Osbourne
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR2 & RCC_CR2_HSI48RDY) return 0;   // Clock is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

void Clk_t::UpdateFreqValues() {
    uint32_t tmp, PllSrc, PreDiv, PllMul;
    uint32_t SysClkHz = HSI_FREQ_HZ;
    // Figure out SysClk
    tmp = (RCC->CFGR & RCC_CFGR_SWS) >> 2;
    switch(tmp) {
        case csHSI:   SysClkHz = HSI_FREQ_HZ; break;
        case csHSE:   SysClkHz = CRYSTAL_FREQ_HZ; break;
        case csPLL: // PLL used as system clock source
            // Get different PLL dividers
            PreDiv = (RCC->CFGR2 & RCC_CFGR2_PREDIV) + 1;
            PllMul = ((RCC->CFGR & RCC_CFGR_PLLMUL) >> 18) + 2;
            if(PllMul > 16) PllMul = 16;
            // Which src is used as pll input?
            PllSrc = RCC->CFGR & RCC_CFGR_PLLSRC;
            switch(PllSrc) {
                case RCC_CFGR_PLLSRC_HSI_DIV2:   SysClkHz = HSI_FREQ_HZ / 2; break;
                case RCC_CFGR_PLLSRC_HSI_PREDIV: SysClkHz = HSI_FREQ_HZ / PreDiv; break;
                case RCC_CFGR_PLLSRC_HSE_PREDIV: SysClkHz = CRYSTAL_FREQ_HZ / PreDiv; break;
                case RCC_CFGR_PLLSRC_HSI48_PREDIV: SysClkHz = HSI48_FREQ_HZ / PreDiv; break;
                default: break;
            }
            SysClkHz *= PllMul;
            break;

        case csHSI48: SysClkHz = HSI48_FREQ_HZ; break;
    } // switch

    // AHB freq
    const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    AHBFreqHz = SysClkHz >> tmp;
    // APB freq
    const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    tmp = APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE) >> 8];
    APBFreqHz = AHBFreqHz >> tmp;
    // Timer multi
    TimerClkMulti = (tmp == 0)? 1 : 2;
    // ==== Update prescaler in System Timer ====
    uint32_t Psc = (SYS_TIM_CLK / OSAL_ST_FREQUENCY) - 1;
    TMR_DISABLE(STM32_ST_TIM);          // Stop counter
    uint32_t Cnt = STM32_ST_TIM->CNT;   // Save current time
    STM32_ST_TIM->PSC = Psc;
    TMR_GENERATE_UPD(STM32_ST_TIM);
    STM32_ST_TIM->CNT = Cnt;            // Restore time
    TMR_ENABLE(STM32_ST_TIM);
}

// ==== Common use ====
// AHB, APB
void Clk_t::SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APBDiv) {
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE);  // Clear bits
    tmp |= ((uint32_t)AHBDiv)  << 4;
    tmp |= ((uint32_t)APBDiv) << 8;
    RCC->CFGR = tmp;
}
void Clk_t::SetupBusDividers(uint32_t Dividers) {
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE);  // Clear bits
    tmp |= Dividers;
    RCC->CFGR = tmp;
}

static inline uint8_t WaitSWS(uint32_t Desired) {
    uint32_t StartUpCounter=0;
    do {
        if((RCC->CFGR & RCC_CFGR_SWS) == Desired) return OK; // Done
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return TIMEOUT;
}

// Enables HSI, switches to HSI
uint8_t Clk_t::SwitchTo(ClkSrc_t AClkSrc) {
    uint32_t tmp = RCC->CFGR & ~RCC_CFGR_SW;
    switch(AClkSrc) {
        case csHSI:
            if(EnableHSI() != OK) return 1;
            RCC->CFGR = tmp | RCC_CFGR_SW_HSI;  // Select HSI as system clock src
            return WaitSWS(RCC_CFGR_SWS_HSI);
            break;

        case csHSE:
            if(EnableHSE() != OK) return 2;
            RCC->CFGR = tmp | RCC_CFGR_SW_HSE;  // Select HSE as system clock src
            return WaitSWS(RCC_CFGR_SWS_HSE);
            break;

        case csPLL:
            if(EnablePLL() != OK) return 3;
            RCC->CFGR = tmp | RCC_CFGR_SW_PLL; // Select PLL as system clock src
            return WaitSWS(RCC_CFGR_SWS_PLL);
            break;

        case csHSI48:
            if(EnableHSI48() != OK) return FAILURE;
            RCC->CFGR = tmp | RCC_CFGR_SW_HSI48;
            return WaitSWS(RCC_CFGR_SWS_HSI48);
            break;
    } // switch
    return FAILURE;
}

// Disable PLL first!
// HsePreDiv: 1...16; PllMul: pllMul[]
uint8_t Clk_t::SetupPLLDividers(uint8_t HsePreDiv, PllMul_t PllMul) {
    if(RCC->CR & RCC_CR_PLLON) return 1;    // PLL must be disabled to change dividers
    // Set HSE divider
    HsePreDiv--;
    if(HsePreDiv > 0x0F) HsePreDiv = 0x0F;
    uint32_t tmp = RCC->CFGR2;
    tmp &= ~RCC_CFGR2_PREDIV;
    tmp |= HsePreDiv;
    RCC->CFGR2 = tmp;
    // Setup PLL divider
    tmp = RCC->CFGR;
    tmp &= ~RCC_CFGR_PLLMUL;
    tmp |= ((uint32_t)PllMul) << 18;
    RCC->CFGR = tmp;
    return 0;
}

// Setup Flash latency depending on CPU freq. Page 60 of ref manual.
// Call after UpdateFreqValues.
void Clk_t::SetupFlashLatency(uint32_t FrequencyHz) {
    uint32_t tmp = FLASH->ACR;
    if(FrequencyHz <= 24000000) tmp &= ~FLASH_ACR_LATENCY;
    else tmp |= FLASH_ACR_LATENCY;
    FLASH->ACR = tmp;
}

void Clk_t::PrintFreqs() {
    Uart.Printf(
            "AHBFreq=%uMHz; APBFreq=%uMHz\r",
            Clk.AHBFreqHz/1000000, Clk.APBFreqHz/1000000);
}

void Clk_t::EnableCRS() {
    RCC->APB1ENR |= RCC_APB1ENR_CRSEN;      // Enable CRS clocking
    RCC->APB1RSTR |= RCC_APB1RSTR_CRSRST;   // }
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CRSRST;  // } Reset CRS
    // Configure Synchronization input
    // Clear SYNCDIV[2:0], SYNCSRC[1:0] & SYNCSPOL bits
    CRS->CFGR &= ~(CRS_CFGR_SYNCDIV | CRS_CFGR_SYNCSRC | CRS_CFGR_SYNCPOL);
    // Configure CRS prescaler, source & polarity
    CRS->CFGR |= (CRS_PRESCALER | CRS_SOURCE | CRS_POLARITY);
    // Configure Frequency Error Measurement
    CRS->CFGR &= ~(CRS_CFGR_RELOAD | CRS_CFGR_FELIM);
    CRS->CFGR |= (CRS_RELOAD_VAL | (CRS_ERROR_LIMIT << 16));
    // Adjust HSI48 oscillator smooth trimming
    CRS->CR &= ~CRS_CR_TRIM;
    CRS->CR |= (HSI48_CALIBRATN << 8);
    // Enable auto trimming
    CRS->CR |= CRS_CR_AUTOTRIMEN;
    // Setup USB clock source = HSI48
    RCC->CFGR3 &= ~RCC_CFGR3_USBSW;
    // Enable Frequency error counter
    CRS->CR |= CRS_CR_CEN;
}

void Clk_t::DisableCRS() {
    CRS->CR &= ~CRS_CR_CEN;
    RCC->APB1ENR &= ~RCC_APB1ENR_CRSEN;
    // Setup USB clock source = PLLCLK to allow switch off HSI48
    RCC->CFGR3 |= RCC_CFGR3_USBSW;
}

void Clk_t::SwitchToHsi48() {
    ISavedAhbDividers = GetAhbApbDividers();
//    Uart.PrintfNow("cr21=%X\r", RCC->CR2);
    IHsi48WasOn = IsHSI48On();
    chSysLock();
    SetupFlashLatency(48000000);
    SetupBusDividers(ahbDiv1, apbDiv1);
    if(!IHsi48WasOn) SwitchTo(csHSI48);  // Switch HSI48 on if was off
    UpdateFreqValues();
    chSysUnlock();
}

void Clk_t::SwitchToHsi() {
    chSysLock();
    SetupBusDividers(ISavedAhbDividers);
    if(!IHsi48WasOn) {    // Switch hsi48 off if was off
        SwitchTo(csHSI);
        DisableHSI48();
    }
    UpdateFreqValues();
    SetupFlashLatency(AHBFreqHz);
    chSysUnlock();
}

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {
    // Enable HSI. It is enabled by default, but who knows.
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    // SYSCFG clock enabled here because it is a multi-functional unit
    // shared among multiple drivers using external IRQs
    rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, 1);
}
#elif defined STM32F40_41xxx
#include "kl_lib.h"

// =================================== Clk =====================================
uint8_t Clk_t::HSEEnable() {
    RCC->CR |= RCC_CR_HSEON;    // Enable HSE
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSERDY) return 0;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::HSIEnable() {
    RCC->CR |= RCC_CR_HSION;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_HSIRDY) return 0;   // HSE is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

uint8_t Clk_t::PLLEnable() {
    RCC->CR |= RCC_CR_PLLON;
    // Wait until ready
    uint32_t StartUpCounter=0;
    do {
        if(RCC->CR & RCC_CR_PLLRDY) return 0;   // PLL is ready
        StartUpCounter++;
    } while(StartUpCounter < CLK_STARTUP_TIMEOUT);
    return 1; // Timeout
}

void Clk_t::LsiEnable() {
    RCC->CSR |= RCC_CSR_LSION;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);
}

void Clk_t::UpdateFreqValues() {
    uint32_t tmp, pllvco=0, InputDiv_M, Multi_N, SysDiv_P;
    uint32_t SysFreqHz;     // SYSCLK, 168 MHz max, used for Ethernet PTP clk
    // Figure out SysClk
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    switch(tmp) {
        case 0x04: // HSE
            SysFreqHz = CRYSTAL_FREQ_HZ;
            break;

        case 0x08: // PLL used as system clock source
            // Get different PLL dividers
            InputDiv_M = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
            Multi_N    = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
            SysDiv_P   = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1 ) * 2;
            // Calculate pll freq
            pllvco = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE)? CRYSTAL_FREQ_HZ : HSI_FREQ_HZ;
            pllvco = (pllvco / InputDiv_M) * Multi_N;
            SysFreqHz = pllvco / SysDiv_P;
            break;

        default: // HSI
            SysFreqHz = HSI_FREQ_HZ;
            break;
    } // switch

    // AHB freq
    const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    AHBFreqHz = SysFreqHz >> tmp;
    // APB freqs
    const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    tmp = APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> 10];
    APB1FreqHz = AHBFreqHz >> tmp;
    tmp = APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> 13];
    APB2FreqHz = AHBFreqHz >> tmp;
    // Timer clock multiplier: 1 if APB_divider==1, 2 otherwise
    TimerAPB1ClkMulti = (RCC->CFGR & RCC_CFGR_PPRE1_2)? 2 : 1;
    TimerAPB2ClkMulti = (RCC->CFGR & RCC_CFGR_PPRE2_2)? 2 : 1;

    // ==== USB and SDIO freq ====
    UsbSdioFreqHz = 0;      // Will be changed only in case of PLL enabled
    if(RCC->CR & RCC_CR_PLLON) {
        // Get different PLL dividers
        InputDiv_M = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
        Multi_N    = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
        uint32_t SysDiv_Q = (RCC->PLLCFGR & RCC_PLLCFGR_PLLQ) >> 24;
        // Calculate pll freq
        pllvco = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE)? CRYSTAL_FREQ_HZ : HSI_FREQ_HZ;
        pllvco = (pllvco / InputDiv_M) * Multi_N;
        if(SysDiv_Q >= 2) UsbSdioFreqHz = pllvco / SysDiv_Q;
    }
}

// ==== Common use ====
// AHB, APB1, APB2
void Clk_t::SetupBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div) {
    // Setup dividers
    uint32_t tmp = RCC->CFGR;
    tmp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);  // Clear bits
    tmp |= ((uint32_t)AHBDiv)  << 4;
    tmp |= ((uint32_t)APB1Div) << 10;
    tmp |= ((uint32_t)APB2Div) << 13;
    RCC->CFGR = tmp;
}

// Enables HSI, switches to HSI
uint8_t Clk_t::SwitchToHSI() {
    if(HSIEnable() != 0) return 1;
    RCC->CFGR &= ~RCC_CFGR_SW;      // }
    RCC->CFGR |=  RCC_CFGR_SW_HSI;  // } Select HSI as system clock src
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Wait till ready
    return 0;
}

// Enables HSE, switches to HSE
uint8_t Clk_t::SwitchToHSE() {
    if(HSEEnable() != 0) return 1;
    RCC->CFGR &= ~RCC_CFGR_SW;      // }
    RCC->CFGR |=  RCC_CFGR_SW_HSE;  // } Select HSE as system clock src
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE); // Wait till ready
    return 0;
}

// Enables HSE, enables PLL, switches to PLL
uint8_t Clk_t::SwitchToPLL() {
    if(HSEEnable() != 0) return 1;
    if(PLLEnable() != 0) return 2;
    RCC->CFGR &= ~RCC_CFGR_SW;          // }
    RCC->CFGR |=  RCC_CFGR_SW_PLL;      // } Select PLL as system clock src
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until ready
    return 0;
}

// Disable PLL first!
// InputDiv_M: 2...63;  Multi_N:  2...432;
// SysDiv_P: sd2,4,6,8; UsbDiv_Q: 2...15.
uint8_t Clk_t::SetupPLLDividers(uint8_t InputDiv_M, uint16_t Multi_N, PllSysDiv_P_t SysDiv_P, uint8_t UsbDiv_Q) {
    if(RCC->CR & RCC_CR_PLLON) return 1;    // PLL must be disabled to change dividers
    RCC->PLLCFGR =
            RCC_PLLCFGR_PLLSRC_HSE |        // Use only HSE as src
            ((uint32_t)InputDiv_M << 0) |
            ((uint32_t)Multi_N  << 6) |
            ((uint32_t)SysDiv_P << 16) |
            ((uint32_t)UsbDiv_Q << 24);
    return 0;
}

// Setup Flash latency depending on CPU freq and voltage. Page 54 of ref manual.
uint8_t Clk_t::SetupFlashLatency(uint8_t AHBClk_MHz, uint16_t Voltage_mV) {
    uint32_t tmp = FLASH->ACR;
    tmp &= ~FLASH_ACR_LATENCY;  // Clear Latency bits
    tmp |= FLASH_ACR_ICEN | FLASH_ACR_DCEN; // Enable instruction & data prefetch by ART
    if((2700 < Voltage_mV) and (Voltage_mV <= 3600)) {
        if     (AHBClk_MHz <=  30) tmp |= FLASH_ACR_LATENCY_0WS;
        else if(AHBClk_MHz <=  60) tmp |= FLASH_ACR_LATENCY_1WS;
        else if(AHBClk_MHz <=  90) tmp |= FLASH_ACR_LATENCY_2WS;
        else if(AHBClk_MHz <= 120) tmp |= FLASH_ACR_LATENCY_3WS;
        else if(AHBClk_MHz <= 150) tmp |= FLASH_ACR_LATENCY_4WS;
        else                       tmp |= FLASH_ACR_LATENCY_5WS;
    }
    else if((2400 < Voltage_mV) and (Voltage_mV <= 2700)) {
        if     (AHBClk_MHz <= 24) tmp |= FLASH_ACR_LATENCY_0WS;
        else if(AHBClk_MHz <= 48) tmp |= FLASH_ACR_LATENCY_1WS;
        else if(AHBClk_MHz <= 72) tmp |= FLASH_ACR_LATENCY_2WS;
        else if(AHBClk_MHz <= 96) tmp |= FLASH_ACR_LATENCY_3WS;
        else                      tmp |= FLASH_ACR_LATENCY_4WS;
    }
    else if((2100 < Voltage_mV) and (Voltage_mV <= 2400)) {
        if     (AHBClk_MHz <= 18) tmp |= FLASH_ACR_LATENCY_0WS;
        else if(AHBClk_MHz <= 36) tmp |= FLASH_ACR_LATENCY_1WS;
        else if(AHBClk_MHz <= 54) tmp |= FLASH_ACR_LATENCY_2WS;
        else if(AHBClk_MHz <= 72) tmp |= FLASH_ACR_LATENCY_3WS;
        else if(AHBClk_MHz <= 90) tmp |= FLASH_ACR_LATENCY_4WS;
        else if(AHBClk_MHz <=108) tmp |= FLASH_ACR_LATENCY_5WS;
        else                      tmp |= FLASH_ACR_LATENCY_6WS;
    }
    else if((1650 < Voltage_mV) and (Voltage_mV <= 2100)) {
        if     (AHBClk_MHz <= 16) tmp |= FLASH_ACR_LATENCY_0WS;
        else if(AHBClk_MHz <= 32) tmp |= FLASH_ACR_LATENCY_1WS;
        else if(AHBClk_MHz <= 48) tmp |= FLASH_ACR_LATENCY_2WS;
        else if(AHBClk_MHz <= 64) tmp |= FLASH_ACR_LATENCY_3WS;
        else if(AHBClk_MHz <= 80) tmp |= FLASH_ACR_LATENCY_4WS;
        else if(AHBClk_MHz <= 96) tmp |= FLASH_ACR_LATENCY_5WS;
        else if(AHBClk_MHz <=112) tmp |= FLASH_ACR_LATENCY_6WS;
        else                      tmp |= FLASH_ACR_LATENCY_7WS;
    }
    else return 1;

    FLASH->ACR = tmp;
    return 0;
}

void Clk_t::MCO1Enable(Mco1Src_t Src, McoDiv_t Div) {
    PinSetupAlterFunc(GPIOA, 8, omPushPull, pudNone, AF0, ps100MHz);
    RCC->CFGR &= ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);   // First, disable output and clear settings
    RCC->CFGR |= ((uint32_t)Src) | ((uint32_t)Div << 24);
}
void Clk_t::MCO1Disable() {
    PinSetupAnalog(GPIOA, 8);
    RCC->CFGR &= ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);
}
void Clk_t::MCO2Enable(Mco2Src_t Src, McoDiv_t Div) {
    PinSetupAlterFunc(GPIOC, 9, omPushPull, pudNone, AF0, ps100MHz);
    RCC->CFGR &= ~(RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE);   // First, disable output and clear settings
    RCC->CFGR |= ((uint32_t)Src) | ((uint32_t)Div << 27);
}
void Clk_t::MCO2Disable() {
    PinSetupAnalog(GPIOC, 9);
    RCC->CFGR &= ~(RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE);
}

void Clk_t::PrintFreqs() {
    Uart.Printf(
            "\rAHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz; TimMulti1=%u, TimMulti2=%u",
            Clk.AHBFreqHz/1000000, Clk.APB1FreqHz/1000000, Clk.APB2FreqHz/1000000,
            TimerAPB1ClkMulti, TimerAPB2ClkMulti);
}

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {
    RCC->APB1ENR = RCC_APB1ENR_PWREN;   // PWR clock enable
    PWR->CR = 0;                        // PWR initialization

    // Enable HSI. It is enabled by default, but who knows.
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    // SYSCFG clock enabled here because it is a multi-functional unit
    // shared among multiple drivers using external IRQs
    rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, 1);
}
#endif

/*
 * main.cpp
 *
 *  Created on: 26 дек. 2015 г.
 *      Author: Kreyl
 */

#include "main.h"
#include "usb_cdc.h"
#include "color.h"
#include "radio_lvl1.h"
#include "led.h"
#include "Sequences.h"

App_t App;

LedRGB_t Led { {LED_GPIO, LEDR_PIN, LED_TMR, LEDR_CHNL}, {LED_GPIO, LEDG_PIN, LED_TMR, LEDG_CHNL}, {LED_GPIO, LEDB_PIN, LED_TMR, LEDB_CHNL} };

int main(void) {
    // ==== Setup clock frequency ====
    Clk.EnablePrefetch();
    Clk.SetupBusDividers(ahbDiv2, apbDiv1);
    Clk.UpdateFreqValues();

    // Init OS
    halInit();
    chSysInit();
    App.InitThread();

    // ==== Init hardware ====
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP | SYSCFG_CFGR1_USART1RX_DMA_RMP;
    Uart.Init(115200, UART_GPIO, UART_TX_PIN, UART_GPIO, UART_RX_PIN);
    Uart.Printf("\r%S %S\r", APP_NAME, BUILD_TIME);
    Clk.PrintFreqs();

    Led.Init();

#if USB_ENABLED
    UsbCDC.Init();
    chThdSleepMilliseconds(45);
    // Enable HSI48
    chSysLock();
    while(Clk.SwitchTo(csHSI48) != OK) {
        Uart.PrintfI("Hsi48 Fail\r");
        chThdSleepS(MS2ST(207));
    }
    Clk.UpdateFreqValues();
    chSysUnlock();
    Clk.PrintFreqs();
    Clk.SelectUSBClock_HSI48();
    Clk.EnableCRS();
    UsbCDC.Connect();
#endif

    if(Radio.Init() != OK) {
        Led.StartSequence(lsqFailure);
        chThdSleepMilliseconds(2700);
    }
    else Led.StartSequence(lsqStart);

    // Main cycle
    App.ITask();
}

__attribute__ ((__noreturn__))
void App_t::ITask() {
    while(true) {
        uint32_t EvtMsk = chEvtWaitAny(ALL_EVENTS);

        if(EvtMsk & EVTMSK_USB_READY) {
            Uart.Printf("UsbReady\r");
            Led.StartSequence(lsqUSB);
        }

        if(EvtMsk & EVTMSK_USB_NEW_CMD) {
            OnCmd((Shell_t*)&UsbCDC);
            UsbCDC.SignalCmdProcessed();
        }
        if(EvtMsk & EVTMSK_UART_NEW_CMD) {
            OnCmd((Shell_t*)&Uart);
            Uart.SignalCmdProcessed();
        }
    } // while true
}

#if 1 // ======================= Command processing ============================
void App_t::OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    __unused int32_t dw32 = 0;  // May be unused in some configurations
//    PShell->Printf(">%S\r", PCmd->Name);
    Uart.Printf("%S\r", PCmd->Name);
//    UsbCDC.Printf("%S\r", PCmd->Name);
    // Handle command
    if(PCmd->NameIs("Ping")) {
        PShell->Ack(OK);
    }

//    else if(PCmd->NameIs("Set")) {
//        uint8_t Rslt = CMD_ERROR;
//        for(uint8_t i = 0; i < 3; i++) {
//            // Get array of params
//            if(PCmd->GetArray(LedParams[i].Arr, 5) == OK) {
////                LedParams[i].Print();
//                if(LedParams[i].Check(PShell) == OK) {
//                    Rslt = OK;
//                    uint8_t indx = LedParams[i].Indx - 1;
//                    chSysLock();
//                    LedWs.DesiredClr[indx].Set(LedParams[i].R, LedParams[i].G, LedParams[i].B);
//                    LedWs.SmoothValue[indx] = LedParams[i].Smooth;
//                    chSysUnlock();
//                }
//                else break;
//            }
//            else break;
//        } // for
//        if(Rslt == OK) LedWs.StartProcess();
//        PShell->Ack(Rslt);
//    }

    else PShell->Ack(CMD_UNKNOWN);
}
#endif

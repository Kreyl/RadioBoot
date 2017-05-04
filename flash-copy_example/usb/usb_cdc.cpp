/*
 * usb_cdc.cpp
 *
 *  Created on: 03 сент. 2015 г.
 *      Author: Kreyl
 */

#include "usb_cdc.h"
#include "usb.h"
#include "descriptors_cdc.h"
#include "main.h"

UsbCDC_t UsbCDC;

#if 1 // ========================== Endpoints ==================================
// ==== EP1 ====
static USBInEndpointState ep1instate;
static USBOutEndpointState ep1outstate;

// EP1 initialization structure (both IN and OUT).
static const USBEndpointConfig ep1config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,                   // setup_cb
    sduDataTransmitted,     // in_cb
    sduDataReceived,        // out_cb
    64,                     // in_maxsize
    64,                     // out_maxsize
    &ep1instate,            // in_state
    &ep1outstate,           // out_state
    2,                      // in_multiplier: Determines the space allocated for the TXFIFO as multiples of the packet size
    NULL                    // setup_buf: Pointer to a buffer for setup packets. Set this field to NULL for non-control endpoints
};

// ==== EP2 ====
static USBInEndpointState ep2instate;

// EP2 initialization structure (IN only).
static const USBEndpointConfig ep2config = {
    USB_EP_MODE_TYPE_INTR,
    NULL,
    sduInterruptTransmitted,
    NULL,
    16,
    0,
    &ep2instate,
    NULL,
    1,
    NULL
};
#endif

#if 1 // ============================ Events ===================================
static void SOFHandler(USBDriver *usbp) {
  osalSysLockFromISR();
  sduSOFHookI(&UsbCDC.SDU1);
  osalSysUnlockFromISR();
}

static void usb_event(USBDriver *usbp, usbevent_t event) {
    switch (event) {
        case USB_EVENT_RESET:
            return;
        case USB_EVENT_ADDRESS:
            return;
        case USB_EVENT_CONFIGURED:
            chSysLockFromISR();
            /* Enable the endpoints specified in the configuration.
            Note, this callback is invoked from an ISR so I-Class functions must be used.*/
            usbInitEndpointI(usbp, USBD2_DATA_IN_EP, &ep1config);
            usbInitEndpointI(usbp, USBD2_INTERRUPT_REQUEST_EP, &ep2config);

            sduConfigureHookI(&UsbCDC.SDU1);   // Resetting the state of the CDC subsystem
            App.SignalEvtI(EVTMSK_USB_READY);
            chSysUnlockFromISR();
            return;
        case USB_EVENT_SUSPEND:
            return;
        case USB_EVENT_WAKEUP:
            return;
        case USB_EVENT_STALLED:
            return;
    } // switch
}

#endif

#if 1  // ==== USB driver configuration ====
const USBConfig UsbCfg = {
    usb_event,          // This callback is invoked when an USB driver event is registered
    GetDescriptor,      // Device GET_DESCRIPTOR request callback
    sduRequestsHook,    // This hook allows to be notified of standard requests or to handle non standard requests
    SOFHandler          // Start Of Frame callback
};

// Serial over USB driver configuration
const SerialUSBConfig SerUsbCfg = {
    &USBD1,                     // USB driver to use
    USBD2_DATA_IN_EP,           // Bulk IN endpoint used for outgoing data transfer
    USBD2_DATA_OUT_EP,          // Bulk OUT endpoint used for incoming data transfer
    USBD2_INTERRUPT_REQUEST_EP  // Interrupt IN endpoint used for notifications
};
#endif

#if 1 // ========================== RX Thread ==================================
static THD_WORKING_AREA(waThdCDCRX, 128);
static THD_FUNCTION(ThdCDCRX, arg) {
    chRegSetThreadName("CDCRX");
    while (true) {
        if(UsbCDC.IsActive()) {
            msg_t m = UsbCDC.SDU1.vmt->get(&UsbCDC.SDU1);
            if(m > 0) {
//                UsbCDC.SDU1.vmt->put(&UsbCDC.SDU1, (uint8_t)m);   // repeat what was sent
                if(UsbCDC.Cmd.PutChar((char)m) == pdrNewCmd) {
                    chSysLock();
                    App.SignalEvtI(EVTMSK_USB_NEW_CMD);
                    chSchGoSleepS(CH_STATE_SUSPENDED); // Wait until cmd processed
                    chSysUnlock();  // Will be here when application signals that cmd processed
                }
            } // if >0
        } // if active
        else chThdSleepMilliseconds(540);
    } // while true
}

#endif

void UsbCDC_t::Init() {
    PinSetupAnalog(GPIOA, 11);
    PinSetupAnalog(GPIOA, 12);
    // Objects
    usbInit();
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &SerUsbCfg);
    // RX thread
    IPThd = chThdCreateStatic(waThdCDCRX, sizeof(waThdCDCRX), NORMALPRIO, ThdCDCRX, NULL);
}

void UsbCDC_t::Connect() {
    usbDisconnectBus(SerUsbCfg.usbp);
    chThdSleepMilliseconds(1008);
    usbStart(SerUsbCfg.usbp, &UsbCfg);
    usbConnectBus(SerUsbCfg.usbp);
}

static inline void uFPutChar(char c) {
    UsbCDC.SDU1.vmt->put(&UsbCDC.SDU1, c);
}

void UsbCDC_t::Printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    kl_vsprintf(uFPutChar, 0xFFFF, format, args);
    va_end(args);
}

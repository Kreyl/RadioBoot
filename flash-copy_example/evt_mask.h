/*
 * evt_mask.h
 *
 *  Created on: Apr 12, 2013
 *      Author: g.kruglov
 */

#pragma once
// Event masks
#define EVTMSK_UART_NEW_CMD     EVENT_MASK(1)

#define EVTMSK_USB_NEW_CMD      EVENT_MASK(9)
#define EVTMSK_USB_READY        EVENT_MASK(10)
#define EVTMSK_USB_SUSPEND      EVENT_MASK(11)

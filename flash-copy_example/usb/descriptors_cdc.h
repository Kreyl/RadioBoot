/*
 * descriptors_cdc.h
 *
 *  Created on: 03 сент. 2015 г.
 *      Author: Kreyl
 */

#ifndef USB_DESCRIPTORS_CDC_H_
#define USB_DESCRIPTORS_CDC_H_

// Endpoints to be used for USBD2
#define USBD2_DATA_IN_EP                1
#define USBD2_DATA_OUT_EP               1
#define USBD2_INTERRUPT_REQUEST_EP      2

// Endpoint Sizes for Full-Speed devices
#define EP0_SZ              64  // Control Endpoint must have a packet size of 64 bytes
#define EP_INTERRUPT_SZ     8   // Max size is 64 bytes
#define EP_BULK_SZ          64  // Max size is 64 bytes

#ifdef __cplusplus
extern "C" {
#endif
const USBDescriptor *GetDescriptor(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang);
#ifdef __cplusplus
}
#endif

#endif /* USB_DESCRIPTORS_CDC_H_ */

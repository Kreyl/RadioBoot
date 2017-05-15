/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2016, Alexey Kramarenko
    All rights reserved.
*/

#ifndef CONFIG_H
#define CONFIG_H

// =============================== LEDS ========================================
#define LED_WHITE_PIN                               B0
#define LED_BLUE_PIN                                B1

#define LED_BLINK_FREQ_HZ                           25
#define LED_BLINK_TOGGLE_TIMES                      6

#define LED_BLINK_TIM_REG                           TIM3
#define LED_BLINK_TIM                               TIM_3
#define LED_BLINK_TIM_CHANNEL                       TIM_CHANNEL1
#define LED_BLINK_IRQ_VECTOR                        TIM3_IRQn

// =============================== USB =========================================
#define USB_PORT_NUM                                USB_0
#define USBD_PROCESS_SIZE                           1200
#define USBD_PROCESS_PRIORITY                       150

// =============================== COMM ========================================
#define COMM_COMMAND_END                            0x0D

// ================================= DEBUG =====================================

#define DBG_CONSOLE                                 UART_1
#define DBG_CONSOLE_TX_PIN                          A9
#define DBG_CONSOLE_BAUD                            115200

#define TEST_ROUNDS                                 10000

#define APP_DEBUG                                   1

#if (APP_DEBUG)
#define APP_COMM_DEBUG                              1
#define APP_LED_DEBUG                               1
#define APP_RADIO_DEBUG                             1

#endif //  APP_DEBUG
#endif // CONFIG_H

/*
 * system_config.h
 *
 *  Created on: 4 θώνÿ 2017 γ.
 *      Author: RomaJam
 */

#ifndef STM32_CONFIG_H_
#define STM32_CONFIG_H_


//------------------------------ POWER -----------------------------------------------
//depends on hardware implementation
#define POWER_MANAGEMENT                        0
//disable only for power saving if no EXTI or remap is used
#define SYSCFG_ENABLED                          0
//save few bytes here
#define STM32_DECODE_RESET                      0
//0 meaning HSI. If not defined, 25MHz will be defined by default by ST lib
#define HSE_VALUE                               0
#define HSE_RTC_DIV                             0
#define HSE_BYPASS                              0
//0 meaning HSE
#define LSE_VALUE                               0
//STM32L0 || STM32L
#define MSI_RANGE                               0

//
#define PLL_MUL                                 8
#define PLL_DIV                                 4

//STM32F10X_CL only
// use PLL2 as clock source for main PLL. Set to 0 to disable
#define PLL2_DIV                                0
#define PLL2_MUL                                0

//STM32F2, STM32F4
#define PLL_M                                   0
#define PLL_N                                   0
#define PLL_P                                   0

#define STANDBY_WKUP                            0

//------------------------------ UART ------------------------------------------
//Use UART as default stdio
#define UART_STDIO                              1
//PIN_DEFAULT and PIN_UNUSED can be also set.
#define UART                                    UART_1
#define UART_TX_PIN                             A9
//#define UART_AF_NUMBER                          AF7
#define UART_AF_NUMBER                          AF4
#define UART_BAUD                               115200
#define UART_DATA_BITS                          8
#define UART_PARITY                             'N'
#define UART_STOP_BITS                          1

#endif /* STM32_CONFIG_H_ */

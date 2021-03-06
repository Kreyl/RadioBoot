/*
 * cc1101_config.h
 *
 *  Created on: 2 ���. 2017 �.
 *      Author: RomaJam
 */

#ifndef CC1101_CONFIG_H
#define CC1101_CONFIG_H

#include "config.h"
#include "cc1101_defines.h"
#include "../rexos/userspace/stm32/stm32_driver.h"

// DEBUG
#define CC1101_DEBUG                            1
#define CC1101_DEBUG_REQUESTS                   0
#define CC1101_DEBUG_ERRORS                     1
#define CC1101_DEBUG_FLOW                       0
#define CC1101_DEBUG_INFO                       0


// CONFIGURATION
#define CC1101_SPI                              SPI_1

#define CC1101_CS_PIN                           A4
#define CC1101_CLK_PIN                          A5
#define CC1101_MISO_PIN                         A6
#define CC1101_MOSI_PIN                         A7
#define CC1101_GDO0_PIN                         A3
#define CC1101_GDO2_PIN                         A2

#define CC1101_GDO0_EXTI_IRQ                    EXTI3_IRQn
#define CC1101_GDO2_EXTI_IRQ                    EXTI2_IRQn

#define CC1101_IO_SIZE                          64
#define CC1101_PROCESS_SIZE                     600
#define CC1101_PROCESS_PRIORITY                 201

// Bitrate
//#define CC1101_BITRATE_10K
//#define CC1101_BITRATE_38K4
//#define CC1101_BITRATE_100K
//#define CC1101_BITRATE_250K
#define CC1101_BITRATE_500K

// ================================== RF CONFIG ==================================
// All this is for 27.0 MHz crystal, and for 868 MHz carrier
// ============================ Common use values ==============================
#define CC_TX_FIFO_SIZE                         33
#define CC_RX_FIFO_SIZE                         32

// ===================== Frequency 868 MHz: RF Studio ==========================
#define CC_FREQ2_VALUE                          0x20        // Frequency control word, high byte.
#define CC_FREQ1_VALUE                          0x25        // Frequency control word, middle byte.
#define CC_FREQ0_VALUE                          0xED        // Frequency control word, low byte.

// =================================== Common ==================================

// Exponent of Channel Spacing
#define CC_MDMCFG1_CHANSPC_E    0x03

// Clear channel signal
#define CC_FIFOTHR_VALUE        0b00000111  // RX attenuation = 0; RXFIFO and TXFIFO thresholds: TX 33, RX 32

// Device address.
#define CC_ADDR_VALUE           0x01

// ========================= Bitrate-specific ==================================
#ifdef CC1101_BITRATE_10K
#define CC_FSCTRL1_VALUE    0x06        // Frequency synthesizer control: IF - RF studio
#define CC_FSCTRL0_VALUE    0x00        // Frequency synthesizer control: freq offset

#define CC_MDMCFG4_VALUE    0xC8        // Modem configuration: channel bandwidth
#define CC_MDMCFG3_VALUE    0x84        // Modem configuration.
#define CC_MDMCFG2_VALUE    0x13        // Filter, modulation format, Manchester coding, SYNC_MODE=011 => 30/32 sync word bits
#define CC_MDMCFG0_VALUE    0xE5        // Modem configuration.

#define CC_DEVIATN_VALUE    0x34        // Modem deviation setting - RF studio
#define CC_FREND1_VALUE     0x56        // Front end RX configuration - RF studio
#define CC_FREND0_VALUE     0x10        // Front end TX configuration.

#define CC_FOCCFG_VALUE     0x16        // Frequency Offset Compensation Configuration - RF studio
#define CC_BSCFG_VALUE      0x6C        // Bit synchronization Configuration - RF studio
//#define CC_AGCCTRL2_VALUE   0x03        // AGC control: All gain settings can be used, max gain, 33 dB magn target
#define CC_AGCCTRL2_VALUE   0x43        // AGC control: RF studio
#define CC_AGCCTRL1_VALUE   0x40        // AGC control: RF studio
#define CC_AGCCTRL0_VALUE   0x91        // AGC control: RF studio

#define CC_FSCAL3_VALUE     0xE9        // }
#define CC_FSCAL2_VALUE     0x2A        // }
#define CC_FSCAL1_VALUE     0x00        // }
#define CC_FSCAL0_VALUE     0x1F        // } Frequency synthesizer calibration: RF studio

#define CC_TEST2_VALUE      0x81        // Various test settings: RF studio
#define CC_TEST1_VALUE      0x35        // Various test settings: RF studio
#define CC_TEST0_VALUE      0x09        // Various test settings: RF studio

// ********************
#elif defined CC1101_BITRATE_38K4
#define CC_FSCTRL1_VALUE    0x06        // }
#define CC_FSCTRL0_VALUE    0x00        // } Frequency synthesizer control: RF studio, nothing to do here

#define CC_MDMCFG4_VALUE    0xCA        // }
#define CC_MDMCFG3_VALUE    0x75        // } Modem configuration: RF Studio, nothing to do here
#define CC_MDMCFG2_VALUE    0x13        // Filter, modulation format, Manchester coding, SYNC_MODE=011 => 30/32 sync word bits
#define CC_MDMCFG0_VALUE    0xF8        // Modem configuration: RF Studio, nothing to do here

#define CC_DEVIATN_VALUE    0x34        // Modem deviation setting - RF studio, nothing to do here
#define CC_FREND1_VALUE     0x56        // Front end RX configuration - RF studio, no docs, nothing to do
#define CC_FREND0_VALUE     0x10        // Front end TX configuration - RF studio, no docs, nothing to do

#define CC_FOCCFG_VALUE     0x16        // Frequency Offset Compensation - RF studio, some unknown reasons for settings
#define CC_BSCFG_VALUE      0x6C        // Bit synchronization Configuration - RF studio, some unknown reasons for settings
#define CC_AGCCTRL2_VALUE   0x03        // AGC control: 00 - all gain settings, 000 - max possible gain, 011 - target ampl=33dB
#define CC_AGCCTRL1_VALUE   0x40        // Generally, nothing interesting
#define CC_AGCCTRL0_VALUE   0x91        // AGC filter settings: RF studio, some unknown reasons for settings

#define CC_FSCAL3_VALUE     0xE9        // }
#define CC_FSCAL2_VALUE     0x2A        // }
#define CC_FSCAL1_VALUE     0x00        // }
#define CC_FSCAL0_VALUE     0x1F        // } Frequency synthesizer calibration: RF studio, nothing to do here

#define CC_TEST2_VALUE      0x81        // Various test settings: RF studio
#define CC_TEST1_VALUE      0x31        // Various test settings: RF studio
#define CC_TEST0_VALUE      0x09        // Various test settings: RF studio

// ********************
#elif defined CC1101_BITRATE_100K
#define CC_FSCTRL1_VALUE    0x08        // }
#define CC_FSCTRL0_VALUE    0x00        // } Frequency synthesizer control: RF studio, nothing to do here

#define CC_MDMCFG4_VALUE    0x5B        // }
#define CC_MDMCFG3_VALUE    0xE5        // } Modem configuration: RF Studio, nothing to do here
//#define CC_MDMCFG2_VALUE    0x11        // Filter, GFSK, no Manchester coding, SYNC_MODE=010 => 16/16 sync word bits
#define CC_MDMCFG2_VALUE    0x13        // Filter, GFSK, no Manchester coding, SYNC_MODE=011 => 30/32 sync word bits
#define CC_MDMCFG0_VALUE    0xE5        // Channel spacing mantissa. See exponent at MDMCFG1. RF studio.

#define CC_DEVIATN_VALUE    0x46        // Modem deviation setting: 46 kHz
#define CC_FREND1_VALUE     0xB6        // Front end RX configuration - RF studio, no docs, nothing to do
#define CC_FREND0_VALUE     0x10        // Front end TX configuration - RF studio, no docs, nothing to do

#define CC_FOCCFG_VALUE     0x1D        // Frequency Offset Compensation - RF studio, some unknown reasons for settings
#define CC_BSCFG_VALUE      0x1C        // Bit synchronization Configuration - RF studio, some unknown reasons for settings
#define CC_AGCCTRL2_VALUE   0xC7        // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
#define CC_AGCCTRL1_VALUE   0x00        // Generally, nothing interesting
#define CC_AGCCTRL0_VALUE   0xB2        // AGC filter settings: RF studio, some unknown reasons for settings

#define CC_FSCAL3_VALUE     0xEA        // }
#define CC_FSCAL2_VALUE     0x2A        // }
#define CC_FSCAL1_VALUE     0x00        // }
#define CC_FSCAL0_VALUE     0x1F        // } Frequency synthesizer calibration: RF studio, nothing to do here

#define CC_TEST2_VALUE      0x81        // Various test settings: RF studio
#define CC_TEST1_VALUE      0x35        // Various test settings: RF studio
#define CC_TEST0_VALUE      0x09        // Various test settings: RF studio

// ********************
#elif defined CC1101_BITRATE_250K
#define CC_FSCTRL1_VALUE    0x0C        // }
#define CC_FSCTRL0_VALUE    0x00        // } Frequency synthesizer control: RF studio, nothing to do here

#define CC_MDMCFG4_VALUE    0x2D        // }
#define CC_MDMCFG3_VALUE    0x2F        // } Modem configuration: RF Studio, nothing to do here
#define CC_MDMCFG2_VALUE    0x13        // Filter, modulation format, no Manchester coding, SYNC_MODE=011 => 30/32 sync word bits
//#define CC_MDMCFG2_VALUE    0x11        // Filter, modulation format, no Manchester coding, SYNC_MODE=001 => 15/16 sync word bits
//#define CC_MDMCFG2_VALUE    0x14        // Filter, modulation format, no Manchester coding, SYNC_MODE=100
#define CC_MDMCFG0_VALUE    0xE5        // Channel spacing mantissa. See exponent at MDMCFG1. RF studio.

#define CC_DEVIATN_VALUE    0x62        // Modem deviation setting - RF studio, nothing to do here
#define CC_FREND1_VALUE     0xB6        // Front end RX configuration - RF studio, no docs, nothing to do
#define CC_FREND0_VALUE     0x10        // Front end TX configuration - RF studio, no docs, nothing to do

#define CC_FOCCFG_VALUE     0x1D        // Frequency Offset Compensation - RF studio, some unknown reasons for settings
#define CC_BSCFG_VALUE      0x1C        // Bit synchronization Configuration - RF studio, some unknown reasons for settings
#define CC_AGCCTRL2_VALUE   0xC7        // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
#define CC_AGCCTRL1_VALUE   0x00        // Generally, nothing interesting
#define CC_AGCCTRL0_VALUE   0xB0        // AGC filter settings: RF studio, some unknown reasons for settings

#define CC_FSCAL3_VALUE     0xEA        // }
#define CC_FSCAL2_VALUE     0x2A        // }
#define CC_FSCAL1_VALUE     0x00        // }
#define CC_FSCAL0_VALUE     0x1F        // } Frequency synthesizer calibration: RF studio, nothing to do here

#define CC_TEST2_VALUE      0x88        // Various test settings: RF studio
#define CC_TEST1_VALUE      0x31        // Various test settings: RF studio
#define CC_TEST0_VALUE      0x09        // Various test settings: RF studio

#elif defined CC1101_BITRATE_500K // ================== 500k =======================
#define CC_FSCTRL1_VALUE    0x0E        // }
#define CC_FSCTRL0_VALUE    0x00        // } Frequency synthesizer control: RF studio, nothing to do here

#define CC_MDMCFG4_VALUE    0x0E        // }
#define CC_MDMCFG3_VALUE    0x2F        // } Modem configuration: RF Studio, nothing to do here
#define CC_MDMCFG2_VALUE    0x73        // Filter on, modulation format MSK, no Manchester, SYNC_MODE=011 => 30/32 sync word bits
#define CC_MDMCFG0_VALUE    0xFF        // Channel spacing mantissa. See exponent at MDMCFG1. RF studio.

#define CC_DEVIATN_VALUE    0x00        // Modem deviation setting - RF studio, nothing to do here
#define CC_FREND1_VALUE     0xB6        // Front end RX configuration - RF studio, no docs, nothing to do
#define CC_FREND0_VALUE     0x10        // Front end TX configuration - RF studio, no docs, nothing to do

#define CC_FOCCFG_VALUE     0x1D        // Frequency Offset Compensation - RF studio, some unknown reasons for settings
#define CC_BSCFG_VALUE      0x1C        // Bit synchronization Configuration - RF studio, some unknown reasons for settings
#define CC_AGCCTRL2_VALUE   0xC7        // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
#define CC_AGCCTRL1_VALUE   0x00        // Generally, nothing interesting
#define CC_AGCCTRL0_VALUE   0xB0        // AGC filter settings: RF studio, some unknown reasons for settings

#define CC_FSCAL3_VALUE     0xEA        // }
#define CC_FSCAL2_VALUE     0x2A        // }
#define CC_FSCAL1_VALUE     0x00        // }
#define CC_FSCAL0_VALUE     0x1F        // } Frequency synthesizer calibration: RF studio, nothing to do here

#define CC_TEST2_VALUE      0x88        // Various test settings: RF studio
#define CC_TEST1_VALUE      0x31        // Various test settings: RF studio
#define CC_TEST0_VALUE      0x09        // Various test settings: RF studio
#endif

// Rare use settings
#define CC_SYNC1_VALUE      0xD3
#define CC_SYNC0_VALUE      0x91

#define CC_CHANNR_VALUE     0x00        // Channel number.

#endif /* CC1101_CONFIG_H */

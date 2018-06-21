/*
 * cc1101_defines.h
 *
 *  Created on: 19 θώνÿ 2018 γ.
 *      Author: RLeonov
 */

#ifndef _CC1101_DEFINES_H_
#define _CC1101_DEFINES_H_
// ================================= BITS DEFENITION ===========================
// IOCFG2
#define CC_IOCFG2_GDO2_ACTIVE_HIGH              (0 << 6)
#define CC_IOCFG2_GDO2_ACTIVE_LOW               (1 << 6)
#define CC_IOCFG2_GDO2_CFG                      0x29        /* reset value 0x29 */

// IOCFG1
#define CC_IOCFG1_GDO_DS_HIGH                   (1 << 7)
#define CC_IOCFG1_GDO_DS_LOW                    (0 << 7)
#define CC_IOCFG1_GDO1_ACTIVE_LOW               (1 << 6)
#define CC_IOCFG1_GDO1_ACTIVE_HIGH              (0 << 6)
#define CC_IOCFG1_GDO1_CFG                      0x2E        /* reset value 0x2E */

// IOCFG0
#define CC_IOCFG0_TEMP_SENSOR_ENABLE            (1 << 7)
#define CC_IOCFG0_GDO0_ACTIVE_LOW               (1 << 6)
#define CC_IOCFG0_GDO0_ACTIVE_HIGH              (0 << 6)
#define CC_IOCFG0_GDO0_CFG                      0x3F        /* reset value 0x3F */

// GDO_CFG
/* Associated to the RX FIFO: Asserts when RX FIFO is filled at or
 *  above the RX FIFO threshold. De-asserts when RX FIFO is drained below the same threshold. */
#define CC_GDO_CFG_RX_FIFO_THRESHOLD            0x00
/* Associated to the RX FIFO: Asserts when RX FIFO is filled at or
 * above the RX FIFO threshold or the end of packet is reached. De-asserts when the RX FIFO is empty. */
#define CC_GDO_CFG_RX_FIFO_EMPTY                0x01
/* Associated to the TX FIFO: Asserts when the TX FIFO is filled at or
 *  above the TX FIFO threshold. De-asserts when the TX FIFO is below the same threshold. */
#define CC_GDO_CFG_TX_FIFO_THRESHOLD            0x02
/* Associated to the TX FIFO: Asserts when TX FIFO is full. De-asserts when the TX FIFO is drained
 * below the TX FIFO threshold. */
#define CC_GDO_CFG_TX_FIFO_EMPTY                0x03
/* Asserts when the RX FIFO has overflowed. De-asserts when the FIFO has been flushed. */
#define CC_GDO_CFG_RX_FIFO_OVERFLOWED           0x04
/* Asserts when the TX FIFO has underflowed. De-asserts when the FIFO is flushed. */
#define CC_GDO_CFG_TX_FIFO_UNDERFLOWED          0x05
/* Asserts when sync word has been sent / received, and de-asserts at the end of the packet.
 * In RX, the pin will also deassert when a packet is discarded due to address or maximum
 * length filtering or when the radio enters RXFIFO_OVERFLOW state.
 * In TX the pin will de-assert if the TX FIFO underflows. */
#define CC_GDO_CFG_TX_UNDERFLOW_RX_OVERFLOW     0x06
/* Asserts when a packet has been received with CRC OK.
 * De-asserts when the first byte is read from the RX FIFO. */
#define CC_GDO_CFG_RX_CRC_OK                    0x07
/* Preamble Quality Reached. Asserts when the PQI is above the programmed
 * PQT value. De-asserted when the chip reenters RX state (MARCSTATE=0x0D)
 * or the PQI gets below the programmed PQT value. */
#define CC_GDO_CFG_PQT_REACHED                  0x08
/* Clear channel assessment. High when RSSI level is below threshold
 * (dependent on the current CCA_MODE setting). */
#define CC_GDO_CFG_CLEAR_CHANNEL                0x09
/* Lock detector output. The PLL is in lock if the lock detector output has a
 * positive transition or is constantly logic high. To check for PLL lock the
 * lock detector output should be used as an interrupt for the MCU. */
#define CC_GDO_CFG_LOCK_DETECTOR                0x0A
/* Serial Clock. Synchronous to the data in synchronous serial mode.
 * In RX mode, data is set up on the falling edge by CC1101 when GDOx_INV=0.
 * In TX mode, data is sampled by CC1101 on the rising edge of the serial clock when GDOx_INV=0. */
#define CC_GDO_CFG_SERIAL_CLOCK                 0x0B
/* Serial Synchronous Data Output. Used for synchronous serial mode. */
#define CC_GDO_CFG_SSDO_SYNC                    0x0C
/* Serial Data Output. Used for asynchronous serial mode. */
#define CC_GDO_CFG_SDO_ASYNC                    0x0D
/* Carrier sense. High if RSSI level is above threshold. Cleared when entering IDLE mode. */
#define CC_GDO_CFG_CARRIER_SENSE                0x0E
/* CRC_OK. The last CRC comparison matched. Cleared when entering/restarting RX mode. */
#define CC_GDO_CFG_LAST_CRC_MATCHING_OK         0x0F
/* RX_HARD_DATA[1]. Can be used together with RX_SYMBOL_TICK for alternative serial RX output. */
#define CC_GDO_CFG_RX_HARD_DATA_1               0x16
/* RX_HARD_DATA[0]. Can be used together with RX_SYMBOL_TICK for alternative serial RX output. */
#define CC_GDO_CFG_RX_HARD_DATA_0               0x17
/* PA_PD. Note: PA_PD will have the same signal level in SLEEP and TX states.
 * To control an external PA or RX/TX switch in applications where the SLEEP state is used
 * it is recommended to use GDOx_CFGx=0x2F instead.*/
#define CC_GDO_CFG_PA_PD                        0x1B
/* LNA_PD. Note: LNA_PD will have the same signal level in SLEEP and RX states.
 * To control an external LNA or RX/TX switch in applications where the SLEEP state is used
 * it is recommended to use GDOx_CFGx=0x2F instead. */
#define CC_GDO_CFG_LNA_PD                       0x1C
/* RX_SYMBOL_TICK. Can be used together with RX_HARD_DATA for alternative serial RX output. */
#define CC_GDO_CFG_RX_SYMBOL_TICK               0x1D
/* WOR_EVNT0 */
#define CC_GDO_CFG_WOR_EVENT0                   0x24
/* WOR_EVENT1 */
#define CC_GDO_CFG_WOR_EVENT1                   0x25
/* CLK 256 */
#define CC_GDO_CFG_CLK_256                      0x26
/* CLK_32K */
#define CC_GDO_CFG_CLK_32K                      0x27
/* CHIP_RDYn */
#define CC_GDO_CFG_CHIR_RDY                     0x29
/* XOSC_STABLE */
#define CC_GDO_CFG_XOSC_STABLE                  0x2B
/* High impedance (3-state) */
#define CC_GDO_CFG_HIZ                          0x2E
/* HW to 0 (HW1 achieved by setting GDOx_INV=1). Can be used to control an external LNA/PA or RX/TX switch. */
#define CC_GDO_CFG_HW                           0x2F
/* There are 3 GDO pins, but only one CLK_XOSC/n can be selected as an output at any
time. If CLK_XOSC/n is to be monitored on one of the GDO pins, the other two GDO pins must
be configured to values less than 0x30. The GDO0 default value is CLK_XOSC/192. */
#define CC_GDO_CFG_CLK_XOCS_DIV1                0x30
#define CC_GDO_CFG_CLK_XOCS_DIV1_5              0x31
#define CC_GDO_CFG_CLK_XOCS_DIV2                0x32
#define CC_GDO_CFG_CLK_XOCS_DIV3                0x33
#define CC_GDO_CFG_CLK_XOCS_DIV4                0x34
#define CC_GDO_CFG_CLK_XOCS_DIV6                0x35
#define CC_GDO_CFG_CLK_XOCS_DIV8                0x36
#define CC_GDO_CFG_CLK_XOCS_DIV12               0x37
#define CC_GDO_CFG_CLK_XOCS_DIV16               0x38
#define CC_GDO_CFG_CLK_XOCS_DIV24               0x39
#define CC_GDO_CFG_CLK_XOCS_DIV32               0x3A
#define CC_GDO_CFG_CLK_XOCS_DIV48               0x3B
#define CC_GDO_CFG_CLK_XOCS_DIV64               0x3C
#define CC_GDO_CFG_CLK_XOCS_DIV96               0x3D
#define CC_GDO_CFG_CLK_XOCS_DIV128              0x3E
#define CC_GDO_CFG_CLK_XOCS_DIV192              0x3F

// MDMCFG2
#define CC_MDMCFG2_DEM_DCFILT_OFF               (0 << 7)
#define CC_MDMCFG2_DEM_DCFILT_ON                (1 << 7) /* Only for data rates <= 250 kBaud */
#define CC_MDMCFG2_MOD_FORMAT_2_FSK             (0 << 4)
#define CC_MDMCFG2_MOD_FORMAT_GFSK              (1 << 4)
#define CC_MDMCFG2_MOD_FORMAT_ASK_OOK           (3 << 4)
#define CC_MDMCFG2_MOD_FORMAT_4_FSK             (4 << 4)
#define CC_MDMCFG2_MOD_FORMAT_MSK               (7 << 4) /* Only for data rates above 26 kBaud */
#define CC_MDMCFG2_MANCHESTER_ENABLE            (1 << 3)

#define CC_MDMCFG2_SYNC_MODE_NO_PREAMBLE_SYNC   (0 << 0)
#define CC_MDMCFG2_SYNC_MODE_15_16_BITS         (1 << 0)
#define CC_MDMCFG2_SYNC_MODE_16_16_BITS         (2 << 0)
#define CC_MDMCFG2_SYNC_MODE_30_32_BITS         (3 << 0)
#define CC_MDMCFG2_SYNC_MODE_CS_THRESHOLD       (4 << 0)
#define CC_MDMCFG2_SYNC_MODE_15_16_BITS_CS      (5 << 0)
#define CC_MDMCFG2_SYNC_MODE_16_16_BITS_CS      (6 << 0)
#define CC_MDMCFG2_SYNC_MODE_30_32_BITS_CS      (7 << 0)

// MDMCFG1
#define CC_MDMCFG1_FEC_ENABLE                   (1 << 7) /* Only supported for fixed packet length mode */
#define CC_DMDCFG1_FEC_DISABLE                  (0 << 7)
#define CC_MDMCFG1_PREAMBLE_BYTES_2             (0 << 4)
#define CC_MDMCFG1_PREAMBLE_BYTES_3             (1 << 4)
#define CC_MDMCFG1_PREAMBLE_BYTES_4             (2 << 4)
#define CC_MDMCFG1_PREAMBLE_BYTES_6             (3 << 4)
#define CC_MDMCFG1_PREAMBLE_BYTES_8             (4 << 4)
#define CC_MDMCFG1_PREAMBLE_BYTES_12            (5 << 4)
#define CC_MDMCFG1_PREAMBLE_BYTES_16            (6 << 4)
#define CC_MDMCFG1_PREAMBLE_BYTES_24            (7 << 4)

// MDMCFG0

// STATUS
#define CC_STATUS_CRC_OK                        (1 << 7)
#define CC_STATUS_CARIER_SENSE                  (1 << 6)
#define CC_STATUS_PQT_REACHED                   (1 << 5)
#define CC_STATUS_CHANNEL_CLEAR                 (1 << 4)
#define CC_STATUS_SFD                           (1 << 3)
#define CC_STATUS_GDO2                          (1 << 2)
#define CC_STATUS_GDO0                          (1 << 0)

// PKTCTRL0
#define CC_PKTCTRL0_WHITE_DATA                  (1 << 6)
#define CC_PKTCTRL0_PKT_FORMAT_NORMAL           (0 << 4)
#define CC_PKTCTRL0_PKT_FORMAT_SYNC             (1 << 4)
#define CC_PKTCTRL0_PKT_FORMAT_RANDOM_TX        (2 << 4)
#define CC_PKTCTRL0_PKT_FORMAT_ASYNC            (3 << 4)
#define CC_PKTCTRL0_CRC_ENABLE                  (1 << 2)
#define CC_PKTCTRL0_LENGTH_FIXED                (0 << 0)
#define CC_PKTCTRL0_LENGTH_VARIABLE             (1 << 0)
#define CC_PKTCTRL0_LENGTH_INFINITE             (2 << 0)
#define CC_PKTCTRL0_LENGTH_RESERVED             (3 << 0)

// PKTCTRL1
#define CC_PKTCTRL1_PQT_SYNC_WORD               (0 << 5)
#define CC_PKTCTRL1_CRC_AUTOFLUSH               (1 << 3)
#define CC_PKTCTRL1_APPEND_STATUS               (1 << 2)
#define CC_PKTCTRL1_ADDR_NOT_CHECK              (0 << 0)
#define CC_PKTCTRL1_ADDR_CHECK_NO_BROADCAST     (1 << 0)
#define CC_PKTCTRL1_ADDR_CHECK_0_BROADCAST      (2 << 0)
#define CC_PKTCTRL1_ADDR_CHECK_0_FF_BROADCAST   (3 << 0)

// =================================== Power ===================================
#define CC_PwrMinus30dBm                        0x03
#define CC_PwrMinus27dBm                        0x08
#define CC_PwrMinus25dBm                        0x0D
#define CC_PwrMinus20dBm                        0x17
#define CC_PwrMinus15dBm                        0x1D
#define CC_PwrMinus10dBm                        0x26
#define CC_PwrMinus6dBm                         0x37
#define CC_Pwr0dBm                              0x50
#define CC_PwrPlus5dBm                          0x86
#define CC_PwrPlus7dBm                          0xCD
#define CC_PwrPlus10dBm                         0xC5
#define CC_PwrPlus12dBm                         0xC0

// ======================= Registers, strobes etc. =============================
// Flags
#define CC_BURST_FLAG                           0b01000000
#define CC_WRITE_FLAG                           0b00000000
#define CC_READ_FLAG                            0b10000000

// Strobes
#define CC_SRES                                 0x30
#define CC_SFSTXON                              0x31
#define CC_SXOFF                                0x32
#define CC_SCAL                                 0x33
#define CC_SRX                                  0x34
#define CC_STX                                  0x35
#define CC_SIDLE                                0x36
#define CC_SWOR                                 0x38
#define CC_SPWD                                 0x39
#define CC_SFRX                                 0x3A
#define CC_SFTX                                 0x3B
#define CC_SWORRST                              0x3C
#define CC_SNOP                                 0x3D

// Status registers
#define CC_PARTNUM                              (0x30 | CC_BURST_FLAG)
#define CC_VERSION                              (0x31 | CC_BURST_FLAG)
#define CC_FREQEST                              (0x32 | CC_BURST_FLAG)
#define CC_LQI                                  (0x33 | CC_BURST_FLAG)
#define CC_RSSI                                 (0x34 | CC_BURST_FLAG)
#define CC_MARCSTATE                            (0x35 | CC_BURST_FLAG)
#define CC_WORTIME1                             (0x36 | CC_BURST_FLAG)
#define CC_WORTIME0                             (0x37 | CC_BURST_FLAG)
#define CC_PKTSTATUS                            (0x38 | CC_BURST_FLAG)
#define CC_VCO_VC_DAC                           (0x39 | CC_BURST_FLAG)
#define CC_TXBYTES                              (0x3A | CC_BURST_FLAG)
#define CC_RXBYTES                              (0x3B | CC_BURST_FLAG)
#define CC_RCCTRL1_STATUS                       (0x3C | CC_BURST_FLAG)
#define CC_RCCTRL0_STATUS                       (0x3D | CC_BURST_FLAG)

// States
#define CC_ST_SLEEP                             0
#define CC_ST_IDLE                              1
#define CC_ST_XOFF                              2
#define CC_ST_MANCAL3                           3
#define CC_ST_MANCAL4                           4
#define CC_ST_MANCAL5                           5
#define CC_ST_FS_WAKEUP6                        6
#define CC_ST_FS_WAKEUP7                        7
#define CC_ST_CALIBRATE8                        8
#define CC_ST_SETTLING9                         9
#define CC_ST_SETTLING10                        10
#define CC_ST_SETTLING11                        11
#define CC_ST_CALIBRATE12                       12
#define CC_ST_RX13                              13
#define CC_ST_RX14                              14
#define CC_ST_RX15                              15
#define CC_ST_TXRX_SETTLING                     16
#define CC_ST_RX_OVERFLOW                       17
#define CC_ST_FSTXON                            18
#define CC_ST_TX19                              19
#define CC_ST_TX20                              20
#define CC_ST_RXTX_SETTLING                     21
#define CC_ST_TX_UNDERFLOW                      22

// Status byte states
#define CC_STB_IDLE                             0x00
#define CC_STB_RX                               0x10
#define CC_STB_TX                               0x20
#define CC_STB_FSTXON                           0x30
#define CC_STB_CALIBRATE                        0x40
#define CC_STB_SETTLING                         0x50
#define CC_STB_RX_OVF                           0x60
#define CC_STB_TX_UNDF                          0x70

// Config registers addresses
#define CC_IOCFG2                               0x00
#define CC_IOCFG1                               0x01
#define CC_IOCFG0                               0x02
#define CC_FIFOTHR                              0x03
#define CC_SYNC1                                0x04
#define CC_SYNC0                                0x05
#define CC_PKTLEN                               0x06
#define CC_PKTCTRL1                             0x07
#define CC_PKTCTRL0                             0x08
#define CC_ADDR                                 0x09
#define CC_CHANNR                               0x0A
#define CC_FSCTRL1                              0x0B
#define CC_FSCTRL0                              0x0C
#define CC_FREQ2                                0x0D
#define CC_FREQ1                                0x0E
#define CC_FREQ0                                0x0F
#define CC_MDMCFG4                              0x10
#define CC_MDMCFG3                              0x11
#define CC_MDMCFG2                              0x12
#define CC_MDMCFG1                              0x13
#define CC_MDMCFG0                              0x14
#define CC_DEVIATN                              0x15
#define CC_MCSM2                                0x16
#define CC_MCSM1                                0x17
#define CC_MCSM0                                0x18
#define CC_FOCCFG                               0x19
#define CC_BSCFG                                0x1A
#define CC_AGCCTRL2                             0x1B
#define CC_AGCCTRL1                             0x1C
#define CC_AGCCTRL0                             0x1D
#define CC_WOREVT1                              0x1E
#define CC_WOREVT0                              0x1F
#define CC_WORCTRL                              0x20
#define CC_FREND1                               0x21
#define CC_FREND0                               0x22
#define CC_FSCAL3                               0x23
#define CC_FSCAL2                               0x24
#define CC_FSCAL1                               0x25
#define CC_FSCAL0                               0x26
#define CC_RCCTRL1                              0x27
#define CC_RCCTRL0                              0x28
#define CC_FSTEST                               0x29
#define CC_PTEST                                0x2A
#define CC_AGCTEST                              0x2B
#define CC_TEST2                                0x2C
#define CC_TEST1                                0x2D
#define CC_TEST0                                0x2E

#define CC_PATABLE                              0x3E

// FIFO
#define CC_FIFO                                 0x3F


#endif /* _CC1101_DEFINES_H_ */

/************************************************
*	  Manager for cc1101 rf chip
*    Copyright (C) 2015  Matthew Rentz
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>
*************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <linux/types.h>
#include <stdint.h>
#include "rpi_gpio.h"
#ifndef CC1101_H
#define CC1101_H
/***************************************************
*       Defines and Macros
*
****************************************************/
// Preprocessor directives for config settings
#define CC1101_865MHZ	1
#define CC1101_868MHZ	2
#define CC1101_915MHZ	3
#define CC1101_BASE_FREQ	CC1101_868MHZ
#define CC1101_CCA_ON	1 // Set to 1 to use CCA wait	

#define CC1101_TX_FIFO_SIZE		64


// Command Strobes--------------------------------
//      for descriptions see cc1101 manual pg 67

#define CC1101_STROBE_SRES      0x30    // Reset strobe
#define CC1101_STROBE_SFSTXON   0X31    // En and cal frq synth
#define CC1101_STROBE_SXOFF     0x32    // turn off osc
#define CC1101_STROBE_SCAL      0x33    // cal frq synth and turn off
#define CC1101_STROBE_SRX       0x34    // Enable RX, cal first if from IDLE & autocal=1
#define CC1101_STROBE_STX       0x35    // Enable TX
#define CC1101_STROBE_SIDLE     0x36    // Exit TX and RX
#define CC1101_STROBE_SWOR      0x38    // start RX wake on radio polling
#define CC1101_STROBE_SPWD      0x39    // power down when csn goes high
#define CC1101_STROBE_SFRX      0x3A    // flush RX buffer
#define CC1101_STROBE_SFTX      0x3B    // flush TX buffer
#define CC1101_STROBE_SWORRST   0x3C    // reset RTC to Event1 value
#define CC1101_STROBE_SNOP      0x3D    // NOP to get status byte

// Register address definitions----------------------

// Configuration Register Adresses
#define CC1101_IOCFG2           0x00
#define CC1101_IOCFG1           0x01
#define CC1101_IOCFG0           0x02
#define CC1101_FIFOTHR          0X03
#define CC1101_SYNC1            0X04
#define CC1101_SYNC0            0X05
#define CC1101_PKTLEN           0x06
#define CC1101_PKTCTRL1         0x07
#define CC1101_PKTCTRL0         0x08
#define CC1101_ADDR             0x09
#define CC1101_CHANNR           0x0A
#define CC1101_FSCTRL1          0x0B
#define CC1101_FSCTRL0          0x0C
#define CC1101_FREQ2            0x0D
#define CC1101_FREQ1            0x0E
#define CC1101_FREQ0            0x0F
#define CC1101_MDMCFG4          0x10
#define CC1101_MDMCFG3          0x11
#define CC1101_MDMCFG2          0x12
#define CC1101_MDMCFG1          0x13
#define CC1101_MDMCFG0          0x14
#define CC1101_DEVIATN          0x15
#define CC1101_MCSM2            0x16
#define CC1101_MCSM1            0x17
#define CC1101_MCSM0            0x18
#define CC1101_FOCCFG           0x19
#define CC1101_BSCFG            0x1A
#define CC1101_AGCTRL2          0x1B
#define CC1101_AGCTRL1          0x1C
#define CC1101_AGCTRL0          0x1D
#define CC1101_WOREVT1          0x1E
#define CC1101_WOREVT0          0x1F
#define CC1101_WORCTRL          0x20
#define CC1101_FREND1           0x21
#define CC1101_FREND0           0x22
#define CC1101_FSCAL3           0x23
#define CC1101_FSCAL2           0x24
#define CC1101_FSCAL1           0x25
#define CC1101_FSCAL0           0x26
#define CC1101_RCCTRL1          0x27
#define CC1101_RCCTRL0          0x28
#define CC1101_FSTEST           0x29
#define CC1101_PTEST            0x2A
#define CC1101_AGCTEST          0x2B
#define CC1101_TEST2            0x2C
#define CC1101_TEST1            0x2D
#define CC1101_TEST0            0x2E

// Status registers
#define CC1101_PARTNUM          0x30
#define CC1101_VERSION          0x31
#define CC1101_FREQUEST         0x32
#define CC1101_LQI              0x33
#define CC1101_RSSI             0x34
#define CC1101_MARCSTATE        0x35
#define CC1101_WORTIME1         0x36
#define CC1101_WORTIME0         0x37
#define CC1101_PKTSTATUS        0x38
#define CC1101_VCO_VC_DAC       0x39
#define CC1101_TXBYTES          0x3A
#define CC1101_RXBYTES          0x3B
#define CC1101_RCCTRL1_STATUS   0x3C
#define CC1101_RCCTRL0_STATUS   0x3D

//PATABLE
#define CC1101_PATABLE			0x3E

//FIFOs
#define CC1101_TX_FIFO			0x3F
#define CC1101_RX_FIFO			0x3F

// -----------------------------------------------------------------------------

// Configuration register field masks
// IOCFG2
#define CC1101_GDO2_INV                   0x40
#define CC1101_GDO2_CFG                   0x3F
// IOCFG1
#define CC1101_GDO1_DS                    0x80
#define CC1101_GDO1_INV                   0x40
#define CC1101_GDO1_CFG                   0x3F
// IOCFG0
#define CC1101_GDO0_TEMP_SENSOR_ENABLE    0x80
#define CC1101_GDO0_INV                   0x40
#define CC1101_GDO0_CFG                   0x3F
// FIFOTHR
#define CC1101_ADC_RETENTION              0x40
#define CC1101_CLOSE_IN_RX                0x30
#define CC1101_FIFO_THR                   0x0F
// SYNC1
#define CC1101_SYNC_MSB                   0xFF
// SYNC0
#define CC1101_SYNC_LSB                   0xFF
// PKTLEN
#define CC1101_PACKET_LENGTH              0xFF
// PKTCTRL1
#define CC1101_PQT                        0xE0
#define CC1101_CRC_AUTOFLUSH              0x08
#define CC1101_APPEND_STATUS              0x04
#define CC1101_ADR_CHK                    0x03
// PKTCTRL0
#define CC1101_WHITE_DATA                 0x40
#define CC1101_PKT_FORMAT                 0x30
#define CC1101_CRC_EN                     0x04
#define CC1101_LENGTH_CONFIG              0x03
// ADDR
#define CC1101_DEVICE_ADDR                0xFF
// CHANNR
#define CC1101_CHANNR_CHAN                0xFF
// FSCTRL1
#define CC1101_FREQ_IF                    0x1F
// FSCTRL0
#define CC1101_FREQOFF                    0xFF
// FREQ2
#define CC1101_FREQ_23_22                 0xC0
#define CC1101_FREQ_21_16                 0x3F
// FREQ1
#define CC1101_FREQ_15_8                  0xFF
// FREQ0
#define CC1101_FREQ_7_0                   0xFF
// MDMCFG4
#define CC1101_CHANBW_E                   0xC0
#define CC1101_CHANBW_M                   0x30
#define CC1101_DRATE_E                    0x0F
// MDMCFG3
#define CC1101_DRATE_M                    0xFF
// MDMCFG2
#define CC1101_DEM_DCFILT_OFF             0x80
#define CC1101_MOD_FORMAT                 0x70
#define CC1101_MANCHESTER_EN              0x08
#define CC1101_SYNC_MODE                  0x07
// MDMCFG1
#define CC1101_FEC_EN                     0x80
#define CC1101_NUM_PREAMBLE               0x70
#define CC1101_CHANSPC_E                  0x03
// MDMCFG0
#define CC1101_CHANSPC_M                  0xFF
// DEVIATN
#define CC1101_DEVIATION_E                0x70
#define CC1101_DEVIATION_M                0x07
// MCSM2
#define CC1101_RX_TIME_RSSI               0x10
#define CC1101_RX_TIME_QUAL               0x08
#define CC1101_RX_TIME                    0x07
// MCSM1
#define CC1101_CCA_MODE                   0x30
#define CC1101_RXOFF_MODE                 0x0C
#define CC1101_TXOFF_MODE                 0x03
// MCSM0
#define CC1101_FS_AUTOCAL                 0x30
#define CC1101_PO_TIMEOUT                 0x0C
#define CC1101_PIN_CTRL_EN                0x02
#define CC1101_XOSC_FORCE_ON              0x01
// FOCCFG
#define CC1101_FOC_BS_CS_GATE             0x20
#define CC1101_FOC_PRE_K                  0x18
#define CC1101_FOC_POST_K                 0x04
#define CC1101_FOC_LIMIT                  0x03
// BSCFG
#define CC1101_BS_PRE_K                   0xC0
#define CC1101_BS_PRE_KP                  0x30
#define CC1101_BS_POST_K                  0x08
#define CC1101_BS_POST_KP                 0x04
#define CC1101_BS_LIMIT                   0x03
// AGCCTRL2
#define CC1101_MAX_DVGA_GAIN              0xC0
#define CC1101_MAX_LNA_GAIN               0x38
#define CC1101_MAGN_TARGET                0x07
// AGCCTRL1
#define CC1101_AGC_LNA_PRIORITY           0x40
#define CC1101_CARRIER_SENSR_REL_THR      0x30
#define CC1101_CARRIER_SENSE_ABS_THR      0x0F
// AGCCTRL0
#define CC1101_HYST_LEVEL                 0xC0
#define CC1101_WAIT_TIME                  0x30
#define CC1101_AGC_FREEZE                 0x0C
#define CC1101_FILTER_LENGTH              0x03
// WOREVT1
#define CC1101_EVENT0_15_8                0xFF
// WOREVT0
#define CC1101_EVENT0_7_0                 0xFF
// WORCTRL
#define CC1101_RC_PD                      0x80
#define CC1101_EVENT1                     0x70
#define CC1101_RC_CAL                     0x08
#define CC1101_WOR_RES                    0x03
// FREND1
#define CC1101_LNA_CURRENT                0xC0
#define CC1101_LNA2MIX_CURRENT            0x30
#define CC1101_LODIV_BUF_CURRENT          0x0C
#define CC1101_MIX_CURRENT                0x03
// FREND0
#define CC1101_LODIV_BUF_CURRENT_TX       0x30
#define CC1101_PA_POWER                   0x07
// FSCAL3
#define CC1101_FSCAL3_7_6                 0xC0
#define CC1101_CHP_CURR_CAL_EN            0x30
#define CC1101_FSCAL3_3_0                 0x0F
// FSCAL2
#define CC1101_VCO_CORE_H_EN              0x20
#define CC1101_FSCAL2_7_0                 0x1F
// FSCAL1
#define CC1101_FSCAL1_7_0                 0x3F
// FSCAL0
#define CC1101_FSCAL0_7_0                 0x7F
// RCCTRL1
#define CC1101_RCCTRL_1                   0x7F
// RCCTRL0
#define CC1101_RCCTRL_0                   0x7F
// FSTEST
#define CC1101_FSTEST_7_0                 0xFF
// PTEST
#define CC1101_PTEST_7_0                  0xFF
// AGCTEST
#define CC1101_AGCTEST_7_0                0xFF
// TEST2
#define CC1101_TEST2_7_0                  0xFF
// TEST1
#define CC1101_TEST1_7_0                  0xFF
// TEST0
#define CC1101_TEST0_7_2                  0xFC
#define CC1101_VCO_SEL_CAL_EN             0x02
#define CC1101_TEST0_0                    0x01

// Status register field masks
// PARTNUM
#define CC1101_PARTNUM_7_0                0xFF
// VERSION
#define CC1101_VERSION_7_0                0xFF
// FREQOFF_EST
#define CC1101_FREQOFF_EST                0xFF
// LQI
#define CC1101_CRC_OK                     0x80u
#define CC1101_LQI_EST                    0x7Fu
// RSSI
#define CC1101_RSSI_7_0                   0xFFu
// MARC_STATE
#define CC1101_MARC_STATE                 0x1Fu
// WORTIME1
#define CC1101_TIME_15_8                  0xFFu
// WORTIME0
#define CC1101_TIME_7_0                   0xFFu
// PKTSTATUS
#define CC1101_PKTSTATUS_CRC_OK           0x80u
#define CC1101_PKTSTATUS_CS               0x40u
#define CC1101_PKTSTATUS_PQT_REACHED      0x20u
#define CC1101_PKTSTATUS_CCA              0x10u
#define CC1101_PKSTATUS_SFD               0x08u
#define CC1101_PKTSTATUS_GDO2             0x04u
#define CC1101_PKTSTATUS_GDO0             0x01u
// VCO_VC_DAC
#define CC1101_VCO_VC_DAC_7_0             0xFFu
// TXBYTES
#define CC1101_TXFIFO_UNDERFLOW           0x80u
#define CC1101_NUM_TXBYTES                0x7Fu
// RXBYTES
#define CC1101_RXFIFO_OVERFLOW            0x80u
#define CC1101_NUM_RXBYTES                0x7Fu
// RCCTRL1_STATUS
#define CC1101_RCCTRL1_STATUS_7_0         0xFFu
// RCCTRL0_STATUS
#define CC1101_RCCTRL0_STATUS_7_0         0xFFu

// Burst/single access masks
#define CC1101_WRITE_SINGLE               0x00u
#define CC1101_WRITE_BURST                0x40u
#define CC1101_READ_SINGLE                0x80u
#define CC1101_READ_BURST                 0xC0u

/*
 *	Status register state values
 */

#define CC1101_STATUS_IDLE		0x00
#define CC1101_STATUS_RX		0x01
#define CC1101_STATUS_TX		0x02
#define CC1101_STATUS_FSTXON		0x03
#define CC1101_STATUS_CALIBRATE		0x04
#define CC1101_STATUS_SETTL		0x05
#define CC1101_STATUS_RX_OVF		0x06
#define CC1101_STATUS_TX_OVF		0x07

// Output Power Settings
#if (CC1101_BASE_FREQ == CC1101_915MHZ)

#define CC1101_PWR_OUT_7DBM				0xC8

#elif (CC1101_BASE_FREQ == CC1101_868MHZ)

#define CC1101_PWR_OUT_7DBM				0xCC

#else

#define CC1101_PWR_OUT_7DBM				0xCC //Until further testing

#endif

// Macros for GD02 and CSn function
#define READ_GD02		(gpio_read(4))
#define CC1101_ENABLE()		(gpio_clear(5))
#define CC1101_DISABLE()    	(gpio_set(5))
#define CC1101_MISO_WAIT()	while(gpio_read(17))

/****************************************************
*       Function Declarations
****************************************************/

/**
 * Init function for cc1101. Performs a soft reset and programs all of the
 * registers based on the config structure. Also initializes the GPIO used
 * for controling the chip and configures and opens the spi
 * */
void cc1101_init();

/**
 * Read the current status of the cc1101
 *
 * @return	status	bits 7:4-status, bits 3:0 number of bytes in TX/RX FIFO
 */
char cc1101_read_status();

/**
 * reads and prints to screen the current settings for all config registers in hex
 */
void cc1101_print_config();

/**
 * Send strobe to cc1101
 *
 * @param strobe byte strobe command to send to cc1101
 * @param access how to access register write or read single or burst
 * @return status of chip
 */
char cc1101_strobe(char strobe, char access);

/**
 * write to single register
 *
 * @param config_reg Register where data will be written
 * @param setting Byte value to write to register
 * @return status of chip
 */
char cc1101_wr_reg(char config_reg, uint8_t setting);

/**
 * Read single byte from register
 *
 * @param reg Register addr to read
 * @param ret_buff Pointer to buffer for returned byte
 * @return status of chip
 */ 
char cc1101_rd_reg(char reg, uint8_t *ret_buff);

/**
 * write to block of registers
 *
 * @param start_reg First register where data will be written
 * @param in_buff Pointer to array of values to write to registers
 * @return status of chip
 */
char cc1101_wr_burst(uint8_t start_reg, uint8_t *in_buff, uint8_t count);

/**
 * Read block of registers
 *
 * @param start_reg First register to read
 * @param ret_buff Pointer to array to write read values
 * @return status of chip
 */
char cc1101_rd_burst(uint8_t start_reg, uint8_t *ret_buff, uint8_t count);

/**
 * Send data to transmit to cc1101. If CCA_ON macro set to 1
 * this function waits for clear channel before sending data.
 * TODO: Implement timeout error
 *
 * @param length # of data bytes to send incl addr (deflt max = 64 bytes)
 * @param address byte address to send message
 * @param data pointer to array of data bytes to send
 * @return status of chip
 */
void cc1101_send_msg(uint8_t length, uint8_t address, void * data);

/**
 * Put cc1101 into rx mode. Also sets the GDO2 to 0x07
 * goes high when message received with correct CRC
 */
void cc1101_start_rx();

/**
 * Busy wait for TX message to be sent
 * TODO: Timeout error
 */
void cc1101_tx_end_wait();

/**
 * close the spi dev file descriptor opened in cc1101_init()
 */
void cc1101_close();

#endif


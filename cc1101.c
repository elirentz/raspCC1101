/************************************************
 *	  Manager for cc1101 rf chip
 *
 *	  RTFM: www.ti.com/lit/ds/swrs061i.pdf
 *	
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
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "rpi_gpio.h"
#include "cc1101.h"
#include "spi.h"

//array of configuration values to write to the cc1101
static uint8_t cc1101_config_on_reset[0x29] = {
	//IOCFG2
	0x06,
	//IOCFG1
	0x2E,
	//IOCFG0
	0x2E,
	//FIFOTHR
	0x07,
	//SYNC1
	0xDC,
	//SYNC0
	0x12,
	//PKTLEN
	0x40,
	//PKTCTRL1
	0x0F,
	//PKTCTRL0
	0x45,
	//ADDR
	0x18,
	//CHANNR
	0x00,
	//FSCTRL1
	0x08,
	//FSCTRL0
	0x00,
#if (CC1101_BASE_FREQ == CC1101_915MHZ)
	//FREQ2
	0x22,
	//0x1E,
	//FREQ1
	0xEF,
	//0xC4,
	//FREQ0
	0x42,
	//0xEC,
#elif (CC1101_BASE_FREQ == CC1101_868MHZ)
	//FREQ2
	0x21,
	//FREQ1
	0x65,
	//FREQ0
	0x6A,
#elif (CC1101_BASE_FREQ == CC1101_868MHZ)
	//FREQ2
	0x21,
	//FREQ1
	0x47,
	//FREQ0
	0xE0,
#endif
	//MDMCFG4
	0X8A,		//203kHz
	//MDMCFG3
	0x83,		//38.383kHz data transfer rate
	//MDMCFG2
	0x92,		//GFSK w/o DC filter or Manchester; 16/16 sync words
	//MDMCFG1
	0x22,
	//MDMCFG0
	0xF8,		//Chanspc = ~200kHz default value
	//DEVIATN
	0x34,		// 19.042 kHz deviation
	//MCSM2
	0x07,		// default value no time out
	//MCSM1
	0x30,		// (default) cca_mode = (rssi < threshold) && (not rx packet)
	//	Idle state after packet RX and TX
	//MCSM0
	0x18,		// autocal IDLE -> (RX || TX); timeout = 64/F_CPU

#if (CC1101_BASE_FREQ == CC1101_915MHZ)
	//FOCCFG
	0x1F,		// Loop gain pre-sync = 4K, post-sync = K/2; Max offset = (+/-)BW(chan)/2
	//BSCFG
	0x1C,		// See cc1101 manual page 84
	//AGCCTRL2
	0xC7,
	//AGCCTRL1
	0x00,
	//AGCCTRL0
	0xB2,
#elif ((CC1101_BASE_FREQ == CC1101_868MHZ) || (CC1101_BASE_FREQ == CC1101_865MHZ))
	//FOCCFG
	0x16,		// Loop gain pre-sync = 4K, post-sync = K/2; Max offset = (+/-)BW(chan)/2
	//BSCFG
	0x6C,		// See cc1101 manual page 84
	//AGCCTRL2
	0x43,
	//AGCCTRL1
	0x38,
	//AGCCTRL0
	0x91,
#endif
	/**************************************************************************
	 * Wake on Radio settings
	 **************************************************************************/
	//WOREVT1
	0x87,		// default setting
	//WOREVT0
	0x6B,		// default setting
	//WORCTRL
	0xF8,		// default setting
	/**************************************************************************
	 * Front End Configuration
	 **************************************************************************/
#if (CC1101_BASE_FREQ == CC1101_915MHZ)
	//FREND1
	0Xb6,
#elif ((CC1101_BASE_FREQ == CC1101_868MHZ) || (CC1101_BASE_FREQ == CC1101_865MHZ))
	//FREND1
	0x56,
#endif
	//FREND0
	0x10,		// default setting
	/**************************************************************************
	 * Frequency Synthesizer Calibration
	 * *************************************************************************/
#if (CC1101_BASE_FREQ == CC1101_915MHZ)
	//FREND1
	//FSCAL3
	0xEA,
#elif ((CC1101_BASE_FREQ == CC1101_868MHZ) || (CC1101_BASE_FREQ == CC1101_865MHZ))
	//FSCAL3
	0xE9,
#endif
	//FSCAL2
	0x2A,
	//FSCAL1
	0x00,
	//FSCAL0
	0x1F,
	/*************************************************************************
	 * RC Oscillator Calibration
	 *************************************************************************/
	//RCCTRL1
	0x41,		// default value
	//RCCTRL0
	0x00,		// default value
};
/*const static char __flash cc1101_config_on_wake[6] = {
	//FSTEST
	0x59,
	//PTEST
	0x7F,
	//AGCTEST
	0x3F,
	//TEST2
	0x81,
	//TEST1
	0x35,
	//TEST0
	0x09,
};*/
int spi_fd;
// local function declarations
void cca_wait();

/************************************************************************/
/*	Manually initializes the T.I. CC1101 per the datasheet Section 19.1.2
 *		  |--50us-|
 * CSn		--|  |----| 
 *		  |__|    |_______
 *	                                                              */
/************************************************************************/
void cc1101_init()
{
	uint8_t tx;
	uint8_t rx[0x29];
	uint8_t zero_buf[0x29];
	memset(zero_buf, 0, 0x29);
	gpio_set_input(5);
	gpio_set_input(4);
	gpio_set_input(17);
	gpio_set_output(5);
	CC1101_DISABLE();	
	spi_fd = init_spi0();
	// Strobe CSn high then low with 
	// SCLK = 1 and SI = 0 to avoid issue (data-sheet section 11.3)
	CC1101_ENABLE();
	usleep(5);
	CC1101_DISABLE();
	CC1101_ENABLE();	
	usleep(10);
	CC1101_DISABLE();
	usleep(40);
	CC1101_ENABLE();
	CC1101_MISO_WAIT();
	// Send reset command and wait for SO to go low
	tx = CC1101_STROBE_SRES | CC1101_WRITE_SINGLE;
	spi0_send_byte(spi_fd, &tx, rx, 1);
	CC1101_MISO_WAIT();
	//send start address and write burst instruction
	tx = CC1101_STROBE_SIDLE | CC1101_WRITE_BURST;
	CC1101_ENABLE();
	CC1101_MISO_WAIT();
	spi0_send_byte(spi_fd, &tx, rx, 1);
	CC1101_DISABLE();
	tx = 0x00 | CC1101_WRITE_BURST;
	CC1101_ENABLE();
	spi0_send_byte(spi_fd, &tx, rx, 1 ); 
	//Send configuration settings
	spi0_send_byte(spi_fd, cc1101_config_on_reset, rx, 0x29);
	CC1101_DISABLE();
}

void cc1101_print_config()
{
	uint8_t tx;
	uint8_t rx[64];
	uint8_t zero_buf[64];

	tx = 0x00 | CC1101_READ_BURST;
	CC1101_ENABLE();
	spi0_send_byte(spi_fd, &tx, rx, 1 ); 
	spi0_send_byte(spi_fd, zero_buf, rx, sizeof(cc1101_config_on_reset));
	CC1101_DISABLE();
	int i;
	for(i = 0; i<sizeof(cc1101_config_on_reset); i++)
	{
		printf("%.02X\n\r", rx[i]); //Print out values in hex
	}
}



char cc1101_read_status()
{
	uint8_t status;
	uint8_t tx = CC1101_STROBE_SNOP;
	CC1101_ENABLE();
	CC1101_MISO_WAIT();
	spi0_send_byte(spi_fd, &tx, &status, 1);
	CC1101_DISABLE();
	return status;
}

char cc1101_strobe(char strobe, char access)
{
	uint8_t header = strobe | access;
	uint8_t status;
	CC1101_ENABLE();
	CC1101_MISO_WAIT();
	spi0_send_byte(spi_fd, &header, &status, 1);
	CC1101_DISABLE();
	return status;
}

char cc1101_wr_reg(char reg, uint8_t value)
{
	uint8_t header;
	uint8_t status;
	header = reg | CC1101_WRITE_SINGLE;
	CC1101_ENABLE();
	CC1101_MISO_WAIT();
	spi0_send_byte(spi_fd, &header, &status, 1);
	spi0_send_byte(spi_fd, &value, &status, 1);
	CC1101_DISABLE();
	return status;
}

char cc1101_wr_burst(uint8_t start_reg, uint8_t *in_buff, uint8_t count)
{
	uint8_t header;
	uint8_t status;
	uint8_t zero_buf[29];
	header = start_reg | CC1101_WRITE_BURST;
	CC1101_ENABLE();
	CC1101_MISO_WAIT();
	spi0_send_byte(spi_fd, &header, &status, 1);
	spi0_send_byte(spi_fd, in_buff, zero_buf, count);
	spi0_send_byte(spi_fd, in_buff, zero_buf, count);
	CC1101_DISABLE();
	return status;
}

char cc1101_rd_reg(char reg, uint8_t *ret_buff)
{
	uint8_t header;
	uint8_t status;
	uint8_t zero_buf[29];
	header = reg | CC1101_READ_SINGLE;
	CC1101_ENABLE();
	CC1101_MISO_WAIT();
	spi0_send_byte(spi_fd, &header, &status, 1);
	spi0_send_byte(spi_fd, zero_buf, ret_buff, 1);
	CC1101_DISABLE();
	return status;
}

char cc1101_rd_burst(uint8_t start_reg, uint8_t *ret_buff, uint8_t count)
{
	uint8_t header;
	uint8_t status;
	uint8_t zero_buf[29];
	memset(zero_buf, 0, 29);
	header = start_reg | CC1101_READ_BURST;
	CC1101_ENABLE();
	CC1101_MISO_WAIT();
	spi0_send_byte(spi_fd, &header, &status, 1);
	spi0_send_byte(spi_fd, zero_buf, ret_buff, count);
	CC1101_DISABLE();
	return status;
}
void cc1101_start_rx()
{
	cc1101_strobe(CC1101_STROBE_SFRX, CC1101_WRITE_SINGLE);
	cc1101_wr_reg(CC1101_IOCFG2, 0x07);
	cc1101_strobe(CC1101_STROBE_SRX, CC1101_WRITE_SINGLE);
}
void cc1101_close()
{
	spi0_close(spi_fd);
}

void cc1101_send_msg(uint8_t length, uint8_t address, void * data)
{
	char * p_data = (char *)data;
	uint8_t rf_buf[64];
	int i;

	if (length < CC1101_TX_FIFO_SIZE)
	{
		rf_buf[0] = length;
		rf_buf[1] = address;
		if (data != NULL)
		{
			for (i = 0; i < (length-1); i++) //length byte not included in length
			{
				rf_buf[i+2] = *p_data;
				p_data++;
			}
		}
		cc1101_strobe(CC1101_STROBE_SFTX, CC1101_WRITE_SINGLE); // clear tx fifo
#if (CC1101_CCA_ON)
		cca_wait();
#endif
		cc1101_strobe(CC1101_STROBE_STX, CC1101_WRITE_SINGLE);
		cc1101_wr_reg(CC1101_IOCFG2, 0x06); 			
		//Set GDO2 so it's 1 when sync sent and 0 after tx complete
		cc1101_wr_burst(CC1101_TX_FIFO, rf_buf, (length + 1));
	}
}
void cca_wait()
{
	cc1101_strobe(CC1101_STROBE_SFRX, CC1101_WRITE_SINGLE); // clear rx fifo
	//Change GDO2 output to CCA, logic high when channel clear
	cc1101_wr_reg(CC1101_IOCFG2, 0x09);			
	cc1101_strobe(CC1101_STROBE_SRX, CC1101_WRITE_SINGLE);
	while(!READ_GD02);
}
void cc1101_tx_end_wait()
{
	// wait for GDO2 to go high when sync word sent
	while(!READ_GD02);
	// wait for GDO2 to go low when packet finished
	while(READ_GD02);
	// End of Tx 
	printf("Message Sent.\n\r");
}	

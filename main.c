/************************************************
 *	Basic main for raspberry pi 2 cc1101 implementation	  
 *
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
 ************************************************/
#include <stdio.h>
#include <fcntl.h> 
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h> 
#include <unistd.h>
#include <stdlib.h>
#include "rpi_gpio.h"
#include "cc1101.h"
#include "spi.h"


int main(int argc, char * argv[])
{
	uint8_t rx[64];
	char buff[256];
	uint8_t length;
	uint8_t address;
	uint8_t status;
	int i;
	
	rpi_gpio_init();
	cc1101_init();
	status = cc1101_read_status();
	if (status != 0x0F)
	{
		cc1101_strobe(CC1101_STROBE_SIDLE, CC1101_WRITE_SINGLE);
	}
	printf("status ok\n\r");
	if(argc > 1)
	{
		printf("argv: %s\n\r", argv[1]);
		if(!strcmp(argv[1], "-w"))
		{
			if(strlen(argv[2]) == 2)
			{
				strcpy(buff, argv[2]);
			} else {
				printf("Adress too short\n\r");
				return 0;
			}
			// translate from ascii hex value to raw value
			for(i=0;i<2;i++)
			{
				if(buff[i] > 0x40 && buff[i] < 0x47)
				{
					address += (buff[i] - 55)<<((1-i)*4);
				}
				else if (buff[i] > 0x2F && buff[i] < 0x3A)
				{
					address += (buff[i]-0x30)<<((1-i)*4); //shift
				}
				else
				{
					printf("Not a hex number. %.02X\n\r", buff[i]);
					return 0;
				}
			}
			printf("Address: 0x%.02X\n\r", address);
			strcpy(buff, argv[3]);
			length = strlen(buff) + 1;
			printf("Length: %d\n\r", length);
			cc1101_send_msg(length, address, buff);
			cc1101_tx_end_wait();
		} else if (!strcmp(argv[1], "-r")) {
			cc1101_start_rx();
			printf("Rx mode started. Waiting...\n\r");
			while(length == 0)
			{
				do
				{
				}while(!READ_GD02);
				cc1101_rd_burst(CC1101_STROBE_SFTX, &length, 1); //read byte count RX_FIFO
			}
			cc1101_rd_burst(CC1101_RX_FIFO, rx,length);
			rx[length-2] = '\0';
			printf("Received Message:\n\r%s\n\r", &rx[2]);
		}

	} else {
		printf("Not enough arguments. Try again.\n\r");
	}
	return 1;
}

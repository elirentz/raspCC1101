/************************************************
 *	  Easy spi Driver using linux spidev
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
 *************************************************/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "spi.h"


static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000; // 500k bits/s
static uint16_t delay;
	
int init_spi0()
{

	int fd;
	int ret;
	
	fd = open(device, O_RDWR);
	if (fd < 0)
		printf("Can't open device");

	//Set Spi mode
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		printf("Can't enter SPI mode");
	
	//Set bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("Can't set bits per word");
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("Can't set bits per word");

	//Set max read and write speed
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printf("Can't set max write speed");
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printf("Can't set max read speed");
	return fd;
}

void spi0_send_byte(int fd, uint8_t * tx, uint8_t * rx, uint8_t count)
{

	int ret;
	struct spi_ioc_transfer spi_st = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = count,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &spi_st);
	if (ret< 1)
		printf("Can't send message");
}
void spi0_close(int fd)
{
	close(fd);
}
	

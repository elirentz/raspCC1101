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
#include <linux/types.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef SPI_H
#define SPI_H

/**
 * Initialize spidev0.0
 *
 * @return fd	file descriptor for spidev0.0
 */
int init_spi0();

/**
 * Send bytes over spi. CS pin is not controlled in this function
 * this allows this function to work for the entire bus connected to 
 * spidev0.0. 
 *
 * @param fd 	file descriptor for the spi device received in init_spi()
 * @param tx 	pointer to the array of bytes to send over bus
 * @param rx 	pointer to the array to return bytes received while transmitting
 * @param count	number of byte to send
 */
void spi0_send_byte(int fd, uint8_t * tx, uint8_t * rx, uint8_t count);

/**
 * Close spidev0.0 file descriptor
 */
void spi0_close(int fd);
#endif

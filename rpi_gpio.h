/*******************************************************
 * Description: Raspberry Pi peripheral mapping
 * 	and gpio macros for the Pi 2 using the BCM2836 
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
 * Authors: Pieter-Jan Van de Maele (www.pieter-jan.com/node/15),
 *	Matthew Rentz
 * 
 ******************************************************************/
#ifndef RPI_H
#define RPI_H 
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

 
 
 

void rpi_gpio_init();
void rpi_gpio_close();
void gpio_set_input(uint8_t gpio);
void gpio_set_output(uint8_t gpio);
uint32_t gpio_read(uint8_t gpio);
void gpio_set(uint8_t gpio);
void gpio_clear(uint8_t gpio);

// Setup gpio alternate function
/*#define SET_GPIO_ALT(g,a) *(gpio.addr + (((g)/10))) |= \
				(((a)<=3?(a) + 4:(a)==4?3:2)<<(((g)%10)*3))*/
 
#endif

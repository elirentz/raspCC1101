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
 * Authors: 	Matthew Rentz
 * 
 ******************************************************************/
#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <unistd.h>
#include "rpi_gpio.h"

#define BCM2709_PERI_BASE       0x3F000000	// Raspberry Pi 2 Base
#define GPIO_BASE               (BCM2709_PERI_BASE + 0x200000)	// GPIO controller 
 
#define BLOCK_SIZE 		(4*1024)

typedef struct {
    unsigned long addr_p;
    int mem_fd;
    void *map;
    volatile unsigned int *addr;
}bcm2836_peripheral_t;

static bcm2836_peripheral_t gpio = {GPIO_BASE}; 

int map_peripheral(bcm2836_peripheral_t *p);
void unmap_peripheral(bcm2836_peripheral_t *p);

void rpi_gpio_init()
{
	if(map_peripheral(&gpio) == -1)
	{
		printf("Fuck!!\n\rMap Failed\n\r");
	}
}
void rpi_gpio_close()
{
	unmap_peripheral(&gpio);
}

void gpio_set_input(uint8_t g)
{
	*(gpio.addr + ((g)/10)) &= ~(7 << (((g)%10) * 3));
}
void gpio_set_output(uint8_t g)
{
	//setup as input first
	*(gpio.addr + ((g)/10)) &= ~(7 << (((g)%10) * 3));
	*(gpio.addr + ((g)/10)) |=  (1 << (((g)%10) * 3));
}
uint32_t gpio_read(uint8_t g)
{
	return (*(gpio.addr + 13) & (1<<(g)));
}
void gpio_set(uint8_t g)
{

	(*(gpio.addr + 7) |= (1 << (g)));
}
void gpio_clear(uint8_t g)
{
	(*(gpio.addr + 10) |= (1 << (g)));
}

int map_peripheral(bcm2836_peripheral_t *p)
{
   // Open /dev/mem
   setuid(0);
   if ((p->mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("Failed to open /dev/mem, try checking permissions.\n");
      return -1;
   }
 
   p->map = mmap(	
      NULL,		//let linux set address
      BLOCK_SIZE,
      PROT_READ|PROT_WRITE,
      MAP_SHARED,
      p->mem_fd,      // File descriptor to physical memory virtual file '/dev/mem'
      p->addr_p       // Address in physical map that we want this memory block to expose
   );
	if(setuid(getuid()) < 0) _exit(1); 
	close(p->mem_fd);
   if (p->map == MAP_FAILED) {
        perror("mmap");
        return -1;
   }
 
   p->addr = (volatile unsigned int *)p->map;
 
   return 0;
}
 
void unmap_peripheral(bcm2836_peripheral_t *p) {
 
    munmap(p->map, BLOCK_SIZE);
}

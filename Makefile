
cc=gcc
cflags= -I../../projects/Flo_Rf_Protocol -Wall
raspCC1101: rpi_gpio.o spi.o cc1101.o main.o
	$(cc) $(cflags) -o raspCC1101 rpi_gpio.o spi.o cc1101.o main.o  
%.o: %.c
	$(cc) $(cflags) -c $< 
clean:
	rm -f raspCC1101

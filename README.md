#raspC1101
CC1101 control functions for the Raspberry Pi 2

Initializes gpio and spi bus for transceiver control.

The code is set up to use in the 868 MHz frequency range by default
but can use the 915 MHz range using a preprocessor variable in the cc1101.h file.

The default reset configuration can be used as well by setting another
preprocessor variable in the cc1101.h file

Gpio setup:
	--Pi--	  cc1101
	----------------
	-Gpio4 	<- GDO2
	-Gpio5 	-> CS/EN
	-Gpio17	<- MISO -jumpered from spi to avoid pin function change

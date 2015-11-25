#raspC1101
CC1101 control functions for the Raspberry Pi 2

Initializes gpio and spi bus for transceiver control.

The code is set up to use in the 868 MHz frequency range by default
but can use the 915 MHz range using a preprocessor variable in the cc1101.h file.

The default reset configuration can be used as well by setting another
preprocessor variable in the cc1101.h file

##Main functionality
There's a basic program in main.c that demonstrates the usage 
of some of the functions.
The command line usage is as follows:
* First option:
	* -r  Read. Puts the chip into receive mode prints message to screen.
	* -w  Write. Followed by the address in hex and then the message to send with no spaces.
```
$sudo ./raspCC1101 -w 18 Hello
```
This example sends "Hello" in ascii text via the CC1101

NOTE: See wiki for default gpio routing info. 

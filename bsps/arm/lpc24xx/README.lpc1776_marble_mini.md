# Support for LBL Marble Mini

# Initialization

Application(s) should call `marble_bsp_init()` from include/bsp/marble.h

# Board specific features

Symlinks for I2C and SPI bus symlinks.

* /dev/i2c-ipmb
* /dev/i2c-pm
* /dev/i2c-fpga
* /dev/spi-fpga
* /dev/spi-pmod3

SoC EEPROM access

* /dev/eeprom

GPIO definitions

* See MARBLE_* in include/bsp/marble.h
* marble_gpio_read() and marble_gpio_write()

## Status

[ ] Console

Needs verification

[ ] Clock

Currently configured to remain on internal 12MHz osc.
External osc. is 25 MHz?

[ ] I2C

Needs verification

[ ] SPI

Needs verification

[ ] MII MDIO

Add interface

[ ] EEPROM

Needs testing

[ ] GPIO

Need to confirm pin direction and need for internal pull up/down on outputs.

Abstraction API?

[ ] FMC JTAG

Needed?

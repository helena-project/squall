Squall
======

Squall is a low cost, 1 inch round BLE sensor tag based on the Nordic
nRF51822 BLE / Cortex M0 SoC. It is designed to be the basis for experimenting
with BLE tag platforms.

### Squall Versions

Squall comes in two flavors: a stripped down version with only the essentials
that has a raw cost just over $5. This includes the SoC, an antenna, expansion
headers, and a battery clip. This version is targeted towards sensor deployments
where a low cost per device is essential to permit larger deployments.

The second version adds USB support so that the nRF51822 can be programmed
with a serial bootloader, and adds a reset button and 32 kHz crystal. This
design is intended to be easier to get started with as it only requires
a USB connection to program. It also supports a rechargeable battery that
will charge over USB.

### Hardware Tools

To aid developing with Squall, there is a
[breakout board](/hardware/squall-breakout/rev_b) that makes it easier to debug
signals and program the board.

To program Squall over JTAG, there is an
[adapter board](https://github.com/lab11/nrf51-tools/tree/master/hardware/jlink_to_tag/rev_b)
that adapts the J-Link JTAG 20-pin header to the much smaller
[tag connect](http://www.tag-connect.com/catalog/6) header.


Daughter Boards
---------------

Squall is designed to be a base BLE platform with shields on top. A couple
shields have been developed:

- [Rain](/hardware/rain) - Sensing platform for wearable exploration.
- [BLEES](https://github.com/lab11/blees) - Environmental sensing platform.

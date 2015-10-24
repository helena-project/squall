Setting Up The nRF51822
=======================

This guide explains how to get going with running code on nRF51822 based
platforms.

Setup
=====

1. Make sure you have the [arm-none-eabi-gcc](https://launchpad.net/gcc-arm-embedded)
toolchain. You just need the binaries for your platform.

1. Get the [Segger flashing tools](http://www.segger.com/jlink-software.html)
for your platform.



Install an Application
======================


1. Now compile and load the application code. This will also
load the softdevice onto the nRF if needed.

        make flash

    For platforms that are not Squall:

        make flash TARGET=nrf6310

    Choices:

        NRF6310
        PCA10000
        PCA10001
        PCA10003
        SQUALL


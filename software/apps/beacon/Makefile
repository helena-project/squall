PROJECT_NAME = $(shell basename "$(realpath ./)")

APPLICATION_SRCS = $(notdir $(wildcard ./*.c))
APPLICATION_SRCS += softdevice_handler.c
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_debug_assert_handler.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += led.c

DEVICE = NRF51
BOARD = BOARD_SQUALL

USE_SOFTDEVICE = s110

SDK_PATH ?= $(HOME)/Downloads/nrf51822/
TEMPLATE_PATH ?= $(HOME)/git/nrf51-pure-gcc-setup/template/

LIBRARY_PATHS += ../../include
SOURCE_PATHS += ../../src

CFLAGS = -Os
GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile

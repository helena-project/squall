PROJECT_NAME = $(shell basename "$(realpath ./)")

APPLICATION_SRCS = $(notdir $(wildcard ./*.c))
APPLICATION_SRCS += softdevice_handler.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += led.c

DEVICE = NRF51

ifndef TARGET
	TARGET = SQUALL
endif

TARGET_UPPER = $(shell echo $(TARGET) | tr a-z A-Z)
BOARD = BOARD_$(TARGET_UPPER)

USE_SOFTDEVICE = s110

SDK_PATH ?= $(HOME)/code/nrf51822_sdk/sdk_6/nrf51822/
TEMPLATE_PATH ?= $(HOME)/git/nrf51-pure-gcc-setup/template/

LIBRARY_PATHS += ../../include
SOURCE_PATHS += ../../src

CFLAGS = -Os
GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile

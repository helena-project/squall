PROJECT_NAME = $(shell basename "$(realpath ./)")

APPLICATION_SRCS = $(notdir $(wildcard ./*.c))
APPLICATION_SRCS += softdevice_handler.c
APPLICATION_SRCS += ble_debug_assert_handler.c
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_dis.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += app_scheduler.c
APPLICATION_SRCS += ble_conn_params.c
APPLICATION_SRCS += app_error.c
APPLICATION_SRCS += app_util_platform.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += led.c
APPLICATION_SRCS += squall.c
APPLICATION_SRCS += twi_hw_master.c


DEVICE = NRF51

ifndef TARGET
    TARGET = SQUALL
endif

TARGET_UPPER = $(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LOWER = $(shell echo $(TARGET) | tr A-Z a-z)
BOARD = BOARD_$(TARGET_UPPER)

USE_SOFTDEVICE = s110

SDK_VERSION = 7

SDK_PATH ?= $(HOME)/Programs/Nordic/sdk/
SOFTDEVICE ?= $(HOME)/Programs/Nordic/softdevices/s110/s110_nrf51822_7.1.0_softdevice.hex

TEMPLATE_PATH ?= ../../nrf51-pure-gcc-setup/template/


LIBRARY_PATHS += ../../include
SOURCE_PATHS += ../../src
LIBRARY_PATHS += .
SOURCE_PATHS += .

CFLAGS += -Os
GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile

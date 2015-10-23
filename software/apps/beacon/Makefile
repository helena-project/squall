PROJECT_NAME = $(shell basename "$(realpath ./)")

APPLICATION_SRCS = $(notdir $(wildcard ./*.c))
APPLICATION_SRCS += softdevice_handler.c
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_conn_params.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += app_util_platform.c
APPLICATION_SRCS += nrf_drv_common.c
APPLICATION_SRCS += nrf_delay.c
APPLICATION_SRCS += led.c

APPLICATION_SRCS += simple_ble.c
APPLICATION_SRCS += simple_adv.c

DEVICE = NRF51

ifndef TARGET
	TARGET = SQUALL
endif

TARGET_UPPER = $(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LOWER = $(shell echo $(TARGET) | tr A-Z a-z)
BOARD = BOARD_$(TARGET_UPPER)

APPLICATION_SRCS += $(TARGET_LOWER).c

USE_SOFTDEVICE = s110

SDK_VERSION = 9
NRF_BASE_PATH ?= ../../nrf5x-base/
TEMPLATE_PATH ?= ../../nrf5x-base/nrf51-pure-gcc-setup/template/
SOFTDEVICE ?= ../../nrf5x-base/softdevice/s110_nrf51_8.0.0_softdevice.hex
SDK_PATH ?= ../../nrf5x-base/sdk/nrf51_sdk_9.0.0/

LIBRARY_PATHS += ../../include
SOURCE_PATHS += ../../src

CFLAGS = -Os
CFLAGS += $(MFLAGS)
GDB_PORT_NUMBER = 2331

include $(NRF_BASE_PATH)/Makefile.include
include $(TEMPLATE_PATH)Makefile

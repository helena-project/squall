PROJECT_NAME = $(shell basename "$(realpath ./)")

APPLICATION_SRCS = $(notdir $(wildcard ./*.c))
# APPLICATION_SRCS += softdevice_handler.c
# APPLICATION_SRCS += ble_advdata.c
# APPLICATION_SRCS += ble_debug_assert_handler.c
# APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += led.c

# APPLICATION_SRCS += system_nrf51.c
APPLICATION_SRCS += mem_manager.c

APPLICATION_SRCS += ipv6.c
APPLICATION_SRCS += ipv6_utils.c
APPLICATION_SRCS += udp6.c
APPLICATION_SRCS += icmp6.c

APPLICATION_SRCS += iot_context_manager.c
APPLICATION_SRCS += iot_pbuffer.c

APPLICATION_SRCS += app_button.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += app_error.c
APPLICATION_SRCS += app_gpiote.c
APPLICATION_SRCS += app_trace.c
APPLICATION_SRCS += app_scheduler.c

APPLICATION_SRCS += nrf_assert.c
APPLICATION_SRCS += nrf_delay.c
APPLICATION_SRCS += ble_conn_params.c
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += softdevice_handler.c

APPLICATION_SRCS += simple_uart.c



DEVICE = NRF51

ifndef TARGET
	TARGET = SQUALL
endif

TARGET_UPPER = $(shell echo $(TARGET) | tr a-z A-Z)
BOARD = BOARD_$(TARGET_UPPER)

SDK_VERSION = iot
USE_SOFTDEVICE = s110
USE_BLE = 1

SDK_PATH ?= $(HOME)/code/nrf51822_sdk/nrf51_iot/nrf51/
TEMPLATE_PATH ?= $(HOME)/git/nrf51-pure-gcc-setup/template/

LIBRARY_PATHS += ../../include
LIBRARY_PATHS += .
SOURCE_PATHS += ../../src





# CFLAGS = -Os
CFLAGS += $(MFLAGS)
GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile

PROJECT_NAME = $(shell basename "$(realpath ./)")

APPLICATION_SRCS = main.c
APPLICATION_SRCS += softdevice_handler.c

APPLICATION_SRCS += bootloader.c
APPLICATION_SRCS += bootloader_util_gcc.c

APPLICATION_SRCS += dfu_dual_bank.c
APPLICATION_SRCS += dfu_transport_serial.c

APPLICATION_SRCS += hci_slip.c
APPLICATION_SRCS += hci_mem_pool.c
APPLICATION_SRCS += hci_transport.c

APPLICATION_SRCS += app_button.c
APPLICATION_SRCS += app_gpiote.c
APPLICATION_SRCS += app_scheduler.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += app_uart.c

APPLICATION_SRCS += crc16.c
APPLICATION_SRCS += pstorage.c

APPLICATION_SRCS += nrf_delay.c


DEVICE = NRF51
BOARD  = BOARD_NRF6310

USE_SOFTDEVICE = s110
USE_BOOTLOADER = 1

SDK_PATH ?= $(HOME)/Downloads/nrf51822/
TEMPLATE_PATH ?= $(HOME)/git/nrf51-pure-gcc-setup/template/

LIBRARY_PATHS += ./
LIBRARY_PATHS += ../../include
LIBRARY_PATHS += $(SDK_INCLUDE_PATH)bootloader_dfu/
LIBRARY_PATHS += $(SDK_INCLUDE_PATH)bootloader_dfu/hci_transport/

CFLAGS = -Os
GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile

PROJECT_NAME = $(shell basename "$(realpath ./)")

APPLICATION_SRCS = main.c
APPLICATION_SRCS += softdevice_handler.c

APPLICATION_SRCS += bootloader.c
APPLICATION_SRCS += bootloader_util_gcc.c

APPLICATION_SRCS += dfu_dual_bank.c
#APPLICATION_SRCS += dfu_single_bank.c
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
BOARD  = BOARD_SQUALL

USE_SOFTDEVICE = s110
USE_BOOTLOADER = 1

#SDK_PATH ?= $(HOME)/Downloads/nrf51822/
SDK_PATH ?= $(HOME)/nrf51822_sdk/nrf51822/
TEMPLATE_PATH ?= $(HOME)/git/nrf51-pure-gcc-setup/template/

LIBRARY_PATHS += ./
LIBRARY_PATHS += ../../include
LIBRARY_PATHS += $(SDK_INCLUDE_PATH)bootloader_dfu/
LIBRARY_PATHS += $(SDK_INCLUDE_PATH)bootloader_dfu/hci_transport/

# Need to make sure that the squall specific header files get used, not
# the ones in the SDK
CFLAGS += -include bootloader_types.h
CFLAGS += -include dfu_types.h
CFLAGS += -include hci_transport_config.h


CFLAGS += -Os
GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile

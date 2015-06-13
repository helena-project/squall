#ifndef TORCH_H
#define TORCH_H

#include "nrf_gpio.h"

#ifndef DEVICE_NAME
#define DEVICE_NAME    "torch"
#endif

extern uint8_t MAC_ADDR[6];

void platform_init ();

// No actual LEDs, but these help applications compile
#define LED_START      10
#define LED_0          10
#define LED_1          10
#define LED_STOP       10

// Interrupt pin to the CC2538
#define INTERRUPT_PIN 12

// SPI lines running to the CC2538
#define SPIS_MISO_PIN  0    // SPI MISO signal.
#define SPIS_CSN_PIN   11   // SPI CSN signal.
#define SPIS_MOSI_PIN  2    // SPI MOSI signal.
#define SPIS_SCK_PIN   1    // SPI SCK signal.

// UART lines to the CC2538
#define RX_PIN_NUMBER  19
#define TX_PIN_NUMBER  20
#define CTS_PIN_NUMBER 7  // not connected
#define RTS_PIN_NUMBER 7  // not connected
#define HWFC           false


#endif

#ifndef SQUALL_H
#define SQUALL_H

#include "nrf_gpio.h"

#ifndef DEVICE_NAME
#define DEVICE_NAME    "squall"
#endif

extern uint8_t MAC_ADDR[6];


#define LED_START      13
#define LED_0          13
#define LED_1          13
	//  #define LED_2          22
	// #define LED_3          23
	// #define LED_4          24
#define LED_STOP       13

//#define BUTTON_START   16
//#define BUTTON_0       16
//#define BUTTON_1       17
//#define BUTTON_STOP    17
//#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

// This pin is mapped to the FTDI chip to all the device to enter the
// bootloader mode.
#define BOOTLOADER_CTRL_PIN  3
#define BOOTLOADER_CTRL_PULL NRF_GPIO_PIN_PULLUP



// #define BSP_LED_0      LED_1
// #define BSP_LED_1      LED_2
// #define BSP_LED_2      LED_3
// #define BSP_LED_3      LED_4

// #define BSP_LED_0_MASK (1<<BSP_LED_0)
// #define BSP_LED_1_MASK (1<<BSP_LED_1)
// #define BSP_LED_2_MASK (1<<BSP_LED_2)
// #define BSP_LED_3_MASK (1<<BSP_LED_3)

// #define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK)
// /* all LEDs are lit when GPIO is low */
// #define LEDS_INV_MASK  LEDS_MASK

// #define BUTTONS_NUMBER 4

// #define BUTTON_START   17
// #define BUTTON_1       17
// #define BUTTON_2       18
// #define BUTTON_3       19
// #define BUTTON_4       20
// #define BUTTON_STOP    20
// #define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

// #define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 }

// #define BSP_BUTTON_0   BUTTON_1
// #define BSP_BUTTON_1   BUTTON_2
// #define BSP_BUTTON_2   BUTTON_3
// #define BSP_BUTTON_3   BUTTON_4

// #define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
// #define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)
// #define BSP_BUTTON_2_MASK (1<<BSP_BUTTON_2)
// #define BSP_BUTTON_3_MASK (1<<BSP_BUTTON_3)

// #define BUTTONS_MASK   0x001E0000

#define I2C_SCL_PIN   7
#define I2C_SDA_PIN   8

#define RX_PIN_NUMBER  28
#define TX_PIN_NUMBER  29
#define CTS_PIN_NUMBER 0
#define RTS_PIN_NUMBER 0
#define HWFC           false

#define SPIS_MISO_PIN  10    // SPI MISO signal.
#define SPIS_CSN_PIN   1     // SPI CSN signal.
#define SPIS_MOSI_PIN  11    // SPI MOSI signal.
#define SPIS_SCK_PIN   9     // SPI SCK signal.

#define SPIM0_SCK_PIN       9u     /**< SPI clock GPIO pin number. */
#define SPIM0_MOSI_PIN      11u     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM0_MISO_PIN      10u     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM0_SS_PIN        1u     /**< SPI Slave Select GPIO pin number. */

//#define SPIM1_SCK_PIN       16u     /**< SPI clock GPIO pin number. */
//#define SPIM1_MOSI_PIN      18u     /**< SPI Master Out Slave In GPIO pin number. */
//#define SPIM1_MISO_PIN      17u     /**< SPI Master In Slave Out GPIO pin number. */
//#define SPIM1_SS_PIN        19u     /**< SPI Slave Select GPIO pin number. */

// serialization APPLICATION board

// UART
// this configuration works with the SPI wires setup
#define SER_APP_RX_PIN              20     // UART RX pin number.
#define SER_APP_TX_PIN              22     // UART TX pin number.
#define SER_APP_CTS_PIN             23     // UART Clear To Send pin number.
#define SER_APP_RTS_PIN             21     // UART Request To Send pin number.

// SPI
#if 0
#define SER_APP_SPIM0_SCK_PIN       20     // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      17     // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      16     // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        21     // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       19     // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       18     // SPI REQUEST GPIO pin number
#else
#define SER_APP_SPIM0_SCK_PIN       23     // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      20     // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      22     // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        21     // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       25     // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       24     // SPI REQUEST GPIO pin number
#endif

// serialization CONNECTIVITY board

// UART
#if 0
#define SER_CON_RX_PIN              22    // UART RX pin number.
#define SER_CON_TX_PIN              20    // UART TX pin number.
#define SER_CON_CTS_PIN             21    // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             23    // UART Request To Send pin number. Not used if HWFC is set to false.
#else
// this configuration works with the SPI wires setup
#define SER_CON_RX_PIN              20    // UART RX pin number.
#define SER_CON_TX_PIN              22    // UART TX pin number.
#define SER_CON_CTS_PIN             21    // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             23    // UART Request To Send pin number. Not used if HWFC is set to false.
#endif

//SPI
#if 0
#define SER_CON_SPIS_SCK_PIN        20    // SPI SCK signal.
#define SER_CON_SPIS_MISO_PIN       16    // SPI MISO signal.
#define SER_CON_SPIS_MOSI_PIN       17    // SPI MOSI signal.
#define SER_CON_SPIS_CSN_PIN        21    // SPI CSN signal.
#define SER_CON_SPIS_RDY_PIN        19     // SPI READY GPIO pin number.
#define SER_CON_SPIS_REQ_PIN        18     // SPI REQUEST GPIO pin number.
#else
#define SER_CON_SPIS_SCK_PIN        23    // SPI SCK signal.
#define SER_CON_SPIS_MOSI_PIN       22    // SPI MOSI signal.
#define SER_CON_SPIS_MISO_PIN       20    // SPI MISO signal.
#define SER_CON_SPIS_CSN_PIN        21    // SPI CSN signal.
#define SER_CON_SPIS_RDY_PIN        25     // SPI READY GPIO pin number.
#define SER_CON_SPIS_REQ_PIN        24     // SPI REQUEST GPIO pin number.
#endif

#define SER_CONN_ASSERT_LED_PIN     LED_0

#endif

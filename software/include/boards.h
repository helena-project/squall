#ifndef BOARDS_H
#define BOARDS_H

#if defined(BOARD_NRF6310)
	#include "boards/nrf6310.h"
	#define DEVICE_NAME "nrf6310"
#elif defined(BOARD_PCA10000)
	#include "boards/pca10000.h"
	#define DEVICE_NAME "pca10000"
#elif defined(BOARD_PCA10001)
	#include "boards/pca10001.h"
	#define DEVICE_NAME "pca10001"
#elif defined(BOARD_PCA10003)
	#include "boards/pca10003.h"
	#define DEVICE_NAME "pca10003"
#elif defined(BOARD_SQUALL)
	#include "squall.h"
#elif defined(BOARD_ZIGBEAG)
	#include "zigbeag.h"
#elif defined(BOARD_QUITBIT)
	#include "quitbit.h"
#elif defined(BOARD_FIRESTORM)
	#include "firestorm.h"
#elif defined(BOARD_TORCH)
	#include "torch.h"
#else
	#error "Board is not defined in boards.h"
#endif


#define LEDS_OFF(leds_mask) do {  NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                            NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LEDS_ON(leds_mask) do {  NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                           NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LED_IS_ON(leds_mask) ((leds_mask) & (NRF_GPIO->OUT ^ LEDS_INV_MASK) )

#define LEDS_INVERT(leds_mask) do { uint32_t gpio_state = NRF_GPIO->OUT;      \
                              NRF_GPIO->OUTSET = ((leds_mask) & ~gpio_state); \
                              NRF_GPIO->OUTCLR = ((leds_mask) & gpio_state); } while (0)

#define LEDS_CONFIGURE(leds_mask) do { uint32_t pin;                  \
                                  for (pin = 0; pin < 32; pin++) \
                                      if ( (leds_mask) & (1 << pin) )   \
                                          nrf_gpio_cfg_output(pin); } while (0)

#endif

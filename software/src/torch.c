#include "torch.h"

#include "led.h"

uint8_t MAC_ADDR[6] = {0x00, 0x00, 0x50, 0xe5, 0x98, 0xc0};

void platform_init () {

	// Setup the interrupt pin to the CC2538
	nrf_gpio_cfg_output(INTERRUPT_PIN);
    nrf_gpio_pin_clear(INTERRUPT_PIN);

}


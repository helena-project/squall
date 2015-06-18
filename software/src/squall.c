
#include "squall.h"

#include "twi_master.h"

#include "led.h"

uint8_t MAC_ADDR[6] = {0x00, 0x00, 0x50, 0xe5, 0x98, 0xc0};

bool platform_init () {
	bool err;

	led_init(LED_0);

	err = twi_master_init();

	return err;
}


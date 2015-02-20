
#include "squall.h"

#include "twi_master.h"

#include "led.h"

bool platform_init () {
	bool err;

	led_init(LED_0);

	err = twi_master_init();

	return err;
}


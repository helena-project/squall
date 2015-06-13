
#include "torch.h"

#include "twi_master.h"

#include "led.h"

bool platform_init () {
	bool err;


	err = twi_master_init();

	return err;
}


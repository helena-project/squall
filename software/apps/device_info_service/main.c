
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "bleApp.h"


int main(void)
{
    // Initialize
	appInit();

    // Enter main loop
    while(1)
    {
		appService();
    }
}

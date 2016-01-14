
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_delay.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "led.h"
#include "bleConfig.h"
#include "boards.h"

#include "simple_ble.h"
#include "simple_adv.h"
#include "simple_timer.h"

/*******************************************************************************
 *   State and Configuration
 ******************************************************************************/

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
    .platform_id       = 0x40,              // used as 4th octect in device BLE address
    .device_id         = DEVICE_ID_DEFAULT,
    .adv_name          = DEVICE_NAME,       // used in advertisements if there is room
    .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
    .min_conn_interval = MSEC_TO_UNITS(8, UNIT_1_25_MS),
    .max_conn_interval = MSEC_TO_UNITS(10, UNIT_1_25_MS),
};

// State for this CuSToM service application
static ble_cstm_t cstm;

// Randomly generated UUID for this service
simple_ble_service_t custom_service = {
    .uuid128 = {{0x83, 0xbe, 0xd0, 0x58, 0xf1, 0xaf, 0x42, 0x62,
	             0x9c, 0xf8, 0x4f, 0x73, 0xe9, 0xdb, 0x9c, 0xe4}}
};
simple_ble_char_t num_char = {.uuid16 = CUSTOM_SERVICE_CHAR_NUMBER_SHORT_UUID};


/*******************************************************************************
 *   Timer Callback
 ******************************************************************************/

static void timer_handler (void* p_context) {
	cstm.num_value++;
}


/*******************************************************************************
 *   INIT FUNCTIONS
 ******************************************************************************/

// Init services
void services_init (void) {

    // Add custom service
    simple_ble_add_service(&custom_service);

    // Add the characteristic
    simple_ble_add_characteristic(1, 0, 0,0,  // read, write, notify, vlen
                                  1, (uint8_t*) &cstm.num_value,
                                  &custom_service,
                                  &num_char);
}

void ble_error(uint32_t error_code) {
    // display error when developing on the squall platform
    led_init(LED_0);
    led_on(LED_0);
}


int main(void) {

	// Initialize
	led_init(LED_0);

	// Setup Simple BLE
	simple_ble_init(&ble_config);

	// Setup our advertisement
	simple_adv_service(&custom_service.uuid_handle);

    // Start timer to update characteristic
    simple_timer_start(1000, timer_handler);

	// Enter main loop
	while(1) {
		power_manage();
	}
}


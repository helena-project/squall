
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_delay.h"
#include "ble.h"
#include "ble_db_discovery.h"
#include "softdevice_handler.h"
#include "app_util.h"
#include "app_error.h"
#include "ble_advdata_parser.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "app_trace.h"
#include "ble_hrs_c.h"
#include "ble_bas_c.h"
#include "ble_dis.h"
#include "app_util.h"
#include "app_timer.h"
#include "simple_ble.h"
#include "simple_adv.h"

#include "simple_ble.h"
#include "simple_adv.h"


ble_uuid_t dis_service_uuid = {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE};

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
    .platform_id       = 0x40,              // used as 4th octect in device BLE address
    .device_id         = DEVICE_ID_DEFAULT,
    .adv_name          = DEVICE_NAME,       // used in advertisements if there is room
    .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
    .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
    .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};


// Init the Device Info service
void services_init (void) {
	uint32_t 	err_code;
	ble_dis_init_t dis_init;

	memset(&dis_init, 0, sizeof(dis_init));

	ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char*)MANUFACTURER_NAME);
	ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char*)MODEL_NUMBER);
	ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char*)HARDWARE_REVISION);
	ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char*)FIRMWARE_REVISION);

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

	err_code = ble_dis_init(&dis_init);
	APP_ERROR_CHECK(err_code);
}


int main(void) {

    // Setup BLE
    simple_ble_init(&ble_config);

    // Advertise this
    simple_adv_service(&dis_service_uuid);

    // Enter main loop
    while(1) {
		power_manage();
    }
}

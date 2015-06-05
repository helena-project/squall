
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
#include "bleConfig.h"
#include "boards.h"

//module-private variable declarations

// Randomly generated UUID for this service
const ble_uuid128_t custom_service_uuid128 = {
	{0x83, 0xbe, 0xd0, 0x58, 0xf1, 0xaf, 0x42, 0x62,
	 0x9c, 0xf8, 0x4f, 0x73, 0xe9, 0xdb, 0x9c, 0xe4}
};

ble_uuid_t custom_service_uuid;

// Security requirements for this application.
static ble_gap_sec_params_t m_sec_params = {
	SEC_PARAM_TIMEOUT,
	SEC_PARAM_BOND,
	SEC_PARAM_MITM,
	SEC_PARAM_IO_CAPABILITIES,
	SEC_PARAM_OOB,
	SEC_PARAM_MIN_KEY_SIZE,
	SEC_PARAM_MAX_KEY_SIZE,
};

// State for this CuSToM service application
static ble_cstm_t cstm;

static ble_gap_adv_params_t m_adv_params;

static app_timer_id_t  characteristic_timer;



static void advertisingStart(void) {

	//start advertising
	uint32_t             err_code;
	err_code = sd_ble_gap_adv_start(&m_adv_params);
	APP_ERROR_CHECK(err_code);
}

static void advertisingStop(void) {
	//start advertising
	uint32_t             err_code;
	err_code = sd_ble_gap_adv_stop();
	APP_ERROR_CHECK(err_code);
}


//softdevice assertion callback
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

//service error callback
static void service_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}


//connection parameters event handler callback
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
	uint32_t err_code;

	if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
	{
		err_code = sd_ble_gap_disconnect(cstm.conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}

//connection parameters error callback
static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}





//ble event callback
static void on_ble_evt(ble_evt_t * p_ble_evt) {
	uint32_t                         err_code;
	static ble_gap_evt_auth_status_t m_auth_status;
	ble_gap_enc_info_t *             p_enc_info;

	switch (p_ble_evt->header.evt_id) {
		case BLE_GAP_EVT_CONNECTED:
			cstm.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			//advertisingStop();
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			cstm.conn_handle = BLE_CONN_HANDLE_INVALID;
			advertisingStart();
			break;

		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			err_code = sd_ble_gap_sec_params_reply(cstm.conn_handle,
												   BLE_GAP_SEC_STATUS_SUCCESS,
												   &m_sec_params);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
			err_code = sd_ble_gatts_sys_attr_set(cstm.conn_handle, NULL, 0);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GAP_EVT_AUTH_STATUS:
			m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
			break;

		case BLE_GAP_EVT_SEC_INFO_REQUEST:
			p_enc_info = &m_auth_status.periph_keys.enc_info;
			if (p_enc_info->div ==
							p_ble_evt->evt.gap_evt.params.sec_info_request.div) {
				err_code = sd_ble_gap_sec_info_reply(cstm.conn_handle,
															p_enc_info, NULL);
				APP_ERROR_CHECK(err_code);
			} else {
				// No keys found for this device
				err_code = sd_ble_gap_sec_info_reply(cstm.conn_handle, NULL, NULL);
				APP_ERROR_CHECK(err_code);
			}
			break;

		case BLE_GAP_EVT_TIMEOUT:
			if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT) {
				err_code = sd_power_system_off();
				APP_ERROR_CHECK(err_code);
			}
			break;

		default:
			break;
	}
}


//dispatching a BLE stack event to all modules with a BLE stack event handler.
//This function is called from the scheduler in the main loop after a BLE stack
//event has been received.
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	on_ble_evt(p_ble_evt);
	ble_conn_params_on_ble_evt(p_ble_evt);

	/*
	YOUR_JOB: Add service ble_evt handlers calls here, like, for example:
	ble_bas_on_ble_evt(&m_bas, p_ble_evt);
	*/

}


//for dispatching a system event to interested modules.
//is called from the System event interrupt handler after a system
//event has been received.
static void sys_evt_dispatch(uint32_t sys_evt)
{

}

static void timer_handler (void* p_context) {
	cstm.num_value++;
}



/*******************************************************************************
 *   INIT FUNCTIONS
 ******************************************************************************/

// Initialize connection parameters
static void conn_params_init(void) {
	uint32_t               err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params                  = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail             = false;
	cp_init.evt_handler                    = on_conn_params_evt;
	cp_init.error_handler                  = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}

static void timers_init(void) {
	uint32_t err_code;

	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

	err_code = app_timer_create(&characteristic_timer,
	                            APP_TIMER_MODE_REPEATED,
	                            timer_handler);
	APP_ERROR_CHECK(err_code);

	// Start timer to update characteristic
	err_code = app_timer_start(characteristic_timer, UPDATE_RATE, NULL);
	APP_ERROR_CHECK(err_code);
}

// initialize advertising
static void advertising_init(void) {
	uint32_t      err_code;
	ble_advdata_t advdata;
	ble_advdata_t srdata;
	uint8_t       flags =  BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	// Advertise our custom service
	ble_uuid_t adv_uuids[] = {custom_service_uuid};

	// Build and set advertising data
	memset(&advdata, 0, sizeof(advdata));
	memset(&srdata, 0, sizeof(srdata));

	advdata.name_type               = BLE_ADVDATA_NO_NAME;
	advdata.include_appearance      = true;
	advdata.flags.size              = sizeof(flags);
	advdata.flags.p_data            = &flags;
	advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
	advdata.uuids_complete.p_uuids  = adv_uuids;

	// Put the name in the SCAN RESPONSE data
	srdata.name_type                = BLE_ADVDATA_FULL_NAME;

	err_code = ble_advdata_set(&advdata, &srdata);
	APP_ERROR_CHECK(err_code);

	memset(&m_adv_params, 0, sizeof(m_adv_params));

	m_adv_params.type				= BLE_GAP_ADV_TYPE_ADV_IND;
	m_adv_params.p_peer_addr		= NULL;
	m_adv_params.fp					= BLE_GAP_ADV_FP_ANY;
	m_adv_params.interval    		= APP_ADV_INTERVAL;
	m_adv_params.timeout     		= APP_ADV_TIMEOUT_IN_SECONDS;
}

//init services
static void services_init(void)
{
	uint32_t err_code;

	// Setup our long UUID so that nRF recognizes it. This is done by
	// storing the full UUID and essentially using `custom_service_uuid`
	// as a handle.
	custom_service_uuid.uuid = CUSTOM_SERVICE_SHORT_UUID;
	err_code = sd_ble_uuid_vs_add(&custom_service_uuid128, &(custom_service_uuid.type));
	APP_ERROR_CHECK(err_code);
	cstm.uuid_type = custom_service_uuid.type;

	// Add the custom service to the system
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
	                                    &custom_service_uuid,
	                                    &(cstm.service_handle));
	APP_ERROR_CHECK(err_code);

	// Add the number characteristic to the service
	{
		ble_gatts_char_md_t char_md;
		ble_gatts_attr_t    attr_char_value;
		ble_uuid_t          char_uuid;
		ble_gatts_attr_md_t attr_md;

		// Init value
		cstm.num_value = 0xbbcc;

		memset(&char_md, 0, sizeof(char_md));

		// This characteristic is a read
		char_md.char_props.read          = 1;
		char_md.p_char_user_desc         = NULL;
		char_md.p_char_pf                = NULL;
		char_md.p_user_desc_md           = NULL;
		char_md.p_cccd_md                = NULL;
		char_md.p_sccd_md                = NULL;

		char_uuid.type = cstm.uuid_type;
		char_uuid.uuid = CUSTOM_SERVICE_CHAR_NUMBER_SHORT_UUID;

		memset(&attr_md, 0, sizeof(attr_md));

		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

		attr_md.vloc    = BLE_GATTS_VLOC_USER;
		attr_md.rd_auth = 0;
		attr_md.wr_auth = 0;
		attr_md.vlen    = 1;

		memset(&attr_char_value, 0, sizeof(attr_char_value));

		attr_char_value.p_uuid    = &char_uuid;
		attr_char_value.p_attr_md = &attr_md;
		attr_char_value.init_len  = 2;
		attr_char_value.init_offs = 0;
		attr_char_value.max_len   = 2;
		attr_char_value.p_value   = (uint8_t*) &cstm.num_value;

		err_code = sd_ble_gatts_characteristic_add(cstm.service_handle,
		                                           &char_md,
		                                           &attr_char_value,
		                                           &cstm.num_handles);
		APP_ERROR_CHECK(err_code);
	}

}

// gap name/appearance/connection parameters
static void gap_params_init(void) {
	uint32_t                err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	sd_ble_gap_tx_power_set(4);

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
										  (const uint8_t *)DEVICE_NAME,
										  strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency     = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

//ble event interrupt
static void ble_stack_init(void) {
	uint32_t err_code;

	// Initialize the SoftDevice handler module.
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION,
																		false);

	// Enable BLE stack
	ble_enable_params_t ble_enable_params;
	memset(&ble_enable_params, 0, sizeof(ble_enable_params));
	ble_enable_params.gatts_enable_params.service_changed =
											IS_SRVC_CHANGED_CHARACT_PRESENT;
	err_code = sd_ble_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	//Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);

	// Set the address of this squall
	{
		ble_gap_addr_t gap_addr;

		// Get the current original address
		sd_ble_gap_address_get(&gap_addr);

		// Set the new BLE address with the Michigan OUI
		gap_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
		memcpy(gap_addr.addr+2, MAC_ADDR+2, sizeof(gap_addr.addr)-2);
		err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &gap_addr);
		APP_ERROR_CHECK(err_code);
	}
}


static void power_manage(void) {
	uint32_t err_code = sd_app_evt_wait();
	APP_ERROR_CHECK(err_code);
}



int main(void) {

	// Initialize
	led_init(LED_0);

	ble_stack_init();
	gap_params_init();
	services_init();
	advertising_init();

	// You must initialize and use app timers to initialize conn params
	timers_init();

	conn_params_init();
	advertisingStart();

	// Enter main loop
	while(1) {
		power_manage();
	}
}

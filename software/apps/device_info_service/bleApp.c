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
#include "ble_dis.h"
#include "boards.h"

//module-private variable declarations

//Security requirements for this application.
static ble_gap_sec_params_t             m_sec_params;

//current connection handle
static uint16_t 		m_conn_handle = BLE_CONN_HANDLE_INVALID;
static ble_gap_adv_params_t m_adv_params;

//module-private function declaration
static void timers_init();
void assert_nrf_callback(uint16_t, const uint8_t*);
static void service_error_handler(uint32_t);
static void gap_params_init(void);
static void services_init(void);
static void sec_params_init(void);
static void power_manage(void);
static void scheduler_init(void);
static void ble_stack_init(void);
static void sys_evt_dispatch(uint32_t);
static void on_ble_evt(ble_evt_t * p_ble_evt);
static void advertising_init(void);
static void conn_params_init(void);
static void bleInit();
static void advertisingStart();
static void advertisingStop();

//global function implementation
void appService(void) {
	power_manage();
}


void appInit() {
	bleInit();
}

void bleInit() {
    ble_stack_init();
    scheduler_init();
    sec_params_init();
    gap_params_init();
    advertising_init();
    services_init();

	//You must initialize and use app timers to initialize conn params
	timers_init();

    conn_params_init();
	advertisingStart();
}

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

//initialize advertising
static void advertising_init(void)
{
    uint32_t      err_code;
	ble_advdata_t advdata;
    uint8_t       flags =  BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    ble_uuid_t adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance		= true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

	memset(&m_adv_params, 0, sizeof(m_adv_params));

	m_adv_params.type				= BLE_GAP_ADV_TYPE_ADV_IND;
	m_adv_params.p_peer_addr		= NULL;
	m_adv_params.fp					= BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    		= APP_ADV_INTERVAL;
    m_adv_params.timeout     		= APP_ADV_TIMEOUT_IN_SECONDS;
}

static void timers_init(void)
{
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
}


//gap name/apperance/connection parameters
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

	sd_ble_gap_tx_power_set(4);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_COMPUTER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


//init services
static void services_init(void)
{
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


//init security
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}

//connection parameters event handler callback
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

//connection parameters error callback
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

//initialize connection parameters

static void conn_params_init(void)
{
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



//ble event callback
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			//advertisingStop();
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
			advertisingStart();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div ==
							p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle,
															p_enc_info, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;
        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
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

//ble event interrupt
static void ble_stack_init(void)
{
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

    // Set the address of this BLEES / squall
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


static void scheduler_init(void)
{

}


static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


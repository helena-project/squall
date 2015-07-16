/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */



/**
 * This file is the main file for the application described in application note
 * nAN-36 Creating Bluetooth® Low Energy Applications Using nRF51822.
 */



#include <stdint.h>

#include <string.h>

#include "nordic_common.h"

#include "nrf.h" //no

#include "app_error.h"

#include "nrf_gpio.h"

#include "nrf51_bitfields.h"

#include "ble.h"

#include "ble_hci.h"

#include "ble_srv_common.h"

#include "ble_advdata.h" // no

#include "ble_conn_params.h"

//#include "boards.h"

#include "app_scheduler.h"

#include "softdevice_handler.h" //no

//#include "app_timer_appsh.h"
#include "app_timer.h"


#include "ble_error_log.h"

#include "app_gpiote.h"

#include "app_button.h"

#include "ble_debug_assert_handler.h"

#include "pstorage.h"

#include "ble_ess.h"

#include "bsp.h"

#include "ble_gap.h"

#include "ble_gatts.h"

#include "ble_sensorsim.h"

#include "device_manager.h"

 #include "app_trace.h"


#include "squall.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DEVICE_NAME                     "squall_ess_notif"                             /**< Name of device. Will be included in the advertising data. */


#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */



#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */

#define APP_TIMER_MAX_TIMERS            3                                           /**< Maximum number of simultaneously created timers. */

#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */



#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */

#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (1 second). */

#define SLAVE_LATENCY                   0                                           /**< Slave latency. */

#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */

#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */

#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */



#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */



#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define ESS_MEAS_INTERVAL   APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)


#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */

#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */

#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */

#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */

#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */

#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */

#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

#define TEMP_LEVEL_INCREMENT    100
#define PRES_LEVEL_INCREMENT    100
#define HUM_LEVEL_INCREMENT     100
#define MAX_TEMP_LEVEL          100000000
#define MIN_TEMP_LEVEL          123
#define MAX_PRES_LEVEL          100000000
#define MIN_PRES_LEVEL          456
#define MAX_HUM_LEVEL           100000000
#define MIN_HUM_LEVEL           789

#define BOND_DELETE_ALL_BUTTON_ID            1                                          /**< Button used for deleting all bonded centrals during startup. */


static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_ess_t                        m_ess;

static ble_sensorsim_cfg_t                   m_temp_sim_cfg;                         /**< Temperature sensor simulator configuration. */
static ble_sensorsim_state_t                 m_temp_sim_state;                       /**< Temperature sensor simulator state. */

static ble_sensorsim_cfg_t                   m_pres_sim_cfg;                         /**< Pressure sensor simulator configuration. */
static ble_sensorsim_state_t                 m_pres_sim_state;                       /**< Pressure sensor simulator state. */

static ble_sensorsim_cfg_t                   m_hum_sim_cfg;                         /**< Humidity sensor simulator configuration. */
static ble_sensorsim_state_t                 m_hum_sim_state;                       /**< Humidity sensor simulator state. */

static app_timer_id_t                        m_ess_timer_id;                        /**< ESS timer. */

static bool     m_ess_meas_not_conf_pending = false; /** Flag to keep track of when a notification confirmation is pending */

static bool                                  m_memory_access_in_progress = false;       /**< Flag to keep track of ongoing operations on persistent memory. */

static dm_application_instance_t             m_app_handle;                              /**< Application identifier allocated by device manager */

static dm_security_status_t                 m_security_status;

static ble_gap_whitelist_t                  m_whitelist;

// Persistent storage system event handler
void pstorage_sys_event_handler (uint32_t p_evt);



/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    ble_debug_assert_handler(error_code, line_num, p_file_name);
    
    // On assert, the system can only recover with a reset.
    //NVIC_SystemReset();
}

/**@brief Function for populating simulated ess measurement.
 */
/*
 static void ess_sim_measurement(ble_ess_meas_t * p_meas)
 {
 static ble_date_time_t s_time_stamp = { 2012, 12, 5, 11, 05, 03 };
 static uint8_t         s_ndx        = 0;
 //p_meas->blood_pressure_units_in_kpa       = false;
 p_meas->time_stamp_present                = (s_ndx == 0) || (s_ndx == 2);
 //p_meas->pulse_rate_present                = (s_ndx == 0) || (s_ndx == 1);
 //p_meas->user_id_present                   = false;
 //p_meas->measurement_status_present        = false;
 p_meas->temp = m_ess_meas_sim_val[s_ndx].temp;
 p_meas->pres  = m_ess_meas_sim_val[s_ndx].pres;
 p_meas->hum = m_ess_meas_sim_val[s_ndx].hum;
 p_meas->time_stamp = s_time_stamp;
 // Update index to simulated measurements.
 s_ndx++;
 if (s_ndx == NUM_SIM_MEAS_VALUES)
 {
 s_ndx = 0;
 }
 // Update simulated time stamp.
 s_time_stamp.seconds += 27;
 if (s_time_stamp.seconds > 59)
 {
 s_time_stamp.seconds -= 60;
 s_time_stamp.minutes++;
 if (s_time_stamp.minutes > 59)
 {
 s_time_stamp.minutes = 0;
 }
 }
 } */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void ess_update(void)
{
    uint32_t err_code;
    //ess_meas_t meas_values;
    //uint8_t * temp_meas_val;
    //uint8_t  * pres_meas_val;
    //uint8_t  * hum_meas_val;
    
    int16_t sim_temp_meas;
    uint32_t sim_pres_meas;
    uint16_t sim_hum_meas;

    uint8_t  temp_meas_val[4];
    //uint8_t  pres_meas_val[4];
    //uint8_t  hum_meas_val[4];
    
    uint32_t meas;


    //ess_sensorsim_measure(&m_ess, &meas_values);
    meas = (uint16_t)(ble_sensorsim_measure(&m_temp_sim_state, &m_temp_sim_cfg));
    //uint8_t * temp_meas_val = (uint8_t*)(&sim_temp_meas);
    
    memcpy(temp_meas_val, meas, 2);

    //printf("%d", *temp_meas_val);


    err_code = ble_ess_char_value_update(&m_ess, &(m_ess.temp_char_handles), (m_ess.temp_val_last), temp_meas_val, MAX_TEMP_LEN, (m_ess.temp_trigger_val_cond), m_ess.temp_trigger_val_buff, true );
    
    //err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
    
    //ess_sensorsim_measure(&m_ess, &meas_values);
    sim_pres_meas = (uint32_t)(ble_sensorsim_measure(&m_pres_sim_state, &m_pres_sim_cfg));
    uint8_t * pres_meas_val = (uint8_t*)(&sim_pres_meas);
    
    err_code = ble_ess_char_value_update(&m_ess, &(m_ess.pres_char_handles), (m_ess.pres_val_last), pres_meas_val, MAX_PRES_LEN, (m_ess.pres_trigger_val_cond), m_ess.pres_trigger_val_buff, false);
    
    //err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
    
    
    //ess_sensorsim_measure(&m_ess, &meas_values);
    sim_hum_meas = (uint16_t)(ble_sensorsim_measure(&m_hum_sim_state, &m_hum_sim_cfg));
    uint8_t * hum_meas_val = (uint8_t*)(&sim_hum_meas);
    
    err_code = ble_ess_char_value_update(&m_ess, &(m_ess.hum_char_handles), (m_ess.hum_val_last), hum_meas_val, MAX_HUM_LEN, (m_ess.hum_trigger_val_cond), m_ess.hum_trigger_val_buff, false);
    
    //err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
    
    
    
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void ess_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ess_update();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 
 * Parameters for APP_TIMER_INIT:
 * [in] PRESCALER   Value of the RTC1 PRESCALER register. This will decide the timer tick rate. Set to 0 for no prescaling.
 * [in] MAX_TIMERS  Maximum number of timers that can be created at any given time.
 * [in] OP_QUEUES_SIZE  Size of queues holding timer operations that are pending execution.
 * [in] USE_SCHEDULER   TRUE if the application is using the event scheduler, FALSE otherwise.
 * Note:
 * Since this macro allocates a buffer, it must only be called once (it is OK to call it several times as long as it is from the same location, e.g. to do a reinitialization).
*/
static void timers_init(void)
{
    uint32_t err_code;
    
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
    
    // Create timers.
    // APP_TIMER_MODE_REPEATED means the timer will resart every time it expires
    // ess_meas_timeout_handler called when the timer expires
    err_code = app_timer_create(&m_ess_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                ess_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
    
    /* //Create timers.
    err_code = app_timer_create(&m_temp_timer_id,
    APP_TIMER_MODE_REPEATED,
    temp_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_hum_timer_id,
    APP_TIMER_MODE_REPEATED,
    hum_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_pres_timer_id,
    APP_TIMER_MODE_REPEATED,
    pres_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
    */
    
}





/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)

{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    volatile uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    ble_uuid_t adv_uuids[] = {
        {ESS_UUID_SERVICE, m_ess.uuid_type}
    };
    
    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    
    
    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    //err_code = ble_advdata_set(&advdata, NULL);
    //err_code = ble_advdata_set(NULL, &scanrsp);
    
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the ESS events.
 *
 * @details This function will be called for all ESS events which are passed to
 *          the application.
 *
 * @param[in]   p_ess   Environmental Sensing Service structure.
 * @param[in]   p_evt   Event received from the ESS.
 */
static void on_ess_evt(ble_ess_t * p_ess, ble_ess_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_ESS_EVT_NOTIFICATION_ENABLED:
            // Notification has been enabled, send a single ESS measurement.
            ess_meas_send(p_ess);
            break;
            
        case BLE_ESS_EVT_NOTIFICATION_CONFIRMED:
            m_ess_meas_not_conf_pending = false;
            break;
            
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the temperature.
 */
static void temp_char_init(ble_ess_init_t * p_ess_init)
{
    int16_t init_data = 123;
    p_ess_init->init_temp_data = init_data;
    
    int16_t meas = 29472;

    p_ess_init->temp_trigger_condition = TRIG_WHILE_LT;

    p_ess_init->temp_trigger_val_var = (meas);

}

/**@brief Function for initializing the pressure.
 */
static void pres_char_init(ble_ess_init_t * p_ess_init)
{
    uint32_t init_data = 456;
    p_ess_init->init_pres_data = init_data;

    uint32_t meas = 30556;

    p_ess_init->pres_trigger_condition = TRIG_INACTIVE;

    p_ess_init->pres_trigger_val_var = (meas);

}

/**@brief Function for initializing the humidity.
 */
static void hum_char_init(ble_ess_init_t * p_ess_init)
{
    uint16_t init_data = 789;
    p_ess_init->init_hum_data = init_data;
    
    uint16_t meas = 240;

    p_ess_init->hum_trigger_condition = TRIG_INACTIVE;

    p_ess_init->hum_trigger_val_var = (meas);

}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    ble_ess_init_t ess_init;
    
    //Initialize the Environmental Sensing Service
    memset(&ess_init, 0 , sizeof(ess_init));
    
    ess_init.evt_handler = on_ess_evt;
    ess_init.is_notify_supported = true;
    
    
    temp_char_init(&ess_init); //initialize temp
    pres_char_init(&ess_init); //initialize pres
    hum_char_init(&ess_init); //initialize hum
    
    err_code = ble_ess_init(&m_ess, &ess_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the sensor simulators.
 */
static void sensor_sim_init(void)
{
    m_temp_sim_cfg.min          = MIN_TEMP_LEVEL;
    m_temp_sim_cfg.max          = MAX_TEMP_LEVEL;
    m_temp_sim_cfg.incr         = TEMP_LEVEL_INCREMENT;
    m_temp_sim_cfg.start_at_max = false; // false means start at minimum
    
    ble_sensorsim_init(&m_temp_sim_state, &m_temp_sim_cfg);
    
    m_pres_sim_cfg.min          = MIN_PRES_LEVEL;
    m_pres_sim_cfg.max          = MAX_PRES_LEVEL;
    m_pres_sim_cfg.incr         = PRES_LEVEL_INCREMENT;
    m_pres_sim_cfg.start_at_max = false; // false means start at minimum
    
    ble_sensorsim_init(&m_pres_sim_state, &m_pres_sim_cfg);
    
    m_hum_sim_cfg.min          = MIN_HUM_LEVEL;
    m_hum_sim_cfg.max          = MAX_HUM_LEVEL;
    m_hum_sim_cfg.incr         = HUM_LEVEL_INCREMENT;
    m_hum_sim_cfg.start_at_max = false; // false means start at minimum
    
    ble_sensorsim_init(&m_hum_sim_state, &m_hum_sim_cfg);
    
}

/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           api_result_t        event_result)
{
    APP_ERROR_CHECK(event_result);
    
    switch(p_event->event_id)
    {
        case DM_EVT_DEVICE_CONTEXT_STORED:
            dm_security_status_req(p_handle, &m_security_status);
            break;
        case DM_EVT_LINK_SECURED:
            dm_security_status_req(p_handle, &m_security_status);
            break;
        case DM_EVT_DISCONNECTION:
            dm_device_delete(p_handle);
            break;
        default:
            break;
    }

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    uint32_t               err_code;
    dm_init_param_t        init_data;
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    //memset(&init_data, 0, sizeof(init_data));

    init_data.clear_persistent_data = true;

    // Clear all bonded centrals if the Bonds Delete button is pushed.
    //err_code = bsp_button_is_pressed(BOND_DELETE_ALL_BUTTON_ID,&(init_data.clear_persistent_data));
    //APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    
    register_param.sec_param.timeout      = SEC_PARAM_TIMEOUT;
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}



/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for initializing the Connection Parameters module.
 */
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


/**@brief Function for starting timers.
 */
static void timers_start(void)

{
    uint32_t err_code;
    
    // Start application timers.
    err_code = app_timer_start(m_ess_timer_id, ESS_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;

    uint32_t count;

    // Verify if there is any flash access pending, if yes delay starting advertising until
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }

    ble_gap_adv_params_t adv_params;
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
    
    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    //nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}



/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    static ble_gap_master_id_t p_master_id;
    //static ble_gap_sec_keyset_t keys_exchanged;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            //err_code = app_button_enable();
            //APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            dm_device_delete_all(m_app_handle);
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;
        /*
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;
            */
        
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
            break;
        /*
        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            //p_enc_info = keys_exchanged.keys_central.p_enc_key;
            if (p_master_id.ediv == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                //note: second param is confusing...
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
                p_master_id.ediv = p_ble_evt->evt.gap_evt.params.sec_info_request.div;
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;
        */
        case BLE_GAP_EVT_TIMEOUT:
            //dm_device_delete_all(m_app_handle);

            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                //nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
                // Configure buttons with sense level low as wakeup source.
                //nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                // BUTTON_PULL,
                //  NRF_GPIO_PIN_SENSE_LOW);

                // enable buttons to wake-up from power off
                err_code = bsp_buttons_enable( (1 << BOND_DELETE_ALL_BUTTON_ID) );
                APP_ERROR_CHECK(err_code);
                
                // Go to system-off mode (this function will not return; wakeup will cause a reset)
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;
        /*
        case BLE_GATTS_EVT_TIMEOUT:
            dm_device_delete_all(m_app_handle);

            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;
            */
        default:
            // No implementation needed.
            break;
    }
}





/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt); // what does this do?
    ble_ess_on_ble_evt(&m_ess, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            
            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                advertising_start();
            }
            break;
            
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);
    
    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //what is this??
    ble_gap_addr_t addr;
    err_code = sd_ble_gap_address_get(&addr);
    APP_ERROR_CHECK(err_code);
    sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}





/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}



/**@brief Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for application main entry.
 */

int main(void)
{
    // Initialize

    //leds_init();
    
    timers_init();
    
    gpiote_init();
    
    //buttons_init();
    
    ble_stack_init();
    
    scheduler_init();

    device_manager_init();
    
    gap_params_init();
    
    services_init();
    
    advertising_init();

    sensor_sim_init();
    
    conn_params_init();
    
    sec_params_init();
    
    
    // Start execution
    
    timers_start();
    
    advertising_start();
    
    
    // Enter main loop
    
    for (;;)
        
    {
        
        app_sched_execute();
        
        power_manage();
        
    }
    
}

/**
 * @}
 */
#ifndef BLECONFIG_H
#define BLECONFIG_H

//cannot change server database if -
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0

//advertising interval = 1000*0.625ms = 625ms
#define APP_ADV_INTERVAL                MSEC_TO_UNITS(1000, UNIT_0_625_MS)
#define APP_COMPANY_IDENTIFIER			0x11BB
#define MANUFACTURER_NAME 				"Lab11UMich"
#define MODEL_NUMBER 					DEVICE_NAME
#define HARDWARE_REVISION 				"A"
#define FIRMWARE_REVISION 				"0.1"

//advertising timeout sec
#define APP_ADV_TIMEOUT_IN_SECONDS      0

#define UPDATE_RATE     APP_TIMER_TICKS(1000, 0)

//RTC1_Prescale
#define APP_TIMER_PRESCALER             0

#define APP_TIMER_MAX_TIMERS            6

//size of op queues
#define APP_TIMER_OP_QUEUE_SIZE         5


//500ms
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)

//1s
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)

#define SLAVE_LATENCY                   0

//supervisory timeout 4s
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)

//time from initiating event(conn or notify start) to first time param_update c
//called
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)


#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)

//attempts before giving up parameter negotiation
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

//timeout for pairing or sec requests secs
#define SEC_PARAM_TIMEOUT               30

//perform bonding
#define SEC_PARAM_BOND                  1

//no man in the middle
#define SEC_PARAM_MITM                  0

//no i/o capability
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE

//no out of bound data
#define SEC_PARAM_OOB                   0

#define SEC_PARAM_MIN_KEY_SIZE          7

#define SEC_PARAM_MAX_KEY_SIZE          16

#define DEAD_BEEF                       0xDEADBEEF

//max scheduler event size
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)

//max num in scheduler queue
#define SCHED_QUEUE_SIZE                10

#define MAX_PKT_LEN                     20

#define UI_SHORT_UUID             0xE9DB // service UUID
#define UI_SERVICE_COMPTYPE_SHORT_UUID 	0xE9DC // our number characteristic
#define UI_SERVICE_LEN_SHORT_UUID 		0xE9DD // our number characteristic
#define UI_SERVICE_UI_BASE_SHORT_UUID 	0xE9DE // our number characteristic

//the UI service definition
	//COMPTYPE (GZIP = 0x01)
	//LEN		(The length of the total UI, the number of chars is CEIL(LEN/512)
	//UI1
	//UI2
	//...number of 512 byte UI chars


typedef struct ble_ui_s
{
    uint16_t      				conn_handle;                           /**< Handle of the current connection (as provided by the S110 SoftDevice). This will be BLE_CONN_HANDLE_INVALID when not in a connection. */
    uint16_t                    service_handle;                        /**< Handle of DFU Service (as provided by the S110 SoftDevice). */
    uint8_t                     uuid_type;                             /**< UUID type assigned for DFU Service by the S110 SoftDevice. */
    ble_gatts_char_handles_t    comp_handles;                       /**< Handles related to the DFU Packet characteristic. */
    ble_gatts_char_handles_t    len_handles;                       /**< Handles related to the DFU Packet characteristic. */
    ble_gatts_char_handles_t*   ui_handles;                       /**< Handles related to the DFU Packet characteristic. */
    ble_srv_error_handler_t     error_handler;                         /**< Function to be called in case of an error. */
	uint8_t 					comp_value;							//value of compression type
	uint32_t 					len_value;							//total length of UI in bytes
	uint8_t* 					ui_pointer;						//pointer to the start of UI in memory
} ble_ui_t;

#endif

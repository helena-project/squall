#ifndef BLE_ESS_H__

#define BLE_ESS_H__



#include <stdint.h>

#include <stdbool.h>

#include "ble.h"

#include "ble_srv_common.h"

#include "stdio.h"


#define ESS_UUID_BASE {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#define ESS_UUID_SERVICE 0x181A

#define ESS_UUID_TEMP_CHAR 0x2A6E

#define ESS_UUID_PRES_CHAR 0x2A6D

#define ESS_UUID_HUM_CHAR 0x2A6F

#define INIT_TEMP_LEN 2

#define MAX_TEMP_LEN 2

#define INIT_PRES_LEN 4

#define MAX_PRES_LEN 4

#define INIT_HUM_LEN 2

#define MAX_HUM_LEN 2

#define ESS_UUID_ES_MEAS_DESC 0x290C

#define ESS_UUID_ES_TRIGGER_SETTING 0x290D

//trigger condition types

#define TRIG_INACTIVE 0x00

#define TRIG_FIXED_INTERVAL 0x01

#define TRIG_NO_LESS 0x02

#define TRIG_VALUE_CHANGE 0x03

#define TRIG_WHILE_LT 0x04

#define TRIG_WHILE_LTE 0x05

#define TRIG_WHILE_GT 0x06

#define TRIG_WHILE_GTE 0x07

#define TRIG_WHILE_E 0x08

#define TRIG_WHILE_NE 0x09

#define MAX_TRIG_LEN 32

/**@brief ESS event type. */
typedef enum
{
    BLE_ESS_EVT_NOTIFICATION_ENABLED,                                         /**< ESS value notification enabled event. */
    BLE_ESS_EVT_NOTIFICATION_DISABLED,                                        /**< ESS value notification disabled event. */
    BLE_ESS_EVT_NOTIFICATION_CONFIRMED                                        /**< Confirmation of an ESS notification has been received. */
} ble_ess_evt_type_t;

/**@brief ESS event. */
typedef struct
{
    ble_ess_evt_type_t evt_type;                                            /**< Type of event. */
} ble_ess_evt_t;

// Forward declaration of the ble_ess_t type.
typedef struct ble_ess_s ble_ess_t;

// Forward declaration of the ess_trig_set_desc_t type.
typedef struct ess_trig_set_desc_s ess_trig_set_desc_t;

//typedef struct ess_meas_des_s ess_meas_des_t;

/**@brief ESS event handler type. */
typedef void (*ble_ess_evt_handler_t) (ble_ess_t * p_ess, ble_ess_evt_t * p_evt);


/**@brief ESS init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_ess_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the ESS. */
	int16_t 	init_temp_data;
	uint32_t 	init_pres_data;
	uint16_t 	init_hum_data;
	
	uint8_t		temp_trigger_condition;
	uint8_t*	temp_trigger_var_buffer;
	
	uint8_t		pres_trigger_condition;
	uint8_t*	pres_trigger_var_buffer;
	
	uint8_t		hum_trigger_condition;
	uint8_t*	hum_trigger_var_buffer;
    bool						is_notify_supported;		/**< Determines if notifications are supported */
	//bool						is_temp_notify_supported;		/**< Determines if notifications are supported */
    //bool						is_pres_notify_supported;		/**< Determines if notifications are supported */
    //bool						is_hum_notify_supported;		/**< Determines if notifications are supported */
	
} ble_ess_init_t;

/**@brief ESS structure. This contains various status information for the service. */
typedef struct ble_ess_s
{
    ble_ess_evt_handler_t         evt_handler;              /**< Event handler to be called for handling events in the ESS. */
    bool						is_notify_supported;		/**< TRUE if notifications are supported */
    uint16_t                      ess_service_handle;         /**< Handle of ESS (as provided by the BLE stack). */
    ble_gatts_char_handles_t      temp_char_handles;          /**< Handles related to the Temperature characteristic. */
	ble_gatts_char_handles_t      pres_char_handles;          /**< Handles related to the Pressure characteristic. */
	ble_gatts_char_handles_t      hum_char_handles;           /**< Handles related to the Humidity characteristic. */
	
	uint8_t *	temp_val_last;
	uint8_t * pres_val_last;
	uint8_t * hum_val_last;
	
	uint16_t 		temp_trigger_handle;
	uint16_t 		pres_trigger_handle;
	uint16_t 		hum_trigger_handle;
	
	uint8_t                       uuid_type;
    uint16_t                      conn_handle;                /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    
} ble_ess_t;


/**@brief ESS Trigger Setting Descriptor */
//This is mandatory if notifications are supported. Otherwise, it is excluded.
typedef struct ess_trig_set_desc_s
{
	ble_uuid_t * desc_uuid;
	uint8_t	condition;
	uint16_t var_len;
	uint8_t * var_buff;
	
} ess_trig_set_desc_t;


/**@brief ESS Measurement Descriptor */
// This is only mandatory is multiple instances of an ESS Characteristic with the same UUID are supported; Otherwise, it is optional.
/*
 typedef struct ess_meas_des_s
 {
 ble_uuid_t* desc_uuid;
 int flags : 16; // insert bitfield for field
 uint8_t samp_func; // sampling function
 unsigned int meas_per : 24; //measurement period
 unsigned int up_intv : 24; //update interval
 uint8_t app; // application
 uint8_t	meas_unc; //measurement uncertainty
 } ess_meas_desc_t;
 */

/**@brief ESS Configuration Descriptor */
// Mandatory if multiple ES Trigger Setting descriptors are supported; excluded otherwise.
// This this is a C2/C3 requirement, and notifications are not implemented in this version, this descriptor will also not be implemented.
/*
 typedef struct
 {
 //octet?!
 } ess_config_des;
 */


/**@brief Function for initializing the ESS.
 *
 * @param[out]  p_ess       ESS structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_ess_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */

uint32_t ble_ess_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @param[in]   p_ess      ESS structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_ess_on_ble_evt(ble_ess_t * p_ess, ble_evt_t * p_ble_evt);

uint32_t ess_meas_set(ble_ess_t * p_ess);

/**@brief Function for sending ESS  measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed an ESS
 *          measurement. If notification has been enabled, the measurement data is encoded and
 *          sent to the client.
 *
 * @param[in]   p_ess       ESS structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ess_meas_send(ble_ess_t * p_ess);

//set the value of the characteristic
uint32_t ess_char_set(ble_ess_t * p_ess,
					 ble_gatts_char_handles_t * ess_char_handles,
					 uint16_t  ess_char_len,
					 uint8_t * ess_char_value_buff );

//send the value of the characteristic in a notification
uint32_t ess_char_send(ble_ess_t * p_ess,
					  ble_gatts_char_handles_t * ess_char_handles,
					  uint16_t char_len);


uint32_t ble_ess_char_value_update(ble_ess_t * p_ess, ble_gatts_char_handles_t *ess_char_handles, uint8_t * ess_meas_val_last, uint8_t * ess_meas_val, uint16_t char_len);


#endif // BLE_ESS_H__



/** @} */
#ifndef BLE_ESS_H__

#define BLE_ESS_H__



#include <stdint.h>

#include <stdbool.h>

#include "ble.h"

#include "ble_srv_common.h"


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


// Forward declaration of the ble_ess_t type.
typedef struct ble_ess_s ble_ess_t;

typedef struct ess_meas_des_s ess_meas_des_t;



/**@brief ESS event handler type. */
typedef void (*ble_ess_evt_handler_t) (ble_ess_t * p_ess, uint8_t new_state);



/**@brief ESS init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_ess_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the ESS. */
    bool						  is_notify_supported;		/**< Determines if notifications are supported */

} ble_ess_init_t;


/**@brief ESS structure. This contains various status information for the service. */
typedef struct ble_ess_s
{
    ble_ess_evt_handler_t         evt_handler;              /**< Event handler to be called for handling events in the ESS. */
    bool						  is_notify_supported;		/**< TRUE if notifications are supported */
    uint16_t                      ess_service_handle;         /**< Handle of ESS (as provided by the BLE stack). */
    ble_gatts_char_handles_t      temp_char_handles;          /**< Handles related to the Temperature characteristic. */
	ble_gatts_char_handles_t      pres_char_handles;          /**< Handles related to the Pressure characteristic. */
	//ble_gatts_char_handles_t      pres_char_desc_handles;          /**< Handles related to the Pressure characteristic. */
	ble_gatts_char_handles_t      hum_char_handles;           /**< Handles related to the Humidity characteristic. */
	uint8_t                       uuid_type;
    uint16_t                      conn_handle;                /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    
} ble_ess_t;

/**@brief ESS Measurement Descriptor */
// This is only mandatory is multiple instances of an ESS Characteristic with the same UUID are supported; Otherwise, it is optional.

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
 


/**@brief ESS Trigger Setting Descriptor */
//This is mandatory if notifications are supported. Otherwise, it is excluded.
//Since notifications are optional, and in this version its function is not implemented, this descriptor will also not be implemented.
/*
 typedef struct
 {
 //octet?!
 } ess_trig_set_des;
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





/**@brief Environmental Sensing Service event. */
//typedef struct
//{
//  ble_ess_evt_type_t evt_type;         /**< Type of event. */
//} ble_ess_evt_t;



/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note For the requirements in the ESS specification to be fulfilled,
 *       ble_ess_battery_level_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client. ~ *~ *~ *~ ~* ~IGNORE *~*~*~*~ *~ *~
 *
 * @param[in]   p_ess      ESS structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */

void ble_ess_on_ble_evt(ble_ess_t * p_ess, ble_evt_t * p_ble_evt);


uint32_t ble_ess_on_temp_change(ble_ess_t * p_ess, uint8_t temp_state);

uint32_t ble_ess_on_pres_change(ble_ess_t * p_ess, uint8_t pres_state);

uint32_t ble_ess_on_hum_change(ble_ess_t * p_ess, uint8_t hum_state);

#endif // BLE_ESS_H__



/** @} */
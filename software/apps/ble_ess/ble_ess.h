/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.

 *

 * The information contained herein is property of Nordic Semiconductor ASA.

 * Terms and conditions of usage are described in detail in NORDIC

 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.

 *

 * Licensees are granted free, non-transferable use of the information. NO

 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from

 * the file.

 *

 */



/** @file

 *

 * @defgroup ble_sdk_srv_ess Battery Service

 * @{

 * @ingroup ble_sdk_srv

 * @brief Battery Service module.

 *

 * @details This module implements the Battery Service with the Battery Level characteristic.

 *          During initialization it adds the Battery Service and Battery Level characteristic

 *          to the BLE stack dataesse. Optionally it can also add a Report Reference descriptor

 *          to the Battery Level characteristic (used when including the Battery Service in

 *          the HID service).

 *

 *          If specified, the module will support notification of the Battery Level characteristic

 *          through the ble_ess_battery_level_update() function.

 *          If an event handler is supplied by the application, the Battery Service will

 *          generate Battery Service events to the application.

 *

 * @note The application must propagate BLE stack events to the Battery Service module by calling

 *       ble_ess_on_ble_evt() from the @ref softdevice_handler callback.

 *

 * @note Attention!

 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile

 *  qualification listings, this section of source code must not be modified.

 */



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


// Forward declaration of the ble_ess_t type.
typedef struct ble_ess_s ble_ess_t;


/**@brief ESS event handler type. */
typedef void (*ble_ess_evt_handler_t) (ble_ess_t * p_ess, uint8_t new_state);



/**@brief ESS init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_ess_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the ESS. */
} ble_ess_init_t;


/**@brief ESS structure. This contains various status information for the service. */
typedef struct ble_ess_s
{
    //ble_ess_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the ESS. */
    uint16_t                      ess_service_handle;                 /**< Handle of ESS (as provided by the BLE stack). */
    ble_gatts_char_handles_t      temp_char_handles;          /**< Handles related to the Temperature characteristic. */
	ble_gatts_char_handles_t      pres_char_handles;          /**< Handles related to the Pressure characteristic. */
	ble_gatts_char_handles_t      hum_char_handles;          /**< Handles related to the Humidity characteristic. */
	uint8_t                     uuid_type;
	//uint8_t                       battery_level_last;             /**< Last Battery Level measurement passed to the Battery Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */

} ble_ess_t;


/**@brief ES Measurement Descriptor */
// This is only mandatory is multiple instances of an ESS Characteristic with the same UUID are supported; Otherwise, it is optional.
/*
typedef struct
{
    //insert bitfield for flags
    uint8_t samp_func; // sampling function
    uint24_t meas_per; //measurement period
    uint24_t up_intv; //update interval
    uint8_t app; // application
    uint8_t	meas_unc; //measurement uncertainty

} es_meas_des;
*/


/**@brief ES Trigger Setting Descriptor */
//This is mandatory if notifications are supported. Otherwise, it is excluded.
//Since notifications are optional, and in this version its function is not implemented, this descriptor will also not be implemented.
/*
typedef struct
{
    //octet?!

} es_trig_set_des;
*/

/**@brief ES Configuration Descriptor */
// Mandatory if multiple ES Trigger Setting descriptors are supported; excluded otherwise.
// This this is a C2/C3 requirement, and notifications are not implemented in this version, this descriptor will also not be implemented.
/*
typedef struct
{
    //octet?!

} es_config_des;
*/


/**@brief Function for initializing the ESSe.

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

  //  ble_ess_evt_type_t evt_type;                                  /**< Type of event. */

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

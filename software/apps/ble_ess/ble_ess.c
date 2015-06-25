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



/* Attention!

 *  To maintain compliance with Nordic Semiconductor ASAís Bluetooth profile

 *  qualification listings, this section of source code must not be modified.

 */



#include "ble_ess.h"

#include <string.h>

#include "nordic_common.h"

#include "ble_srv_common.h"

#include "app_util.h"





/**@brief Function for handling the Connect event.

 *

 * @param[in]   p_ess       Environmental Service structure.

 * @param[in]   p_ble_evt   Event received from the BLE stack.

 */

static void on_connect(ble_ess_t * p_ess, ble_evt_t * p_ble_evt)

{

    p_ess->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

}





/**@brief Function for handling the Disconnect event.

 *

 * @param[in]   p_ess       Environmental Service structure.

 * @param[in]   p_ble_evt   Event received from the BLE stack.

 */

static void on_disconnect(ble_ess_t * p_ess, ble_evt_t * p_ble_evt)

{

    UNUSED_PARAMETER(p_ble_evt);

    p_ess->conn_handle = BLE_CONN_HANDLE_INVALID;

}





void ble_ess_on_ble_evt(ble_ess_t * p_ess, ble_evt_t * p_ble_evt)

{

    switch (p_ble_evt->header.evt_id)

    {

        case BLE_GAP_EVT_CONNECTED:

            on_connect(p_ess, p_ble_evt);

            break;


        case BLE_GAP_EVT_DISCONNECTED:

            on_disconnect(p_ess, p_ble_evt);

            break;


        //case BLE_GATTS_EVT_WRITE:

        //    on_write(p_ess, p_ble_evt);

        //    break;


        default:

            // No implementation needed.

            break;

    }

}


/**@brief Function for adding a characteristic.

 *

 * @param[in]   p_ess        Environmental Service structure.

 * @param[in]   p_ess_init   Information needed to initialize the service.

 *

 * @return      NRF_SUCCESS on success, otherwise an error code.

 */

static uint32_t ess_char_add(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init, uint32_t ess_char_uuid, ble_gatts_char_handles_t *const ess_char_handles)

{

    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write   = 0;
    char_md.char_props.notify = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    //char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_ess->uuid_type;
    ble_uuid.uuid = ess_char_uuid;
    //ble_uuid.uuid = 0x2A1C;


    memset(&attr_md, 0, sizeof(attr_md));


// According to ESS_SPEC_V10, the read operation on cccd should be possible without
// authentication.

//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm); 
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = NULL;


    return sd_ble_gatts_characteristic_add(p_ess->service_handle, &char_md,
                                            &attr_char_value,
                                            ess_char_handles);


}






/**@brief Function for adding the Temperature characteristic.

 *

 * @param[in]   p_ess        Environmental Service structure.

 * @param[in]   p_ess_init   Information needed to initialize the service.

 *

 * @return      NRF_SUCCESS on success, otherwise an error code.

 */
 /*
static uint32_t temp_char_add(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)

{

    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write   = 0;
    char_md.char_props.notify = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    //char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_ess->uuid_type;
    ble_uuid.uuid = ESS_UUID_TEMP_CHAR;
    //ble_uuid.uuid = 0x2A1C;


    memset(&attr_md, 0, sizeof(attr_md));


// According to ESS_SPEC_V10, the read operation on cccd should be possible without
// authentication.

//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm); 
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = 0x8;


    return sd_ble_gatts_characteristic_add(p_ess->service_handle, &char_md,
                                            &attr_char_value,
                                            &p_ess->temp_char_handles);


}
*/


/**@brief Function for adding the Pressure characteristic.

 *

 * @param[in]   p_ess        Environmental Service structure.

 * @param[in]   p_ess_init   Information needed to initialize the service.

 *

 * @return      NRF_SUCCESS on success, otherwise an error code.

 */
/*
static uint32_t pres_char_add(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)

{

    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write   = 0;
    char_md.char_props.notify = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    //char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_ess->uuid_type;
    ble_uuid.uuid = ESS_UUID_PRES_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));


// According to ESS_SPEC_V10, the read operation on cccd should be possible without
// authentication.

//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm); 
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = 0x5;


    return sd_ble_gatts_characteristic_add(p_ess->service_handle, &char_md,
                                            &attr_char_value,
                                            &p_ess->pres_char_handles);


}
*/




uint32_t ble_ess_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)

{

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure

    //p_ess->evt_handler               = p_ess_init->evt_handler;
    p_ess->conn_handle               = BLE_CONN_HANDLE_INVALID;


    // Add service

    ble_uuid128_t base_uuid = {ESS_UUID_BASE};


    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ess->uuid_type);

    if (err_code != NRF_SUCCESS)

    {

        return err_code;

    }


    ble_uuid.type = p_ess->uuid_type;

    ble_uuid.uuid = ESS_UUID_SERVICE;

        err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, 
                                        &ble_uuid, 
                                        &p_ess->service_handle);

    if (err_code != NRF_SUCCESS)

    {

        return err_code;

    }

    err_code = ess_char_add(p_ess, p_ess_init, ESS_UUID_TEMP_CHAR, &p_ess->temp_char_handles);

    if (err_code != NRF_SUCCESS)

    {

        return err_code;

    }

    err_code = ess_char_add(p_ess, p_ess_init, ESS_UUID_PRES_CHAR, &p_ess->pres_char_handles);

    if (err_code != NRF_SUCCESS)

    {

        return err_code;

    }

    /*
    err_code = temp_char_add(p_ess, p_ess_init);

    if (err_code != NRF_SUCCESS)

    {

        return err_code;

    }

    

    err_code = pres_char_add(p_ess, p_ess_init);

    if (err_code != NRF_SUCCESS)

    {

        return err_code;

    }

    */

    return NRF_SUCCESS;

}

uint32_t ble_ess_on_temp_change(ble_ess_t * p_ess, uint8_t temp_state)
{

    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(temp_state);
    
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_ess->temp_char_handles.value_handle;
    params.p_data = &temp_state;
    params.p_len = &len;
    
    return sd_ble_gatts_hvx(p_ess->conn_handle, &params);
    //return sd_ble_gatts_value_set() = 80;
}

uint32_t ble_ess_on_pres_change(ble_ess_t * p_ess, uint8_t pres_state)
{

    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(pres_state);
    
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_ess->pres_char_handles.value_handle;
    params.p_data = &pres_state;
    params.p_len = &len;
    
    return sd_ble_gatts_hvx(p_ess->conn_handle, &params);
    //return sd_ble_gatts_value_set() = 80;
}
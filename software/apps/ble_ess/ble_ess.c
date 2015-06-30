
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

/**@brief Function for adding a pressure characteristic.
 *
 * @param[in]   p_ess        Environmental Service structure.
 * @param[in]   p_ess_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
/* Every Characteristc consists of at least 2 attributes:
 *   1) Characteristic Declaration Attribute
 *       which consists of 3 sub-properties:
 *          a) The Characteristic Properties
 *          b) The Characteristic Value Handle
 *          c) The Characteristc UUID
 *       
 *   2) Characteristic Value Attribute
 *
 *   And sometimes a third attribute:
 *   3) Descriptors Attribute
 *       - If used, this must be placed after the characteristc value attribute
 *
 *   These 2 (or 3) attributes together make up the characteristic description
*/
static uint32_t ess_char_add(ble_ess_t * p_ess, 
                            const ble_ess_init_t * p_ess_init,
                            int ESS_CHAR_UUID,
                            ble_gatts_char_handles_t * ess_char_handles,
                            uint8_t * fake_data_p,
                            uint16_t init_char_len,
                            uint16_t max_char_len)
{
    
    
    ble_gatts_char_md_t char_md; // char metadata (part of Characteristic Properties)
    ble_gatts_attr_t    attr_char_value; //Characteristic Value Attribute- the actual data!
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md; // attribute metadata
    ble_gatts_attr_md_t cccd_md;
    //ess_meas_desc_t     attr_char_desc;
    
    memset(&char_md, 0, sizeof(char_md));
    
    /***** set characteristic properties *****/
    char_md.char_props.read   = 1; // mandatory
    char_md.char_props.write   = 0; // excluded
    char_md.char_props.write_wo_resp   = 0; // excluded
    char_md.char_props.auth_signed_wr   = 0; // excluded
    char_md.char_props.notify = 1; // optional
    char_md.char_props.indicate   = 0; // excluded
    char_md.char_props.broadcast   = 0; // excluded
    
    /***** external properties are optional ****/
    char_md.char_ext_props.reliable_wr = 0; // not mentioned in spec?
    char_md.char_ext_props.wr_aux = 0; // mentioned as past of normal properties?
    
    char_md.p_char_user_desc  = NULL; // null if user description is not required () optional)
    

    /******** if notify is enabled ******/
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    char_md.p_char_pf         = NULL; // presentation format structure not mentioned not included in spec, so null
    char_md.p_user_desc_md    = NULL; // not mentioned in spec
    char_md.p_cccd_md         = &cccd_md; // not mentioned in spec
    char_md.p_sccd_md         = NULL; // not mentioned in spec
    
    ble_uuid.type = p_ess->uuid_type;
    ble_uuid.uuid = ESS_CHAR_UUID;
    //ble_uuid.uuid = 0x2A1C;
    
    /* I currently cannot find any information about required characteristic value attribute metadata */
    memset(&attr_md, 0, sizeof(attr_md));
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    /* 2) The Characteristic Value Attribute consists of the actual data */
    /* for now, we will input fake data */
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid; //the type for this attribute is always the same UUID found in the characteristic's declaration value field
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = init_char_len;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = max_char_len;
    attr_char_value.p_value   = fake_data_p;
    
    //return
    return sd_ble_gatts_characteristic_add(p_ess->ess_service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           ess_char_handles);

    /*
    memset(&attr_char_desc, 0, sizeof(attr_char_desc));
    
    BLE_UUID_BLE_ASSIGN(ble_uuid, ESS_UUID_ES_MEAS_DESC);

    attr_char_desc.desc_uuid = &ble_uuid;
    attr_char_desc.flags = 0;
    attr_char_desc.samp_func = 0;
    attr_char_desc.meas_per = 0;
    attr_char_desc.up_intv = 0;
    attr_char_desc.app = 0;
    attr_char_desc.meas_unc = 0;

    uint16_t id = ESS_UUID_ES_MEAS_DESC;
    uint16_t *ble_uuid_ac = (uint16_t*)&id;

    return sd_ble_gatts_descriptor_add(ESS_CHAR_UUID,
                                &attr_char_desc,
                               &ble_uuid_ac);
    */                           
    
}



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
                                        &p_ess->ess_service_handle);
    
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
    
    err_code = hum_char_add(p_ess, p_ess_init);
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

    uint8_t *fake_data_ptr;
    
    int16_t fake_data_t = 123;
    fake_data_ptr = (uint8_t*)&fake_data_t;

     err_code = ess_char_add(p_ess, p_ess_init, ESS_UUID_TEMP_CHAR, &p_ess->temp_char_handles, fake_data_ptr, INIT_TEMP_LEN, MAX_TEMP_LEN);
     if (err_code != NRF_SUCCESS)
     {
     return err_code;
     }
    uint32_t fake_data_pr = 456;
    fake_data_ptr = (uint8_t*)&fake_data_pr;

     err_code = ess_char_add(p_ess, p_ess_init, ESS_UUID_PRES_CHAR, &p_ess->pres_char_handles, fake_data_ptr, INIT_PRES_LEN, MAX_PRES_LEN);
     if (err_code != NRF_SUCCESS)
     {
     return err_code;
     }

    uint16_t fake_data_h = 789;
    fake_data_ptr = (uint8_t*)&fake_data_h;

     err_code = ess_char_add(p_ess, p_ess_init, ESS_UUID_HUM_CHAR, &p_ess->hum_char_handles, fake_data_ptr, INIT_HUM_LEN, MAX_HUM_LEN);
     if (err_code != NRF_SUCCESS)
     {
     return err_code;
     }
     
    
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

uint32_t ble_ess_on_hum_change(ble_ess_t * p_ess, uint8_t hum_state)
{
    
    
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(hum_state);
    
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_ess->hum_char_handles.value_handle;
    params.p_data = &hum_state;
    params.p_len = &len;
    
    return sd_ble_gatts_hvx(p_ess->conn_handle, &params);
    //return sd_ble_gatts_value_set() = 80;
}
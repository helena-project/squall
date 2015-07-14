#include "ble_ess.h"

#include <string.h>

#include "nordic_common.h"

#include "ble_srv_common.h"

#include "app_util.h"

#define INVALID_TEMP_LEVEL 0xFFFF

#define INVALID_PRES_LEVEL 0xFFFFFFFF

#define INVALID_HUM_LEVEL 0xFFFF

void ess_copybuff( uint8_t * buff_1, uint8_t * buff_2, uint32_t length){

    //*buff_2 = *buff_1;

    buff_2[0] = buff_1[0];

    uint32_t len = 0;

    while (len < length) {
        buff_2[len++] = buff_1[len];
    }
}

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

static bool is_char_value_handle(ble_gatts_evt_hvc_t * p_hvc, ble_ess_t * p_ess){
    if ((p_hvc->handle == p_ess->temp_char_handles.value_handle) ||
        (p_hvc->handle == p_ess->pres_char_handles.value_handle) ||
        (p_hvc->handle == p_ess->hum_char_handles.value_handle) ){
        return true;
    }
    return false;
}


/**@brief Function for handling the HVC event.
 *
 * @details Handles HVC events from the BLE stack.
 *
 * @param[in]   p_ess       ESS structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_hvc(ble_ess_t * p_ess, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_hvc_t * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;
    
    if (is_char_value_handle(p_hvc, p_ess))
    {
        ble_ess_evt_t evt;
        
        evt.evt_type = BLE_ESS_EVT_NOTIFICATION_CONFIRMED;
        p_ess->evt_handler(p_ess, &evt);
    }
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
            
        case BLE_GATTS_EVT_HVC:
            on_hvc(p_ess, p_ble_evt);
            
        default:
            // No implementation needed.
            break;
    }
}

static uint8_t encode_buffer(uint8_t * p_encoded_buffer, const uint8_t * trigger_condition, const uint8_t * trigger_var_buff){
    uint8_t len = 0;
    p_encoded_buffer[len++] = *trigger_condition;
    p_encoded_buffer[len++] = *trigger_var_buff;
    
    return len;
    
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
                            uint16_t max_char_len,
                            uint8_t * trigger_condition,
                            uint8_t * trigger_val,
                            uint16_t * trigger_handle,
                            uint8_t * condition,
                            uint8_t * var_buff)
{
    
    uint32_t err_code;
    
    ble_gatts_char_md_t char_md; // char metadata (part of Characteristic Properties)
    ble_gatts_attr_t    attr_char_value; //Characteristic Value Attribute- the actual data!
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md; // attribute metadata
    ble_gatts_attr_md_t cccd_md;
    //ess_meas_desc_t     attr_char_desc;
    ble_gatts_attr_t    trigger_des;
    
    memset(&char_md, 0, sizeof(char_md));
    
    /***** set characteristic properties *****/
    char_md.char_props.read   = 1; // mandatory
    //char_md.char_props.notify = 1;
    char_md.char_props.notify = p_ess_init->is_notify_supported; // optional
    p_ess->is_notify_supported = char_md.char_props.notify;
    //char_md.char_props.write   = 0; // excluded
    //char_md.char_props.write_wo_resp   = 0; // excluded
    //char_md.char_props.auth_signed_wr   = 0; // excluded
    //char_md.char_props.indicate   = 0; // excluded
    //char_md.char_props.broadcast   = 0; // excluded
    
    /***** external properties are optional ****/
    char_md.char_ext_props.reliable_wr = 0; // not mentioned in spec?
    char_md.char_ext_props.wr_aux = 0; // mentioned as past of normal properties?
    
    
    /******** if notify is enabled ******/
    memset(&cccd_md, 0, sizeof(cccd_md));
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    //cccd_md.write_perm = p_ess_init->ess_meas_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    char_md.p_char_user_desc  = NULL;
    
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
    err_code = sd_ble_gatts_characteristic_add(p_ess->ess_service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              ess_char_handles);
    
    if (err_code != NRF_SUCCESS){ return err_code; }
    
    
    /***** If notification is enabled, add the trigger setting ***/
    //Note: Trigger setting should have been initialized in p_ess_init
   //printf("yo");
    if (p_ess_init->is_notify_supported == true){
        
        *condition = *trigger_condition;
        memcpy(var_buff, trigger_val, max_char_len);


        memset(&trigger_des, 0, sizeof(trigger_des));
        
        //uint16_t init_len = encode_buffer(trigger_des.p_value, &trigger_condition, trigger_var_buff);
        
        BLE_UUID_BLE_ASSIGN(ble_uuid, ESS_UUID_ES_TRIGGER_SETTING);
        trigger_des.p_uuid = &ble_uuid;
        trigger_des.p_attr_md = NULL;
        trigger_des.init_len = MAX_TRIG_LEN;
        //trigger_des.init_len = init_len;
        
        trigger_des.init_offs = 0;
        trigger_des.max_len = MAX_TRIG_LEN;
        //printf("hi");
        trigger_des.p_value = var_buff;

        //var_buff = trigger_var_buff;
        //ess_copybuff(trigger_var_buff, var_buff, var_len);

        err_code = sd_ble_gatts_descriptor_add(BLE_GATT_HANDLE_INVALID, &trigger_des, trigger_handle);
    

       printf("hi");
    }
    
    if (err_code != NRF_SUCCESS){ return err_code; }
    
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
    
    return NRF_SUCCESS;
    
}


uint32_t ble_ess_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)

{
    
    volatile uint32_t   err_code;
    ble_uuid_t ble_uuid;
    
    // Initialize service structure
    p_ess->evt_handler               = p_ess_init->evt_handler;
    //p_ess->evt_handler               = NULL;
    
    p_ess->conn_handle               = BLE_CONN_HANDLE_INVALID;
    
    
    //BLE_UUID_BLE_ASSIGN(ble_uuid, ESS_UUID_SERVICE);
    
    //p_ess->uuid_type = ble_uuid.type;
    
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
    
    //Initial data pointer
    uint8_t *init_data_ptr;
    *(p_ess->temp_val_last) = INVALID_TEMP_LEVEL;
    *(p_ess->pres_val_last) = INVALID_PRES_LEVEL;
    *(p_ess->hum_val_last) = INVALID_HUM_LEVEL;
    (p_ess->temp_val_last) = (uint8_t*)&(p_ess_init->init_temp_data);
    (p_ess->pres_val_last) = (uint8_t*)&(p_ess_init->init_pres_data);
    (p_ess->hum_val_last) = (uint8_t*)&(p_ess_init->init_hum_data);
    
    //Add temperature characteristic
    init_data_ptr = (uint8_t*)&(p_ess_init->init_temp_data);
    
    err_code = ess_char_add(p_ess, p_ess_init, ESS_UUID_TEMP_CHAR, &p_ess->temp_char_handles, init_data_ptr, INIT_TEMP_LEN, MAX_TEMP_LEN, 
        &(p_ess_init->temp_trigger_condition),  (uint8_t*)&(p_ess_init->temp_trigger_val), &p_ess->temp_trigger_handle, &(p_ess->temp_trigger_val_cond), p_ess->temp_trigger_val_buff);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    //printf("hi");

    // Add pressure characteristic
    init_data_ptr = (uint8_t*)&(p_ess_init->init_pres_data);
    
    err_code = ess_char_add(p_ess, p_ess_init, ESS_UUID_PRES_CHAR, &p_ess->pres_char_handles, init_data_ptr, INIT_PRES_LEN, MAX_PRES_LEN, 
        &(p_ess_init->pres_trigger_condition),  (uint8_t*)&(p_ess_init->pres_trigger_val), &p_ess->pres_trigger_handle, &(p_ess->pres_trigger_val_cond), p_ess->pres_trigger_val_buff);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add humidity characteristic
    init_data_ptr = (uint8_t*)&(p_ess_init->init_pres_data);
    
    err_code = ess_char_add(p_ess, p_ess_init, ESS_UUID_HUM_CHAR, &p_ess->hum_char_handles, init_data_ptr, INIT_HUM_LEN, MAX_HUM_LEN, 
       &(p_ess_init->hum_trigger_condition),  (uint8_t*)&(p_ess_init->hum_trigger_val), &p_ess->hum_trigger_handle, &(p_ess->hum_trigger_val_cond), p_ess->hum_trigger_val_buff);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    
    return NRF_SUCCESS;
    
}

uint32_t ess_meas_set(ble_ess_t * p_ess){
    
    uint32_t err_code;
    uint8_t * fake_data_p = (uint8_t*)(0x01);
    
    
    err_code = ess_char_set(p_ess, &p_ess->temp_char_handles, 2, fake_data_p);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = ess_char_set(p_ess, &p_ess->pres_char_handles, 4, fake_data_p);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = ess_char_set(p_ess, &p_ess->hum_char_handles, 2, fake_data_p);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
    
}

uint32_t ess_meas_send(ble_ess_t * p_ess){
    
    uint32_t err_code;
    err_code = ess_char_send(p_ess, &p_ess->temp_char_handles, 2);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = ess_char_send(p_ess, &p_ess->pres_char_handles, 4);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = ess_char_send(p_ess, &p_ess->hum_char_handles, 2);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}


uint32_t ess_char_set(ble_ess_t * p_ess,
                     ble_gatts_char_handles_t * ess_char_handles,
                     uint16_t  ess_char_len,
                     uint8_t * ess_char_value_buff )
{
    
    uint8_t *new_ess_char_value_buff;
    *(new_ess_char_value_buff) = *(ess_char_value_buff) + 1;
    
    return sd_ble_gatts_value_set(ess_char_handles->value_handle,
                                 0, &ess_char_len,
                                 new_ess_char_value_buff);
}

uint32_t ess_char_send(ble_ess_t * p_ess,
                      ble_gatts_char_handles_t * ess_char_handles,
                      uint16_t char_len)
{
    uint32_t   err_code;
    uint8_t * p_data_buff;
    uint16_t * ess_char_len = (uint16_t*)&char_len;
    
    err_code = sd_ble_gatts_value_get(ess_char_handles->value_handle,
                                     0,
                                     ess_char_len,
                                     p_data_buff);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_gatts_hvx_params_t params;
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = ess_char_handles->value_handle;
    params.p_len = &char_len;
    params.p_data = p_data_buff;
    
    
    return sd_ble_gatts_hvx(p_ess->conn_handle, &params);
}

int intcmp( uint8_t * buff_1, uint8_t * buff_2, uint16_t length, bool is_signed){

    int len = length-1;

    if (is_signed){
        
        if ((buff_1[len] >> 7) != (buff_2[len] >> 7) ){

            if ((buff_2[len] >> 7) == 0) return -1;
            else return 1;

        } 

    }

    while( len >= 0){
        if (buff_1[len] > buff_2[len]) return 1;
        else if (buff_1[len] < buff_2[len]) return -1;
        len--;
    }

    return 0;


}


uint32_t ble_ess_char_value_update(ble_ess_t * p_ess, ble_gatts_char_handles_t *ess_char_handles, uint8_t * ess_meas_val_last, 
    uint8_t * ess_meas_val, uint16_t char_len, uint8_t condition, uint8_t * var_buff, bool is_signed)
{
    
    uint32_t err_code = NRF_SUCCESS;
    
    //if (*ess_meas_val != *ess_meas_val_last)
    //{
        uint16_t len = char_len;
        // Save new battery value
        //*ess_meas_val_last = *ess_meas_val;
        
        // Update database
        
        err_code = sd_ble_gatts_value_set(ess_char_handles->value_handle,
                                          0,
                                          &char_len,
                                          ess_meas_val);

        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        
        // Send value if connected and notifying
        if ((p_ess->conn_handle != BLE_CONN_HANDLE_INVALID) && p_ess->is_notify_supported)
        {
            
           if( is_notification_needed(condition, var_buff, ess_meas_val, ess_meas_val_last, char_len, is_signed) ){
                //send the notification
                ble_gatts_hvx_params_t hvx_params;
                
                memset(&hvx_params, 0, sizeof(hvx_params));
                
                len = sizeof(uint8_t);
                
                hvx_params.handle = ess_char_handles->value_handle;
                hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
                hvx_params.offset = 0;
                hvx_params.p_len  = &len;
                hvx_params.p_data = ess_meas_val;
                

                err_code = sd_ble_gatts_hvx(p_ess->conn_handle, &hvx_params);
            }
            
        }
        
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
        
        
    //}
    
    ess_meas_val_last = ess_meas_val;
    
    
    return err_code;
}


bool is_notification_needed(uint8_t condition, uint8_t * operand, uint8_t * ess_meas_val_new, uint8_t * ess_meas_val_old, uint8_t char_len, bool is_signed){

    bool notif_needed = true;
        
        if(condition == TRIG_INACTIVE){
            notif_needed = false;
        }

        else if (condition == TRIG_FIXED_INTERVAL){
            notif_needed = false;
            //break;
        }

        else if (condition == TRIG_NO_LESS){
            notif_needed = true;
            //break;
        }

        else if (condition == TRIG_VALUE_CHANGE){
            if ( *ess_meas_val_new != *ess_meas_val_old){
                notif_needed = true;
            }
            else {
                notif_needed = false;
            }
            // break;
        }

        else {
            int n = intcmp(ess_meas_val_new, operand, char_len, is_signed);

            if (condition == TRIG_WHILE_LT){
                
                if (n < 0){
                    notif_needed = true;
                }
                
                else {
                    notif_needed = false;
                }
            }
                //break;

            else if (condition == TRIG_WHILE_LTE){
                if (n <= 0){
                    notif_needed = true;
                }
                else{
                    notif_needed = false;
                } 
            }
                //break;

            else if (condition == TRIG_WHILE_GT){

                if (n>0){
                   notif_needed = true;
                }
                else{
                    notif_needed = false;
                }
            }
                //break;

            else if (condition == TRIG_WHILE_GTE){
                if (n>=0){
                    notif_needed = true;
                }
                else{
                    notif_needed = false;
                }
            }
                //break;

            else if (condition == TRIG_WHILE_E){
                if(n==0){
                    notif_needed = true;
                }
                else{
                    notif_needed = false;
                }
            }
               // break;

            else if (condition == TRIG_WHILE_NE){
                if(n!=0){
                    notif_needed = true;
                }
                else {
                    notif_needed = false;
                }
            }



        }
            //break;

        //default:
            //break;
    //}
    

    return notif_needed;
}
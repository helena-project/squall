#ifndef BLECONFIG_H
#define BLECONFIG_H

#define CUSTOM_SERVICE_SHORT_UUID             0xE9DB // service UUID
#define CUSTOM_SERVICE_CHAR_NUMBER_SHORT_UUID 0xE9DC // our number characteristic

#define UPDATE_RATE APP_TIMER_TICKS(1000, 0)

typedef struct ble_cstm_s
{
    uint16_t                     service_handle;                        /**< Handle of DFU Service (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t     num_handles;                       /**< Handles related to the DFU Packet characteristic. */
    uint16_t                     num_value;                             /** Value of num characteristic */
} ble_cstm_t;

#endif

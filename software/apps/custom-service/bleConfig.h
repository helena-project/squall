#ifndef BLECONFIG_H
#define BLECONFIG_H

#define CUSTOM_SERVICE_SHORT_UUID             0xE9DB // service UUID
#define CUSTOM_SERVICE_CHAR_NUMBER_SHORT_UUID 0xE9DC // our number characteristic

typedef struct ble_cstm_s
{
    uint16_t                     num_value;                             /** Value of num characteristic */
} ble_cstm_t;

#endif

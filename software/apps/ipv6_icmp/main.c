/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup iot_sdk_icmp_app main.c
 * @{
 * @ingroup iot_sdk_app_ipv6
 * @brief This file contains the source code for Nordic's IPv6 ICMPv6 sample application.
 *
 * This example demonstrates how Nordic's IPv6 stack can be used for sending and receiving ICMPv6 packets.
 * Sending one of four packets is triggered by user buttons on kit. LEDS blinking indicates that response has been received.
 *
 * All available packets are:
 * - Neighbor Solicitation                              | Button 1
 * - Router Solicitation                                | Button 2
 * - Echo Request to all node multicast address ff02::1 | Button 3
 * - Echo Request to remote node (Router)               | Button 4
 *
 * Note: In order to get response for Router Solicitation, peer node has to act as a Router.
 *
 *  Below is MSC explaining the data flow at ICMPv6 level.
 *
 *    +------------------+                                                +------------------+
 *    | Node             |                                                | Router           |
 *    |(this application)|                                                |                  |
 *    +------------------+                                                +------------------+
 *           |                                                                   |
 *           |                                                                   |
 *        ---|------------------------- Interface is UP  ------------------------|---
 *           |                                                                   |
 *        ---|-------------------- Button 1 has been pushed ---------------------|---
 *           |                                                                   |
 *           |[ICMPv6 Neighbor Solicitation to router's link-local address]      |
 *           |------------------------------------------------------------------>|
 *           |                                                                   |
 *           |       [ICMPv6 Neighbor Advertisement to node's link-local address]|
 *           |<------------------------------------------------------------------|
 *           |                                                                   |
 *     LEDS BLINKING                                                             |
 *           |                                                                   |
 *        ---|-------------------- Button 2 has been pushed ---------------------|---
 *           |                                                                   |
 *           |[ICMPv6 Router Solicitation to all routers multicast address]      |
 *           |------------------------------------------------------------------>|
 *           |                                                                   |
 *           |         [ICMPv6 Router Advertisement to node's link-local address]|
 *           |<------------------------------------------------------------------|
 *           |                                                                   |
 *     LEDS BLINKING                                                             |
 *           |                                                                   |
 *        ---|-------------------- Button 3 has been pushed ---------------------|---
 *           |                                                                   |
 *           |[ICMPv6 Echo Request to all nodes multicast address]               |
 *           |------------------------------------------------------------------>|
 *           |                                                                   |
 *           |                [ICMPv6 Echo Response to node's link-local address]|
 *           |<------------------------------------------------------------------|
 *           |                                                                   |
 *     LEDS BLINKING                                                             |
 *           |                                                                   |
 *        ---|-------------------- Button 4 has been pushed ---------------------|---
 *           |                                                                   |
 *           |[ICMPv6 Echo Request to router's link-local address]               |
 *           |------------------------------------------------------------------>|
 *           |                                                                   |
 *           |                [ICMPv6 Echo Response to node's link-local address]|
 *           |<------------------------------------------------------------------|
 *           |                                                                   |
 *     LEDS BLINKING                                                             |
 *           |                                                                   |
 *           -                                                                   -
 */


#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "mem_manager.h"
#include "ble_advdata.h"
#include "app_trace.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_gpiote.h"
#include "ble_ipsp.h"
#include "ipv6_api.h"
#include "icmp6_api.h"
#include "udp_api.h"
#include "nrf_temp.h"

#define DEVICE_NAME                   "IPv6ICMP"                                                    /**< Device name used in BLE undirected advertisement. */

#define APP_GPIOTE_MAX_USERS          1                                                             /**< Maximum GPIOTE number of users. */

#define ADVERTISING_LED               LED_0                                                /**< Is on when device is advertising. */
#define CONNECTED_LED                 BSP_LED_1_MASK                                                /**< Is on when device is connected. */

// #define NS_BUTTON_PIN_NO              BSP_BUTTON_0                                                  /**< Button used to send Neighbor Solicitation message. */
// #define RS_BUTTON_PIN_NO              BSP_BUTTON_1                                                  /**< Button used to send Router Solicitation message. */
// #define PING_ALL_BUTTON_PIN_NO        BSP_BUTTON_2                                                  /**< Button used to send Echo Request to all device. */
// #define PING_PEER_BUTTON_PIN_NO       BSP_BUTTON_3                                                  /**< Button used to send Echo Request to peer device. */

#define APP_TIMER_PRESCALER           2                                                             /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS          4                                                             /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE       5                                                             /**< Size of timer operation queues. */

#define SCHED_MAX_EVENT_DATA_SIZE     128                                                           /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE              8                                                             /**< Maximum number of events in the scheduler queue. */

#define APP_ADV_TIMEOUT               0                                                             /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define APP_ADV_ADV_INTERVAL          MSEC_TO_UNITS(1000, UNIT_0_625_MS)                             /**< The advertising interval. This value can vary between 100ms to 10.24s). */

#define DEAD_BEEF                     0xDEADBEEF                                                    /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_LEDS_BLINK_TIMEOUT        10000                                                          /**< Timeout between LEDS toggling. */
#define APP_MAX_TOGGLING_NUMBER       10                                                            /**< Number of LEDS toggling after successful operation. */

#define APPL_LOG  app_trace_log
#define APPL_DUMP app_trace_dump

#define APPL_ADDR(addr) APPL_LOG("%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x\r\n", \
                                (addr).u8[0],(addr).u8[1],(addr).u8[2],(addr).u8[3],                            \
                                (addr).u8[4],(addr).u8[5],(addr).u8[6],(addr).u8[7],                            \
                                (addr).u8[8],(addr).u8[9],(addr).u8[10],(addr).u8[11],                          \
                                (addr).u8[12],(addr).u8[13],(addr).u8[14],(addr).u8[15])

eui64_t                       eui64_local_iid;                                                      /**< Local EUI64 value that is used as the IID for SLAAC. */
static iot_interface_t      * mp_interface = NULL;                                                  /**< Pointer to IoT interface if any. */
static ble_gap_adv_params_t   m_adv_params;                                                         /**< Parameters to be passed to the stack when starting advertising. */

/**@brief Helper variables for LEDS effect. */
static app_timer_id_t         m_leds_timer_id;
static bool                   m_leds_activate = false;

/**@brief IPv6 well-known addresses. */
static const ipv6_addr_t      all_node_multicast_addr   = {{0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}};
static const ipv6_addr_t      all_router_multicast_addr = {{0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}};
static const ipv6_addr_t      me = {{0x26, 0x07, 0xf0, 0x18, 0x08, 0x00, 0x01, 0xee, 0x02, 0x00, 0x00, 0xff, 0xfe, 0x00, 0x00, 0xff}};
//2607:f018:600:3:020c:29ff:fe3b:b2a5
#define SERVER_IPV6_ADDRESS           0x26, 0x07, 0xf0, 0x18, 0x06, 0x00, 0x00, 0x03, \
                                      0x02, 0x0c, 0x29, 0xFF, 0xFE, 0x3b, 0xb2, 0xa5                /**< IPv6 address of the server node. */


#define UDP_PORT                      0x1717                                                        /**< Port for transmission of UDP packets. */
static udp6_socket_t          m_udp_socket;                                                         /**< UDP socket used for reception and transmission. */


static uint8_t connected = 0;


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
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    //NVIC_SystemReset();

    APPL_LOG("[** ASSERT **]: Error 0x%08lX, Line %ld, File %s\r\n",error_code, line_num, p_file_name);

    led_off(ADVERTISING_LED);

    for(;;)
    {
    }
}


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


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    // Configure leds.
    // LEDS_CONFIGURE(LEDS_MASK);

    // // Set LEDS off.
    // LEDS_OFF(LEDS_MASK);

    led_init(ADVERTISING_LED);
    led_off(ADVERTISING_LED);
}



/**@brief Function for starting LEDS effect.
 *
 * @details Starting timer for handling LEDS toggling.
 */
static void leds_timer_start()
{
    uint32_t err_code;

    // Start app_timer.
    // if(m_leds_activate)
    // {
        // LEDS_OFF(LEDS_MASK);

        err_code = app_timer_start(m_leds_timer_id, APP_LEDS_BLINK_TIMEOUT, NULL);
        APP_ERROR_CHECK(err_code);
    // }
}



/**@brief Function for the GPIOTE initialization.
 *
 * @details Initializes GPIOTE module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for handling button events.
 *
 * @param[in]   pin_no         The pin number of the button pressed.
 * @param[in]   button_action  The action performed on button.
 */
// static void button_event_handler(uint8_t pin_no, uint8_t button_action)
// {
//     uint32_t                   err_code;
//     ipv6_addr_t                src_addr;
//     ipv6_addr_t                dest_addr;
//     icmp6_ns_param_t           ns_param;
//     iot_pbuffer_t            * p_buffer;
//     iot_pbuffer_alloc_param_t  pbuff_param;

//     // Check if interface is UP.
//     if(mp_interface == NULL)
//     {
//         return;
//     }

//     // Create Link Local address.
//     IPV6_CREATE_LINK_LOCAL_FROM_EUI64(&src_addr, mp_interface->local_addr.identifier);

//     if (button_action == APP_BUTTON_PUSH)
//     {
//         // Activate LEDS operation.
//         m_leds_activate = true;
//         LEDS_OFF(LEDS_MASK);

//         switch (pin_no)
//         {
//             case NS_BUTTON_PIN_NO:
//             {
//                 APPL_LOG("[APPL]: Sending Neighbour Solicitation to peer!\r\n");

//                 ns_param.add_aro      = true;
//                 ns_param.aro_lifetime = 1000;

//                 // Unicast address.
//                 IPV6_CREATE_LINK_LOCAL_FROM_EUI64(&ns_param.target_addr,
//                                                   mp_interface->peer_addr.identifier);

//                 // Send Neighbour Solicitation to all nodes.
//                 err_code = icmp6_ns_send(mp_interface,
//                                          &src_addr,
//                                          &ns_param.target_addr,
//                                          &ns_param);
//                 APP_ERROR_CHECK(err_code);
//                 break;
//             }
//             case RS_BUTTON_PIN_NO:
//             {
//                 APPL_LOG("[APPL]: Sending Router Solicitation to all routers FF02::2!\r\n");

//                 // Send Router Solicitation to all routers.
//                 err_code = icmp6_rs_send(mp_interface,
//                                          &src_addr,
//                                          &all_router_multicast_addr);
//                 APP_ERROR_CHECK(err_code);
//                 break;
//             }
//             case PING_ALL_BUTTON_PIN_NO:
//             {
//                 APPL_LOG("[APPL]: Ping all remote nodes using FF02::1 address!\r\n");

//                 pbuff_param.flags  = PBUFFER_FLAG_DEFAULT;
//                 pbuff_param.type   = ICMP6_PACKET_TYPE;
//                 pbuff_param.length = ICMP6_ECHO_REQUEST_PAYLOAD_OFFSET + 10;

//                 // Allocate packet buffer.
//                 err_code = iot_pbuffer_allocate(&pbuff_param, &p_buffer);
//                 APP_ERROR_CHECK(err_code);

//                 // Fill payload of Echo Request with 'A' letters.
//                 memset(p_buffer->p_payload + ICMP6_ECHO_REQUEST_PAYLOAD_OFFSET, 'A', 10);

//                 // Send Echo Request to all nodes.
//                 err_code = icmp6_echo_request(mp_interface, &src_addr, &all_node_multicast_addr, p_buffer);
//                 APP_ERROR_CHECK(err_code);
//                 break;
//             }
//             case PING_PEER_BUTTON_PIN_NO:
//             {
//                 APPL_LOG("[APPL]: Ping peer device!\r\n");

//                 pbuff_param.flags  = PBUFFER_FLAG_DEFAULT;
//                 pbuff_param.type   = ICMP6_PACKET_TYPE;
//                 pbuff_param.length = ICMP6_ECHO_REQUEST_PAYLOAD_OFFSET + 10;

//                 // Allocate packet buffer.
//                 err_code = iot_pbuffer_allocate(&pbuff_param, &p_buffer);
//                 APP_ERROR_CHECK(err_code);

//                 // Unicast address.
//                 IPV6_CREATE_LINK_LOCAL_FROM_EUI64(&dest_addr,
//                                                   mp_interface->peer_addr.identifier);

//                 // Fill payload of Echo Request with 'B' letters.
//                 memset(p_buffer->p_payload + ICMP6_ECHO_REQUEST_PAYLOAD_OFFSET, 'A', 10);

//                 // Send Echo Request to all nodes.
//                 err_code = icmp6_echo_request(mp_interface, &src_addr, &dest_addr, p_buffer);
//                 APP_ERROR_CHECK(err_code);
//                 break;
//             }

//             default:
//               break;
//         }
//     }
// }


/**@brief Function for handling leds.
 *
 * @details Toggling LEDS for certain number of times.
 */
static void leds_timer_handler(void * p_context)
{
    uint32_t        err_code;
    static uint8_t count = 0;

    // if(++blink_number % APP_MAX_TOGGLING_NUMBER)
    // {
    //     LEDS_INVERT(LEDS_MASK);
    // }
    // else
    // {
    //     m_leds_activate = false;

        //err_code = app_timer_stop(m_leds_timer_id);
        //APP_ERROR_CHECK(err_code);

    //     LEDS_OFF(LEDS_MASK);
    // }






    if (connected) {
        volatile int i;
        int j, k;
        int len;

        volatile int32_t temp;

        char str[256];




        led_on(ADVERTISING_LED);
        // led_off(ADVERTISING_LED);
        // for (k=0; k<10000;k++) {
        //     j = i;
        // }
        // led_off(ADVERTISING_LED);
        // led_off(ADVERTISING_LED);
        // led_off(ADVERTISING_LED);
        // led_off(ADVERTISING_LED);
        // led_off(ADVERTISING_LED);
        // led_off(ADVERTISING_LED);
        // led_off(ADVERTISING_LED);
        // led_off(ADVERTISING_LED);
        // led_off(ADVERTISING_LED);
        // led_on(ADVERTISING_LED);


        iot_pbuffer_alloc_param_t   pbuff_param;
        iot_pbuffer_t             * p_buffer = NULL;

        

        

        // p_buffer->p_payload[0] = '{';
        // p_buffer->p_payload[1] = '"';
        // p_buffer->p_payload[2] = 't';
        // p_buffer->p_payload[2] = 'e';
        // p_buffer->p_payload[2] = 'm';
        // p_buffer->p_payload[2] = 'p';
        // p_buffer->p_payload[2] = '"';
        // p_buffer->p_payload[2] = ':';
        // p_buffer->p_payload[2] = '"';
        // p_buffer->p_payload[2] = '"';
        // p_buffer->p_payload[2] = '"';
        // p_buffer->p_payload[3] = 65 + ((count++)%65);
        // p_buffer->p_payload[4] = '\n';


        // TEMP

        NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

        /* Busy wait while temperature measurement is not finished, you can skip waiting if you enable interrupt for DATARDY event and read the result in the interrupt. */
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        while (NRF_TEMP->EVENTS_DATARDY == 0)
        {
            // Do nothing.
        }
        NRF_TEMP->EVENTS_DATARDY = 0;

        /**@note Workaround for PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register. */
        temp = (nrf_temp_read() / 4);

        /**@note Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. */
        NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */

        // len = snprintf(str, "{\"temp\":%i}", temp);
        len = sprintf(str, "{\"temp\":10}");
        len = sprintf(str, "{\"temp\":%i}", temp);

        pbuff_param.flags  = PBUFFER_FLAG_DEFAULT;
        pbuff_param.type   = UDP6_PACKET_TYPE;
        pbuff_param.length = len;

        // Allocate packet buffer.
        err_code = iot_pbuffer_allocate(&pbuff_param, &p_buffer);
        APP_ERROR_CHECK(err_code);


        // len = snprintf(str, "{\"temp\":%i}", temp);
        memcpy(p_buffer->p_payload, str, len);


        // memcpy(p_buffer->p_payload, &packet.packet_seq_num[0], TEST_PACKET_NUM_LEN);
        // memcpy(p_buffer->p_payload+TEST_PACKET_NUM_LEN, &packet.packet_data[0], TEST_PACKET_DATA_LEN);

        ipv6_addr_t dest_ipv6_addr;
        memset(&dest_ipv6_addr, 0x00, sizeof(ipv6_addr_t));
        memcpy(&dest_ipv6_addr.u8[0], (uint8_t[]){SERVER_IPV6_ADDRESS}, IPV6_ADDR_SIZE);

        // Transmit UDP6 packet.
        err_code = udp6_socket_sendto(&m_udp_socket, &dest_ipv6_addr, HTONS(UDP_PORT), p_buffer);
        APP_ERROR_CHECK(err_code);


        // app_timer_stop(m_leds_timer_id);
        // leds_timer_start();


    } else {
        led_toggle(ADVERTISING_LED);
    }












//     {
//     ipv6_addr_t                src_addr;
//     ipv6_addr_t                dest_addr;
//     icmp6_ns_param_t           ns_param;
//     iot_pbuffer_t            * p_buffer;
//     iot_pbuffer_alloc_param_t  pbuff_param;



//     APPL_LOG("[APPL]: Sending Neighbour Solicitation to peer!\r\n");

//     ns_param.add_aro      = true;
//     ns_param.aro_lifetime = 1000;

//     // Unicast address.
//     IPV6_CREATE_LINK_LOCAL_FROM_EUI64(&ns_param.target_addr,
//                                       mp_interface->peer_addr.identifier);

//     // Send Neighbour Solicitation to all nodes.
//     err_code = icmp6_ns_send(mp_interface,
//                              &src_addr,
//                              &ns_param.target_addr,
//                              &ns_param);
//     APP_ERROR_CHECK(err_code);
// }
}



uint32_t rx_udp_port_app_handler(const udp6_socket_t * p_socket,
                                 const ipv6_header_t * p_ip_header,
                                 const udp6_header_t * p_udp_header,
                                 uint32_t              process_result,
                                 iot_pbuffer_t       * p_rx_packet)
{
    // APPL_LOG("[APPL]: Got UDP6 data on socket 0x%08lx\r\n", p_socket->socket_id);

    // APP_ERROR_CHECK(process_result);

    // // Print PORTs
    // // APPL_LOG("[APPL]: UDP Destination port: %lx\r\n", HTONS(p_udp_header->destport));
    // // APPL_LOG("[APPL]: UDP Source port: %lx\r\n",      HTONS(p_udp_header->srcport));

    // uint32_t rx_sequence_num = uint32_decode(&p_rx_packet->p_payload[0]);
    // uint32_t ind_buff = 0;
    // uint32_t err_code = get_packet_buffer_index(&ind_buff, &rx_sequence_num);
    // if (err_code == NRF_ERROR_NOT_FOUND)
    // {
    //     // Received packet sequence number is not found amongst expected packets.
    //     return NRF_SUCCESS;
    // }

    // if (0 == memcmp(p_rx_packet->p_payload, &m_packet_buffer[ind_buff][0], TEST_PACKET_PAYLOAD_LEN))
    // {
    //     // If received packet is as expected, free slot in buffer.
    //     memset(&m_packet_buffer[ind_buff][0], 0x00, TEST_PACKET_PAYLOAD_LEN);
    // }

    return NRF_SUCCESS;
}





/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

    err_code = app_timer_create(&m_leds_timer_id, APP_TIMER_MODE_REPEATED, leds_timer_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Button initialization.
 *
 * @details Initializes all Buttons used by this application.
 */
// static void buttons_init(void)
// {
//     uint32_t err_code;

//     static app_button_cfg_t buttons[] =
//     {
//         {NS_BUTTON_PIN_NO,          false, BUTTON_PULL, button_event_handler},
//         {RS_BUTTON_PIN_NO,          false, BUTTON_PULL, button_event_handler},
//         {PING_ALL_BUTTON_PIN_NO,    false, BUTTON_PULL, button_event_handler},
//         {PING_PEER_BUTTON_PIN_NO, false, BUTTON_PULL, button_event_handler}
//     };

//     #define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)

//     APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, true);

//     err_code = app_button_enable();
//     APP_ERROR_CHECK(err_code);
// }


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t                err_code;
    ble_advdata_t           advdata;
    uint8_t                 flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_gap_addr_t          my_addr;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_address_get(&my_addr);
    APP_ERROR_CHECK(err_code);

    my_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    my_addr.addr[5]   = 0x00;

    err_code = sd_ble_gap_address_set(&my_addr);
    APP_ERROR_CHECK(err_code);

    IPV6_EUI64_CREATE_FROM_EUI48 (eui64_local_iid.identifier,
                                  my_addr.addr,
                                  my_addr.addr_type);

    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_IPSP_SERVICE, BLE_UUID_TYPE_BLE}
    };

    //Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    //Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT;
}


/**@brief ICMP6 module event handler.
 *
 * @details Callback registered with the ICMP6 module to receive asynchronous events from
 *          the module, if the ICMP6_ENABLE_ALL_MESSAGES_TO_APPLICATION constant is not zero
 *          or the ICMP6_ENABLE_ND6_MESSAGES_TO_APPLICATION constant is not zero.
 */
// uint32_t icmp6_handler(iot_interface_t  * p_interface,
//                        ipv6_header_t    * p_ip_header,
//                        icmp6_header_t   * p_icmp_header,
//                        uint32_t           process_result,
//                        iot_pbuffer_t    * p_rx_packet)
// {
//     APPL_LOG("[APPL]: Got ICMP6 Application Handler Event on interface 0x%p\r\n", p_interface);

//     APP_ERROR_CHECK(process_result);

//     APPL_LOG("[APPL]: Source IPv6 Address: ");
//     APPL_ADDR(p_ip_header->srcaddr);
//     APPL_LOG("[APPL]: Destination IPv6 Address: ");
//     APPL_ADDR(p_ip_header->destaddr);
//     APPL_LOG("[APPL]: Process result = 0x%08lX\r\n", process_result);

//     switch(p_icmp_header->type)
//     {
//         case ICMP6_TYPE_DESTINATION_UNREACHABLE:
//             APPL_LOG("[APPL]: ICMP6 Message Type = Destination Unreachable Error\r\n");
//             break;
//         case ICMP6_TYPE_PACKET_TOO_LONG:
//             APPL_LOG("[APPL]: ICMP6 Message Type = Packet Too Long Error\r\n");
//             break;
//         case ICMP6_TYPE_TIME_EXCEED:
//             APPL_LOG("[APPL]: ICMP6 Message Type = Time Exceed Error\r\n");
//             break;
//         case ICMP6_TYPE_PARAMETER_PROBLEM:
//             APPL_LOG("[APPL]: ICMP6 Message Type = Parameter Problem Error\r\n");
//             break;
//         case ICMP6_TYPE_ECHO_REQUEST:
//             APPL_LOG("[APPL]: ICMP6 Message Type = Echo Request\r\n");
//             break;
//         case ICMP6_TYPE_ECHO_REPLY:
//             APPL_LOG("[APPL]: ICMP6 Message Type = Echo Reply\r\n");

//             // Start blinking LEDS to indicate that Echo Reply has been received.
//             // leds_timer_start();

//             break;
//         case ICMP6_TYPE_ROUTER_SOLICITATION:
//             APPL_LOG("[APPL]: ICMP6 Message Type = Router Solicitation\r\n");
//             break;
//         case ICMP6_TYPE_ROUTER_ADVERTISEMENT:
//             APPL_LOG("[APPL]: ICMP6 Message Type = Router Advertisement\r\n");

//             // Start blinking LEDS to indicate that Router Advertisement has been received.
//             // leds_timer_start();

//             break;
//         case ICMP6_TYPE_NEIGHBOR_SOLICITATION:
//             APPL_LOG("[APPL]: ICMP6 Message Type = Neighbor Solicitation\r\n");
//             break;
//         case ICMP6_TYPE_NEIGHBOR_ADVERTISEMENT:
//             APPL_LOG("[APPL]: ICMP6 Message Type = Neighbor Advertisement\r\n");

//             // Start blinking LEDS to indicate that Neighbor Advertisement has been received.
//             // leds_timer_start();

//             break;
//         default:
//             break;
//     }

//     return NRF_SUCCESS;
// }


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    // led_on(ADVERTISING_LED);
}


/**@brief IP Stack interface events handler. */
void ip_app_handler(iot_interface_t * p_interface, ipv6_event_t * p_event)
{
    uint32_t err_code;
    ipv6_addr_conf_t addr_conf;


    APPL_LOG("[APPL]: Got IP Application Handler Event on interface 0x%p\r\n", p_interface);

    // led_off(ADVERTISING_LED);



    switch(p_event->event_id)
    {
        case IPV6_EVT_INTERFACE_ADD:
            APPL_LOG("[APPL]: New interface added!\r\n");
            mp_interface = p_interface;

            // LEDS_ON(CONNECTED_LED);
            // LEDS_OFF(ADVERTISING_LED);
            // leds_timer_start();

            connected = 1;

// app_timer_stop(m_leds_timer_id);

            addr_conf.state = IPV6_ADDR_STATE_PREFERRED;
            memcpy(addr_conf.addr.u8, me.u8, 16);
            err_code = ipv6_address_set(p_interface, &addr_conf);
            APP_ERROR_CHECK(err_code);


            err_code = udp6_socket_allocate(&m_udp_socket);
            APP_ERROR_CHECK(err_code);

            err_code = udp6_socket_bind(&m_udp_socket, IPV6_ADDR_ANY, HTONS(UDP_PORT));
            APP_ERROR_CHECK(err_code);

            err_code = udp6_socket_recv(&m_udp_socket, rx_udp_port_app_handler);
            APP_ERROR_CHECK(err_code);








            break;
        case IPV6_EVT_INTERFACE_DELETE:
            APPL_LOG("[APPL]: Interface removed!\r\n");

            // LEDS_OFF(CONNECTED_LED);

            connected = 0;

            // leds_timer_start();
            advertising_start();

            break;
        case IPV6_EVT_INTERFACE_RX_DATA:
            APPL_LOG("[APPL]: Got unsupported protocol data!\r\n");
            break;

        default:
            //Unknown event. Should not happen.
            break;
    }
}


/**@brief Function for initializing IP stack.
 *
 * @details Initialize the IP Stack.
 */
static void ip_stack_init(void)
{
   uint32_t    err_code;
   ipv6_init_t init_param;

   init_param.p_eui64       = &eui64_local_iid;
   init_param.event_handler = ip_app_handler;

   err_code = ipv6_init(&init_param);
   APP_ERROR_CHECK(err_code);

   // err_code = icmp6_receive_register(icmp6_handler);
   // APP_ERROR_CHECK(err_code);
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_ipsp_evt_handler(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    // SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, true);
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, true);
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION, true);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the transport of IPv6. */
// void ipv6_transport_init(void)
// {
//     ble_stack_init();
//     advertising_init();
// }


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    // led_init(ADVERTISING_LED);
    // led_on(ADVERTISING_LED);



    // Initialize.
    app_trace_init();
    leds_init();
	//led_on(ADVERTISING_LED);
    nrf_temp_init();
    timers_init();
    gpiote_init();
    // buttons_init();
    scheduler_init();

// led_on(ADVERTISING_LED);

    // Transport initialize.
    // ipv6_transport_init();
    ble_stack_init();

// led_on(ADVERTISING_LED);

    advertising_init();

// led_on(ADVERTISING_LED);

    // Initialize IP Stack.
    ip_stack_init();

    APPL_LOG("[APPL]: Advertising.\r\n");

    // Start execution.
    advertising_start();



    // for (;;)
    // {


    //     err_code = sd_app_evt_wait();
    //     APP_ERROR_CHECK(err_code);
    // }

leds_timer_start();




    // Enter main loop.
    for (;;)
    {
        /* Execute event schedule */
        app_sched_execute();

// led_on(ADVERTISING_LED);

        /* Sleep waiting for an application event. */
        err_code = sd_app_evt_wait();

        // led_toggle(ADVERTISING_LED);

        APP_ERROR_CHECK(err_code);


    }
}

/**
 * @}
 */

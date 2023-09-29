/*******************************************************************************
 * Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

#pragma once

#include "wiced_bt_ble.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define HELLO_CLIENT_MAX_NUM_CLIENTS 1

/* Hello Sensor App Timer Timeout in seconds  */
#define HELLO_CLIENT_APP_TIMEOUT_IN_SECONDS                 1

/* Hello Sensor App Fine Timer Timeout in milli seconds  */
#define HELLO_CLIENT_APP_FINE_TIMEOUT_IN_MS                 1

/* Hello Sensor Connection Idle  Timeout in milli seconds  */
#define HELLO_CLIENT_CONN_IDLE_TIMEOUT_IN_SECONDS           3

#define HELLO_CLIENT_VS_ID                  100
#define HELLO_CLIENT_LOCAL_KEYS_VS_ID       101
#define HELLO_CLIENT_PAIRED_KEYS_VS_ID      102

/*read raw rssi every timeout (10 sec)*/
#define read_raw_rssi

#ifdef read_raw_rssi
typedef struct
{
    wiced_bt_device_address_t       bd_addr;     /**< Remote device address */
    wiced_bt_ble_address_type_t     addr_type;   /**< Remmote device address type */
    uint16_t                        conn_id;     /**< ID of the connection */
    wiced_bool_t                    connected;   /**< TRUE if connected, FALSE if disconnected */
    wiced_bt_transport_t            transport;   /**< Transport type of the connection */
    wiced_bt_dev_role_t             link_role;   /**< Link role on this connection */
    wiced_bool_t                    s8_coding_active;
    wiced_bt_ble_channel_sel_algo_t channel_sel_algo;
} rssi_cb_t;

#endif

void application_start(void);

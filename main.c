/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the FreeRTOS
*              LE Long Range Central Example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-
2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
 * Copyright 2021-
2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "FreeRTOSConfig.h"
#include <FreeRTOS.h>
#include "task.h"

#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_types.h"

#include "cy_pdl.h"
#include "cybsp.h"
#include "cybsp_types.h"
#include "cybt_debug_uart.h"
#include "cybt_platform_config.h"
#include "cybt_platform_hci.h"
#include "cybt_platform_trace.h"
#include "cybt_result.h"
#include "cycfg.h"
#include "cyhal.h"
#include "mtb_kvstore.h"
#include "app_serial_flash.h"
#include "cybsp_bt_config.h"
#include "cy_retarget_io.h"

#include "hello_client.h"

extern mtb_kvstore_bd_t block_device;

int main()
{
    cy_rslt_t result;

    /* Initialize the board support package */
    result = cybsp_init();

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

#ifdef ENABLE_BT_SPY_LOG
        cybt_debug_uart_config_t config = {
            .uart_tx_pin = CYBSP_DEBUG_UART_TX,
            .uart_rx_pin = CYBSP_DEBUG_UART_RX,
            .uart_cts_pin = CYBSP_DEBUG_UART_CTS,
            .uart_rts_pin = CYBSP_DEBUG_UART_RTS,
            //.baud_rate = DEBUG_UART_BAUDRATE,
            .baud_rate = 3000000,
            .flow_control = TRUE};
        cybt_debug_uart_init(&config, NULL);
#else
    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
#endif // ENABLE_BT_SPY_LOG

    cybt_platform_set_trace_level(CYBT_TRACE_ID_STACK, CYBT_TRACE_ID_MAX);
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /*Initialize the block device used by kv-store for performing read/write operations to the flash*/
    app_kvstore_bd_init(&block_device);

    application_start();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);
}

/*******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

#include <stdint.h>

#include "user_btn.h"

#include <FreeRTOS.h>
#include <task.h>

#include "cybsp_types.h"
#include "cycfg_pins.h"
#include "cyhal_gpio.h"

#include "wiced_bt_types.h"
#include "wiced_timer.h"

/* Stack size for BTN task */
#define BTN_TASK_STACK_SIZE (512u)

#define GPIO_INTERRUPT_PRIORITY (4u)

#define USER_BUTTON_TIMER_INTERVAL_MS 200

/* Handle of the btn task */
static TaskHandle_t btn_handle;
static wiced_timer_t btn_ms_timer;
static uint32_t ms_timer_count = 0;
static gpio_interrupt_handler_t g_gpio_interrupt_handler_ptr = NULL;

static void gpio_interrupt_handler(void *callback_arg, cyhal_gpio_event_t event);

static cyhal_gpio_callback_data_t cb_data = {.callback = gpio_interrupt_handler, .callback_arg = NULL};

static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint32_t previous_btn_pressed_timer = 0;
    static uint32_t current_btn_pressed_timer = 0;

    if (event != CYHAL_GPIO_IRQ_FALL) return;

    /* xHigherPriorityTaskWoken must be initialised to pdFALSE.  If calling
    vTaskNotifyGiveFromISR() unblocks the handling task, and the priority of
    the handling task is higher than the priority of the currently running task,
    then xHigherPriorityTaskWoken will automatically get set to pdTRUE. */
    xHigherPriorityTaskWoken = pdFALSE;

    current_btn_pressed_timer = ms_timer_count;

    /* Debounce logic to prevent spurious callbacks. Ensures atleast 600ms between button presses*/
    if ((current_btn_pressed_timer - previous_btn_pressed_timer) > (600 / USER_BUTTON_TIMER_INTERVAL_MS)) {
        previous_btn_pressed_timer = current_btn_pressed_timer;

        /* Unblock the handling task so the task can perform any processing necessitated
            by the interrupt.  xHandlingTask is the task's handle, which was obtained
            when the task was created. */
        vTaskNotifyGiveIndexedFromISR(btn_handle, 0, &xHigherPriorityTaskWoken);

        /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
            The macro used to do this is dependent on the port and may be called
            portEND_SWITCHING_ISR. */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void user_btn_thread(void *arg)
{
    for (;;) {

        /* Block indefinitely (without a timeout, so no need to check the function's
        return value) to wait for a notification.  Here the RTOS task notification
        is being used as a binary semaphore, so the notification value is cleared
        to zero on exit.  NOTE!  Real applications should not block indefinitely,
        but instead time out occasionally in order to handle error conditions
        that may prevent the interrupt from sending any more notifications. */
        ulTaskNotifyTakeIndexed(0,              /* Use the 0th notification */
                                pdTRUE,         /* Clear the notification value before exiting. */
                                portMAX_DELAY); /* Block indefinitely. */

        if (g_gpio_interrupt_handler_ptr) g_gpio_interrupt_handler_ptr();
    }
}

void btn_timeout_handler(void *finecount)
{
    ms_timer_count++;
}

void configure_user_btn(gpio_interrupt_handler_t gpio_interrupt_handler)
{
    BaseType_t xReturned;

    xReturned = xTaskCreate(user_btn_thread, "User btn task", BTN_TASK_STACK_SIZE, NULL, 4, &btn_handle);
    if (xReturned != pdPASS) return;

    /* Initialize the user btn */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, CYBSP_BTN_OFF);

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, GPIO_INTERRUPT_PRIORITY, true);

    if (WICED_SUCCESS == wiced_init_timer(&btn_ms_timer, btn_timeout_handler, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER)) {
        wiced_start_timer(&btn_ms_timer, USER_BUTTON_TIMER_INTERVAL_MS);
    }

    g_gpio_interrupt_handler_ptr = gpio_interrupt_handler;
}

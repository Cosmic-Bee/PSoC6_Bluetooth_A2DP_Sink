/******************************************************************************
 * (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 ******************************************************************************/
/******************************************************************************
 * File Name: i2s_audio.c
 *
 * Description: Definitions for constants used in the a2dp sink
 * application and function prototypes.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

#include "i2s_audio.h"

#define BUFFER_SIZE 4096
uint32_t tx_buffer0[BUFFER_SIZE];
uint32_t tx_buffer1[BUFFER_SIZE];
uint32_t *active_tx_buffer;
uint32_t *next_tx_buffer;
size_t active_tx_buffer_index = 0;
bool is_transmitting = false;

static void i2s_event_handler_transmit_streaming(void* arg, cyhal_i2s_event_t event)
{
	if (event & CYHAL_I2S_ASYNC_TX_COMPLETE) {
		is_transmitting = false;
	}
}

void i2s_audio_init(void)
{
    active_tx_buffer = tx_buffer0;
    next_tx_buffer = tx_buffer1;
    active_tx_buffer_index = 0;
    cyhal_i2s_set_async_mode(&i2s, CYHAL_ASYNC_DMA, CYHAL_DMA_PRIORITY_DEFAULT);
    cyhal_i2s_start_tx(&i2s);

    cyhal_i2s_register_callback(&i2s, &i2s_event_handler_transmit_streaming, &i2s);
    cyhal_i2s_enable_event(&i2s, CYHAL_I2S_ASYNC_TX_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
}

void swap_buffers_and_write_to_i2s()
{
    // Send the active buffer to I2S
    cyhal_i2s_write_async(&i2s, active_tx_buffer, BUFFER_SIZE);
    cyhal_i2s_start_tx(&i2s);

    // Swap active and next buffers
    uint32_t *temp = active_tx_buffer;
    active_tx_buffer = next_tx_buffer;
    next_tx_buffer = temp;

    // Reset the active buffer index
    active_tx_buffer_index = 0;
}

// Deinitialize the I2S audio interface
void i2s_audio_deinit(void)
{
    cyhal_i2s_free(&i2s);
}

// Send audio data to the I2S interface
int i2s_audio_data_write(void *data, unsigned long frames_to_send, long *num_frames_written)
{
    if(!is_transmitting && active_tx_buffer >= 32) {
    	is_transmitting = true;
        swap_buffers_and_write_to_i2s();
    }

    uint32_t *src_data = (uint32_t *)data;

    memcpy(&active_tx_buffer[active_tx_buffer_index], &src_data[0], frames_to_send * sizeof(uint32_t));
    active_tx_buffer_index += frames_to_send;

    *num_frames_written = frames_to_send;

    return 0; // Write successful
}


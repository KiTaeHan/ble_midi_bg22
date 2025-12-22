/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "sl_bt_api.h"
#include "sl_main_init.h"
#include "app_assert.h"
#include "app.h"

#include <stdio.h>
#include "em_cmu.h"
#include "sl_sleeptimer.h"
#include "sl_uartdrv_instances.h"

#define SAM2695_RESET_PIN    7
#define SAM2695_RESET_PORT   gpioPortA


// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

sl_sleeptimer_timer_handle_t app_timer;
volatile bool timer_timeout = false;

static void on_timeout(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;
  timer_timeout = true;
}

void app_delayms(uint32_t ms)
{
  sl_status_t sc;

  sc = sl_sleeptimer_start_timer_ms(&app_timer,
                                    ms,
                                    on_timeout,
                                    NULL,
                                    0,
                                    0);
  app_assert_status(sc);

  while (!timer_timeout) 
  {
    // Wait for timer to expire
  }
  timer_timeout = false;
}

void SAM2695_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SAM2695_RESET_PORT, SAM2695_RESET_PIN, gpioModePushPull, 1); // LED0

  GPIO_PinOutClear(SAM2695_RESET_PORT, SAM2695_RESET_PIN); // Hold SAM2695 in reset
  app_delayms(10);
  GPIO_PinOutSet(SAM2695_RESET_PORT, SAM2695_RESET_PIN); // Release SAM2695 from reset
  app_delayms(500);
}

void SAM2695_sendMIDI(uint8_t cmd, uint8_t data1, uint8_t data2)
{
  UARTDRV_TransmitB(sl_uartdrv_usart_MIDI_COM_handle, &cmd, 1);
  UARTDRV_TransmitB(sl_uartdrv_usart_MIDI_COM_handle, &data1, 1);

  if (cmd < 0xC0 || cmd >= 0xE0) // Program Change 등 2바이트 메시지 제외 [6, 7]
  {
    UARTDRV_TransmitB(sl_uartdrv_usart_MIDI_COM_handle, &data2, 1);
  }
}

void SAM2695_test_midi(void)
{
  // 도(60)부터 시(71)까지 연주하는 음계 배열 [10]
  uint8_t scale[] = {60, 62, 64, 65, 67, 69, 71};

  // 기본 피아노 음색 선택 (PC 1번) [8, 9]
  SAM2695_sendMIDI(0xC0, 0x00, 0x00);

  for (int i = 0; i < 7; i++)
  {
    // Note On: 채널 0, 음 번호 scale[i], 강도 100 [6]
    SAM2695_sendMIDI(0x90, scale[i], 100);
    app_delayms(500);
    // Note Off: 강도를 0으로 하여 소리 끔 [6]
    SAM2695_sendMIDI(0x90, scale[i], 0);
  }
}

// Application Init.
void app_init(void)
{
  SAM2695_init();
  SAM2695_test_midi();
}

// Application Process Action.
void app_process_action(void)
{
  printf("Looping\r\n");

  if (app_is_process_required()) {
    /////////////////////////////////////////////////////////////////////////////
    // Put your additional application code here!                              //
    // This is will run each time app_proceed() is called.                     //
    // Do not call blocking functions from here!                               //
    /////////////////////////////////////////////////////////////////////////////
  }
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the default weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

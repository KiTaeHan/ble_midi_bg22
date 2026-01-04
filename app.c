#include "sl_bt_api.h"
#include "sl_main_init.h"
#include "app_assert.h"
#include "gatt_db.h"
#include "app.h"

#include <stdio.h>
#include "em_cmu.h"
#include "sl_sleeptimer.h"
#include "sl_uartdrv_instances.h"

#define SAM2695_RESET_PIN 7
#define SAM2695_RESET_PORT gpioPortA

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

  // MIDI Status Byte 확인: 0xC0(Program Change)와 0xD0(Channel Pressure)는 2바이트 메시지입니다.
  // 그 외(Note On/Off, Control Change, Pitch Bend 등)는 3바이트 메시지입니다.
  if ((cmd & 0xF0) != 0xC0 && (cmd & 0xF0) != 0xD0)
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

// BLE MIDI 패킷 수신 핸들러
void handle_midi_data(uint8_t *data, uint16_t len)
{
  // BLE MIDI 패킷 구조: [Header] [Timestamp] [Status] [Data1] [Data2]

  if (len < 3)
    return; // 최소 헤더+타임스탬프+데이터1바이트

  // 중요: 하나의 BLE 패킷 안에 여러 개의 MIDI 메시지가 포함될 수 있음 (Running Status 포함)
  // BLE MIDI 패킷 내의 타임스탬프 바이트(0x80-0xFF)를 필터링하고 순수 MIDI 데이터만 UART로 전송합니다.
  bool last_byte_was_timestamp = true; // 인덱스 1은 항상 타임스탬프이므로 true로 시작

  // 인덱스 0: Header, 인덱스 1: Timestamp
  // 인덱스 2부터 실제 MIDI 데이터 시작
  for (int i = 2; i < len; i++)
  {
    uint8_t byte = data[i];

    // 현재 바이트가 0x80 이상이고, 바로 앞 바이트가 타임스탬프가 아니었다면 -> 이것은 스트림 내의 타임스탬프입니다.
    if ((byte & 0x80) && !last_byte_was_timestamp)
    {
      last_byte_was_timestamp = true;
      continue; // 전송하지 않고 건너뜀
    }

    UARTDRV_TransmitB(sl_uartdrv_usart_MIDI_COM_handle, &byte, 1);
    last_byte_was_timestamp = false;
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
  // printf("Looping\r\n");

  if (app_is_process_required())
  {
    /////////////////////////////////////////////////////////////////////////////
    // Put your additional application code here!                              //
    // This is will run each time app_proceed() is called.                     //
    // Do not call blocking functions from here!                               //
    /////////////////////////////////////////////////////////////////////////////
  }
}

/**************************************************************************/ 
/**
  * Bluetooth stack event handler.
  * This overrides the default weak implementation.
  *
  * @param[in] evt Event coming from the Bluetooth stack.
*****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  // printf("Event ID: %lx\r\n", SL_BT_MSG_ID(evt->header));

  switch (SL_BT_MSG_ID(evt->header))
  {
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
  // Connection parameter constrains for BLE MIDI
  sl_bt_connection_set_parameters(evt->data.evt_connection_opened.connection,
                                  0xC, 0xC, 0, 0x64, 0, 0xFFFF);
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

  // -------------------------------
  // This event indicates that the value of an attribute in the local GATT
  // database was changed by a remote GATT client.
  case sl_bt_evt_gatt_server_attribute_value_id:
    // Check if the event is for the MIDI Data I/O characteristic.
    // Note: 'gattdb_midi_data_io' must match the ID in your gatt_db.h
    if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_midi_data_io)
    {
      handle_midi_data(evt->data.evt_gatt_server_attribute_value.value.data,
                       evt->data.evt_gatt_server_attribute_value.value.len);
    }
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

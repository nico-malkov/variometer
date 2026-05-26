/**
 * Copyright (c) 2017 - 2022, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nrfx.h> //

#include "app_button.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_msc.h"
#include "app_usbd_string_desc.h"
#include "app_util_platform.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
// #include "ble_cus.h" //
#include "ble_dfu.h"
#include "ble_hci.h"
#include "ble_nus.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "diskio_blkdev.h"
#include "fds.h"
#include "ff.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_atomic.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_block_dev.h"
#include "nrf_block_dev_empty.h"
#include "nrf_block_dev_qspi.h"
#include "nrf_block_dev_ram.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_power.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_saadc.h" //
#include "nrf_drv_twi.h"
#include "nrf_drv_usbd.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_libuarte_async.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_nvmc.h" //
#include "nrf_power.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_queue.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_soc.h" //
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "sensorsim.h" //
#include "timestamping.h"

#include "Device.h"
#include "DeviceIcm20948.h"
#include "Icm20948.h"
#include "Icm20948DataBaseDriver.h" // sleep function
#include "Icm20948setup.h"
#include "SensorConfig.h"
// #include "Icm20948MPUFifoControl.h"  // reset fifo    - not working

#include "battery_helper.h"
#include "myESB.h"
#include "twi_func.h"

// From ESB example
#include "app_util.h"
#include "nrf_error.h"
#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "nrf_gpio.h"
#include "sdk_common.h"

//------------------------------------------------------------------------------------
// General functions part
//------------------------------------------------------------------------------------

static void log_init(void) { // function for debug logging initializing
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void softdevice_init(void) { // function for enabling softdevice
  ret_code_t err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);
}

static void power_management_init(void) { // function for initializing power management
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void) { // function for handling the idle state (main loop) – if there is no pending log operation, then sleep until next the next event occurs
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

//------------------------------------------------------------------------------------
// Bluetooth part
//------------------------------------------------------------------------------------

#define APP_BLE_CONN_CFG_TAG 1                               // a tag identifying the SoftDevice BLE configuration
#define APP_BLE_OBSERVER_PRIO 3                              // application's BLE observer priority. You shouldn't need to modify this value
#define APP_ADV_INTERVAL 64                                  // the advertising interval (in units of 0.625 ms. This value corresponds to 40 ms)
#define APP_ADV_DURATION 0                                   // the advertising duration in units of 10 (millis)econds, 0 is endless
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)     // connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units
#define DEAD_BEEF 0xDEADBEEF                                 // value used as error code on stack dump, can be used to identify stack location on stack unwind
#define DEVICE_STRING "Vario 1.8"                  // name of the device, will be included in the advertising data
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) // time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds)
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)    // maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)    // minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units
#define MAX_CONN_PARAMS_UPDATE_COUNT 3
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) // time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds)
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN     // UUID type for the Nordic UART Service (vendor specific)
#define SEC_PARAM_BOND 1                                     // perform bonding
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE       // no I/O capabilities
#define SEC_PARAM_KEYPRESS 0                                 // keypress notifications not enabled
#define SEC_PARAM_LESC 0                                     // LE Secure Connections not enabled
#define SEC_PARAM_MIN_KEY_SIZE 7                             // minimum encryption key size
#define SEC_PARAM_MITM 0                                     // Man In The Middle protection is not required
#define SEC_PARAM_MAX_KEY_SIZE 16                            //  maximum encryption key size
#define SEC_PARAM_OOB 0                                      // Out Of Band data not available
#define SLAVE_LATENCY 0                                      // slave latency

BLE_ADVERTISING_DEF(m_advertising);               // advertising module instance
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); // BLE NUS service instance
NRF_BLE_GATT_DEF(m_gatt);                         // GATT module instance
NRF_BLE_QWR_DEF(m_qwr);                           // context for the Queued Write module

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                           // handle of the current connection
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;             // maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}}; // universally unique service identifier

typedef void (*bluetooth_get_handle_t)(char S[BLE_NUS_MAX_DATA_LEN]);
static bluetooth_get_handle_t bluetooth_get_handle;

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event) {
  switch (event) {
  case NRF_PWR_MGMT_EVT_PREPARE_DFU:
    NRF_LOG_INFO("Power management wants to reset to DFU mode.");
    // YOUR_JOB: Get ready to reset into DFU mode
    //
    // If you aren't finished with any ongoing tasks, return "false" to
    // signal to the system that reset is impossible at this stage.
    //
    // Here is an example using a variable to delay resetting the device.
    //
    // if (!m_ready_for_reset)
    // {
    //      return false;
    // }
    // else
    //{
    //
    //    // Device ready to enter
    //    uint32_t err_code;
    //    err_code = sd_softdevice_disable();
    //   APP_ERROR_CHECK(err_code);
    //    err_code = app_timer_stop_all();
    //   APP_ERROR_CHECK(err_code);
    //}
    break;

  default:
    // YOUR_JOB: Implement any of the other events available from the power management module:
    //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
    //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
    //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
    return true;
  }

  NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
  return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void *p_context) {
  if (state == NRF_SDH_EVT_STATE_DISABLED) {
    nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);         // softdevice was disabled before going into reset, inform bootloader to skip CRC on next boot
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF); // go to system off
  }
}

NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = {
    .handler = buttonless_dfu_sdh_state_observer,
};

static void advertising_config_get(ble_adv_modes_config_t *p_config) {
  memset(p_config, 0, sizeof(ble_adv_modes_config_t));

  p_config->ble_adv_fast_enabled = true;
  p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
  p_config->ble_adv_fast_timeout = APP_ADV_DURATION;
}

static void disconnect(uint16_t conn_handle, void *p_context) {
  UNUSED_PARAMETER(p_context);

  ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  if (err_code != NRF_SUCCESS) {
    NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
  } else {
    NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
  }
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event) {
  switch (event) {
  case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE: {
    NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

    ble_adv_modes_config_t config; // prevent device from advertising on disconnect
    advertising_config_get(&config);
    config.ble_adv_on_disconnect_disabled = true;
    ble_advertising_modes_config_set(&m_advertising, &config);

    // Disconnect all other bonded devices that currently are connected.
    // This is required to receive a service changed indication
    // on bootup after a successful (or aborted) Device Firmware Update.
    uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
    NRF_LOG_INFO("Disconnected %d links.", conn_count);
    break;
  }

  case BLE_DFU_EVT_BOOTLOADER_ENTER:
    // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
    //           by delaying reset by reporting false in app_shutdown_handler
    NRF_LOG_INFO("Device will enter bootloader mode.");
    break;

  case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
    NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
    // YOUR_JOB: Take corrective measures to resolve the issue
    //           like callingAPP_ERROR_CHECK to reset the device.
    break;

  case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
    NRF_LOG_ERROR("Request to send a response to client failed.");
    // YOUR_JOB: Take corrective measures to resolve the issue
    //           like callingAPP_ERROR_CHECK to reset the device.
    APP_ERROR_CHECK(false);
    break;

  default:
    NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
    break;
  }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt) {
  pm_handler_on_pm_evt(p_evt);
  pm_handler_flash_clean(p_evt);
}

/**@brief Function for starting timers.
 */
static void application_timers_start(void) {
  // YOUR_JOB: Start your timers. below is an example of how to start a timer.
  // uint32_t err_code;
  // err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
  // APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init() {
  ble_gap_sec_params_t sec_param;
  ret_code_t err_code;

  err_code = pm_init();
  APP_ERROR_CHECK(err_code);

  memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

  sec_param.bond = SEC_PARAM_BOND; // security parameters to be used for all security procedures
  sec_param.mitm = SEC_PARAM_MITM;
  sec_param.lesc = SEC_PARAM_LESC;
  sec_param.keypress = SEC_PARAM_KEYPRESS;
  sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
  sec_param.oob = SEC_PARAM_OOB;
  sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
  sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
  sec_param.kdist_own.enc = 1;
  sec_param.kdist_own.id = 1;
  sec_param.kdist_peer.enc = 1;
  sec_param.kdist_peer.id = 1;

  err_code = pm_sec_params_set(&sec_param);
  APP_ERROR_CHECK(err_code);

  err_code = pm_register(pm_evt_handler);
  APP_ERROR_CHECK(err_code);
}

/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void) {
  ret_code_t err_code;

  NRF_LOG_INFO("Erase bonds!");

  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void) {
  uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
  uint32_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
  uint32_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void) {
  // uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  // APP_ERROR_CHECK(err_code);

  // err_code = bsp_btn_ble_sleep_mode_prepare();  // prepare wakeup buttons
  // APP_ERROR_CHECK(err_code);

  ret_code_t err_code = sd_power_system_off(); // go to system-off mode (this function will not return; wakeup will cause a reset)
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  uint32_t err_code;

  switch (ble_adv_evt) {
  case BLE_ADV_EVT_FAST:
    // err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    // APP_ERROR_CHECK(err_code);
    break;
  case BLE_ADV_EVT_IDLE:
    sleep_mode_enter();
    break;
  default:
    break;
  }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  uint32_t err_code;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    NRF_LOG_INFO("Connected");
    // err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    // APP_ERROR_CHECK(err_code);
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    NRF_LOG_INFO("Disconnected");
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  } break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL); // pairing not supported
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_SYS_ATTR_MISSING:
    err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0); // no system attributes have been stored
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTC_EVT_TIMEOUT:
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION); // disconnect on GATT Client timeout event
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION); // disconnect on GATT Server timeout event
    APP_ERROR_CHECK(err_code);
    break;

  default:
    break; //  no implementation needed
  }
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
  ret_code_t err_code;

  // err_code = nrf_sdh_enable_request();
  // APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_sdh_ble_enable(&ram_start); // enable BLE stack
  APP_ERROR_CHECK(err_code);

  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL); // register a handler for BLE events
}

static void nus_data_handler(ble_nus_evt_t *p_evt) {
  if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
    char S[BLE_NUS_MAX_DATA_LEN] = {0};
    for (int i = 0; i < p_evt->params.rx_data.length; i++) {
      S[i] = p_evt->params.rx_data.p_data[i];
    }

    bluetooth_get_handle(S); // bluetooth
  }
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(char *DEVICE_NAME) {
  uint32_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)DEVICE_NAME,
      strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
    m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
  }
  NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
      p_gatt->att_mtu_desired_central,
      p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
static void gatt_init(void) {
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(ble_nus_data_handler_t nus_data_handler, ble_dfu_buttonless_evt_handler_t ble_dfu_evt_handler) {
  uint32_t err_code;
  ble_nus_init_t nus_init;
  nrf_ble_qwr_init_t qwr_init = {0};
  ble_dfu_buttonless_init_t dfus_init = {0};

  qwr_init.error_handler = nrf_qwr_error_handler; // initialize Queued Write Module
  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

  dfus_init.evt_handler = ble_dfu_evt_handler; // DFU
  err_code = ble_dfu_buttonless_init(&dfus_init);

  memset(&nus_init, 0, sizeof(nus_init));   // initialize NUS
  nus_init.data_handler = nus_data_handler; // bluetooth
  err_code = ble_nus_init(&m_nus, &nus_init);
  APP_ERROR_CHECK(err_code);
}

static void bluetooth_init(char *DEVICE_NAME, bluetooth_get_handle_t bluetooth_get, ble_dfu_buttonless_evt_handler_t ble_dfu_evt_handler) {
  bluetooth_get_handle = bluetooth_get;

  gap_params_init(DEVICE_NAME);
  gatt_init();
  services_init(nus_data_handler, ble_dfu_evt_handler);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
  uint32_t err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = true;
  init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; // this option is for endless advertising // BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE; // this option is for limited advertising time

  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void bluetooth_get(char S[BLE_NUS_MAX_DATA_LEN]) {
  NRF_LOG_DEBUG("Bluetooth Read");
  NRF_LOG_HEXDUMP_DEBUG(S, strlen(S));
}

static void bluetooth_send(char S[BLE_NUS_MAX_DATA_LEN]) {
  uint16_t length = strlen(S);

  uint32_t err_code = ble_nus_data_send(&m_nus, S, &length, m_conn_handle);
  if ((err_code != NRF_ERROR_INVALID_STATE) &&
      (err_code != NRF_ERROR_RESOURCES) &&
      (err_code != NRF_ERROR_NOT_FOUND)) {
    // APP_ERROR_CHECK(err_code);
  }
}

static void ble_init(void) {
  bool erase_bonds;
  ble_stack_init();
  bluetooth_init(DEVICE_STRING, bluetooth_get, ble_dfu_evt_handler); // firmware releaase version is in the name of vario
  advertising_init();
  conn_params_init();
  advertising_start();
}

//------------------------------------------------------------------------------------
// USBD part
//------------------------------------------------------------------------------------

#define BTN_RANDOM_FILE 0
#define BTN_LIST_DIR 1
#define BTN_MKFS 2w

#define KEY_EV_RANDOM_FILE_MSK (1U << BTN_RANDOM_FILE)
#define KEY_EV_LIST_DIR_MSK (1U << BTN_LIST_DIR)
#define KEY_EV_MKFS_MSK (1U << BTN_MKFS)

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/**
 * @brief SD card enable/disable
 */
#define USE_SD_CARD 0

/**
 * @brief FatFS for QPSI enable/disable
 */
#define USE_FATFS_QSPI 1

/**
 * @brief Mass storage class user event handler
 */
static void msc_user_ev_handler(app_usbd_class_inst_t const *p_inst,
    app_usbd_msc_user_event_t event);

/**
 * @brief Ram block device size
 *
 * @note Windows fails to format volumes smaller than 190KB
 */
#define RAM_BLOCK_DEVICE_SIZE (380 * 512)

/**
 * @brief  RAM block device work buffer
 */
static uint8_t m_block_dev_ram_buff[RAM_BLOCK_DEVICE_SIZE];

/**
 * @brief  RAM block device definition
 */
NRF_BLOCK_DEV_RAM_DEFINE(
    m_block_dev_ram,
    NRF_BLOCK_DEV_RAM_CONFIG(512, m_block_dev_ram_buff, sizeof(m_block_dev_ram_buff)),
    NFR_BLOCK_DEV_INFO_CONFIG("Vario", "RAM", "1.00"));

/**
 * @brief Empty block device definition
 */
NRF_BLOCK_DEV_EMPTY_DEFINE(
    m_block_dev_empty,
    NRF_BLOCK_DEV_EMPTY_CONFIG(512, 1024 * 1024),
    NFR_BLOCK_DEV_INFO_CONFIG("Vario", "EMPTY", "1.00"));

/**
 * @brief  QSPI block device definition
 */
NRF_BLOCK_DEV_QSPI_DEFINE(
    m_block_dev_qspi,
    NRF_BLOCK_DEV_QSPI_CONFIG(
        512,
        NRF_BLOCK_DEV_QSPI_FLAG_CACHE_WRITEBACK,
        NRF_DRV_QSPI_DEFAULT_CONFIG),
    NFR_BLOCK_DEV_INFO_CONFIG("", "Vario", "Mass Storage"));

#if USE_SD_CARD

#define SDC_SCK_PIN (27)     ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN (26)    ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN (2)     ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN (32 + 15) ///< SDC chip select (CS) pin.

/**
 * @brief  SDC block device definition
 */
NRF_BLOCK_DEV_SDC_DEFINE(
    m_block_dev_sdc,
    NRF_BLOCK_DEV_SDC_CONFIG(
        SDC_SECTOR_SIZE,
        APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)),
    NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00"));

/**
 * @brief Block devices list passed to @ref APP_USBD_MSC_GLOBAL_DEF
 */
#define BLOCKDEV_LIST() (                                 \
    NRF_BLOCKDEV_BASE_ADDR(m_block_dev_ram, block_dev),   \
    NRF_BLOCKDEV_BASE_ADDR(m_block_dev_empty, block_dev), \
    NRF_BLOCKDEV_BASE_ADDR(m_block_dev_qspi, block_dev),  \
    NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev))

#else
#define BLOCKDEV_LIST() ( \
    NRF_BLOCKDEV_BASE_ADDR(m_block_dev_qspi, block_dev))
#endif

/**
 * @brief Endpoint list passed to @ref APP_USBD_MSC_GLOBAL_DEF
 */
#define ENDPOINT_LIST() APP_USBD_MSC_ENDPOINT_LIST(1, 1)

/**
 * @brief Mass storage class work buffer size
 */
#define MSC_WORKBUFFER_SIZE (1024)

/**
 * @brief Mass storage class instance
 */
APP_USBD_MSC_GLOBAL_DEF(m_app_msc,
    0,
    msc_user_ev_handler,
    ENDPOINT_LIST(),
    BLOCKDEV_LIST(),
    MSC_WORKBUFFER_SIZE);

/**
 * @brief Events from keys
 */
static nrf_atomic_u32_t m_key_events;

/**
 * @brief  USB connection status
 */
static bool m_usb_connected = false;

#if USE_FATFS_QSPI

static FATFS m_filesystem;

static void fatfs_error(FRESULT ff_result) {
  switch (ff_result) {
  case 1:
    NRF_LOG_ERROR("FR_DISK_ERR A hard error occurred in the low level disk I/O layer\r\n");
    break;
  case 2:
    NRF_LOG_ERROR("FR_INT_ERR Assertion failed\r\n");
    break;
  case 3:
    NRF_LOG_ERROR("FR_NOT_READY The physical drive cannot work\r\n");
    break;
  case 4:
    NRF_LOG_ERROR("FR_NO_FILE Could not find the file\r\n");
    break;
  case 5:
    NRF_LOG_ERROR("FR_NO_PATH Could not find the path\r\n");
    break;
  case 6:
    NRF_LOG_ERROR("FR_INVALID_NAME The path name format is invalid\r\n");
    break;
  case 7:
    NRF_LOG_ERROR("FR_DENIED Access denied due to prohibited access or directory full\r\n");
    break;
  case 8:
    NRF_LOG_ERROR("FR_EXIST Access denied due to prohibited access\r\n");
    break;
  case 9:
    NRF_LOG_ERROR("FR_INVALID_OBJECT The file/directory object is invalid\r\n");
    break;
  case 10:
    NRF_LOG_ERROR("FR_WRITE_PROTECTED The physical drive is write protected\r\n");
    break;
  case 11:
    NRF_LOG_ERROR("FR_INVALID_DRIVE The logical drive number is invalid\r\n");
    break;
  case 12:
    NRF_LOG_ERROR("FR_NOT_ENABLED The volume has no work area\r\n");
    break;
  case 13:
    NRF_LOG_ERROR("FR_NO_FILESYSTEM There is no valid FAT volume\r\n");
    break;
  case 14:
    NRF_LOG_ERROR("FR_MKFS_ABORTED The f_mkfs() aborted due to any problem\r\n");
    break;
  case 15:
    NRF_LOG_ERROR("FR_TIMEOUT Could not get a grant to access the volume within defined period\r\n");
    break;
  case 16:
    NRF_LOG_ERROR("FR_LOCKED The operation is rejected according to the file sharing policy\r\n");
    break;
  case 17:
    NRF_LOG_ERROR("FR_NOT_ENOUGH_CORE LFN working buffer could not be allocated\r\n");
    break;
  case 18:
    NRF_LOG_ERROR("FR_TOO_MANY_OPEN_FILEST Number of open files > _FS_LOCK\r\n");
    break;
  case 19:
    NRF_LOG_ERROR("FR_INVALID_PARAMETER Given parameter is invalid\r\n");
    break;
  }
}

static bool fatfsInit = 0;
#include <nrfx_qspi.h>

#define WREN_OP_CODE 0x06
#define WRDI_OP_CODE 0x04
#define WRSR_OP_CODE 0x01
#define RDCR_OP_CODE 0x15
#define RDSR_OP_CODE 0x05

#define MX25_HIGH_POWER 1 // 1 = high power mode, 0 = low power mode

nrf_qspi_cinstr_conf_t cinstr_cfg = {
    .opcode = 0x66, // is QSPI_STD_CMD_RSTEN
    .length = NRF_QSPI_CINSTR_LEN_4B,
    .io2_level = true,
    .io3_level = true,
    .wipwait = true,
    .wren = true};

static bool fatfs_init(void) {
  FRESULT ff_result;
  DSTATUS disk_state = STA_NOINIT;

  memset(&m_filesystem, 0, sizeof(FATFS));

  // Initialize FATFS disk I/O interface by providing the block device.
  static diskio_blkdev_t drives[] =
      {
          DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_qspi, block_dev), NULL)};

  diskio_blockdev_register(drives, ARRAY_SIZE(drives));

  // NRF_LOG_INFO("Initializing filesystem on disk (QSPI)...");
  disk_state = disk_initialize(0);
  if (disk_state) {
    NRF_LOG_ERROR("Disk initialization failed.\n");
    return false;
  }

  ret_code_t ret = 0;

  if (MX25_HIGH_POWER == 1) {
    uint8_t rdsr_rx_buf[1] = {0};
    cinstr_cfg.opcode = RDSR_OP_CODE;
    cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_4B;
    ret = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, rdsr_rx_buf); // reads status register
    // NRF_LOG_INFO("rdsr_rx_buf = %u", rdsr_rx_buf[0]);

    nrf_delay_ms(1);

    uint8_t rdcr_rx_buf[2] = {0, 0};
    cinstr_cfg.opcode = RDCR_OP_CODE;
    cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_4B;
    ret = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, rdcr_rx_buf);
    // NRF_LOG_INFO("rdcr_rx_buf = %u, %u", rdcr_rx_buf[0], rdcr_rx_buf[1]); // reads configuration registers 1 and 2

    nrf_delay_ms(1);

    uint8_t wrsr_tx_buf[3] = {0, 0, 0};
    wrsr_tx_buf[0] = 66; // rdsr_rx_buf[0];
    wrsr_tx_buf[1] = 0;  // rdcr_rx_buf[0];
    wrsr_tx_buf[2] = 2;  // Low/High performance switch - 0x10 is high power mode
    cinstr_cfg.opcode = WRSR_OP_CODE;
    cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_4B;
    ret = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, wrsr_tx_buf, NULL); // writes to status and configurtion registers 1 and 2

    nrf_delay_ms(1);

    cinstr_cfg.opcode = RDCR_OP_CODE;
    cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_4B;
    ret = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, rdcr_rx_buf);
    // NRF_LOG_INFO("rdcr_rx_buf = %u, %u", rdcr_rx_buf[0], rdcr_rx_buf[1]);

    if (rdcr_rx_buf[1] == 2) {
      NRF_LOG_INFO("MX25R64F initialization SUCCESS"); // we has succesfully checked that flash operates in high power mode - that means flash is OK
    } else {
      NRF_LOG_INFO("MX25R64F ERROR");
    }
  }

  // NRF_LOG_INFO("Mounting volume...");
  ff_result = f_mount(&m_filesystem, "", 1);
  if (ff_result != FR_OK) {
    if (ff_result == FR_NO_FILESYSTEM) {
      NRF_LOG_ERROR("Mount failed. Filesystem not found. Please format device.\r\n");
    } else {
      NRF_LOG_ERROR("Mount failed: %u\r\n", ff_result);
    }
    return false;
  }

  fatfsInit = 1;
  return true;
}

static void fatfs_mkfs(void) {
  FRESULT ff_result;

  if (m_usb_connected) {
    NRF_LOG_ERROR("Unable to operate on filesystem while USB is connected\r\n");
    return;
  }

  NRF_LOG_INFO("Creating filesystem...");
  static uint8_t buf[512];
  ff_result = f_mkfs("", FM_FAT, 1024, buf, sizeof(buf));
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Mkfs failed.\r\n");
    return;
  }

  // NRF_LOG_INFO("Mounting volume...");
  ff_result = f_mount(&m_filesystem, "", 1);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Mount failed.\r\n");
    return;
  }

  // NRF_LOG_INFO("Done");
}

static void fatfs_ls(void) {
  DIR dir;
  FRESULT ff_result;
  FILINFO fno;

  if (m_usb_connected) {
    NRF_LOG_ERROR("Unable to operate on filesystem while USB is connected\r\n");
    return;
  }

  NRF_LOG_INFO("Listing QSPI flash directory:");
  ff_result = f_opendir(&dir, "/");
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Directory listing failed: %u\r\n", ff_result);
    return;
  }

  uint32_t entries_count = 0;
  do {
    ff_result = f_readdir(&dir, &fno);
    if (ff_result != FR_OK) {
      NRF_LOG_ERROR("Directory read failed: %u\r\n", ff_result);
      return;
    }

    if (fno.fname[0]) {
      if (fno.fattrib & AM_DIR) {
        NRF_LOG_INFO("   <DIR>   %s", (uint32_t)fno.fname);
      } else {
        NRF_LOG_INFO("%9lu  %s", fno.fsize, (uint32_t)fno.fname);
      }
    }

    ++entries_count;
    NRF_LOG_FLUSH();
  } while (fno.fname[0]);

  // NRF_LOG_INFO(" Entries count: %u ", entries_count);
}

void fatfs_write_line(char *filename, char *toWrite, size_t size) {
  FRESULT ff_result;
  FIL file;
  uint32_t bytesWritten;

  if (!fatfsInit) {
    return;
  }

  ff_result = f_open(&file, filename, FA_WRITE);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to open or create file: %u\r\n", ff_result);
    fatfs_error(ff_result);
    return;
  }

  NRF_LOG_INFO("f_size %d", f_size(&file));

  ff_result = f_write(&file, toWrite, size - 1, (UINT *)&bytesWritten);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Write failed: %u\r\n", ff_result);
    fatfs_error(ff_result);
    return;
  }

  ff_result = f_close(&file);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to close file: %s\r\n", filename);
    fatfs_error(ff_result);
    return;
  }
  NRF_LOG_FLUSH();
  return;
}

void fatfs_append_line(char *filename, char *toAppend, size_t size) {
  FRESULT ff_result;
  FIL file;
  uint32_t bytesWritten;

  if (!fatfsInit) {
    return;
  }

  ff_result = f_open(&file, filename, FA_OPEN_APPEND | FA_WRITE);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to open or create file: %u\r\n", ff_result);
    fatfs_error(ff_result);
    return;
  }

  /*ff_result = f_lseek(&file, sizeof(file)-1);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to set pointer to the end of the file: %u\r\n", ff_result);
    fatfs_error(ff_result);
    return;
  }*/

  ff_result = f_write(&file, toAppend, size - 1, (UINT *)&bytesWritten);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Append failed: %u\r\n", ff_result);
    fatfs_error(ff_result);
    return;
  }

  ff_result = f_close(&file);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to close file: %u\r\n", ff_result);
    fatfs_error(ff_result);
    NRF_LOG_FLUSH();
    return;
  }
}

static uint32_t fileSize = 0;

void fatfs_write_next_line(char *filename, char *toAppend, size_t dataSize) {
  FRESULT ff_result;
  FIL file;
  FILINFO fno;
  uint32_t bytesWritten;

  if (!fatfsInit) {
    return;
  }

  /*ff_result = f_stat(filename, &fno);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to get info about the file: %u\r\n", ff_result);
    fatfs_error(ff_result);
    return;
  }*/

  // NRF_LOG_INFO("File size according to FILINFO: %d", fno.fsize);
  // NRF_LOG_INFO("File size according to f_size: %d", f_size(&file));

  ff_result = f_open(&file, filename, FA_WRITE);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to open or create file: %u\r\n", ff_result);
    fatfs_error(ff_result);
    return;
  }

  // NRF_LOG_INFO("f_size %d", f_size(&file));
  // fileSize = fno.fsize;

  ff_result = f_lseek(&file, fileSize); // f_size takes too much time to compute (> 300 ms)
  // ff_result = f_lseek(&file, fno.fsize); // same as previous
  // ff_result = f_lseek(&file, fileSize);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to set pointer to the end of the file: %u\r\n", ff_result);
    fatfs_error(ff_result);
    return;
  }

  ff_result = f_write(&file, toAppend, dataSize - 1, (UINT *)&bytesWritten);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Append failed: %u\r\n", ff_result);
    fatfs_error(ff_result);
    return;
  }

  NRF_LOG_INFO("bytesWritten %d", bytesWritten);
  fileSize = fileSize + bytesWritten;
  NRF_LOG_INFO("fileSize %d", fileSize);

  ff_result = f_close(&file);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to close file: %u\r\n", ff_result);
    fatfs_error(ff_result);
    NRF_LOG_FLUSH();
    return;
  }
}

static void fatfs_create_file(char *filename) {
  FRESULT ff_result;
  FIL file;
  uint32_t bytesWritten;

  if (!fatfsInit) {
    return;
  }

  //(void)snprintf(filename, sizeof(filename), "%s.txt", (uint32_t)name);

  NRF_LOG_INFO("Creating file: %s ...", filename);
  NRF_LOG_FLUSH();

  ff_result = f_open(&file, filename, FA_CREATE_NEW | FA_WRITE);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to open or create file: %u\r\n", ff_result);
    fatfs_error(ff_result);
    NRF_LOG_FLUSH();
    return;
  }

  /*ff_result = f_write(&file, "\r\n", sizeof("\r\n"), (UINT *)&bytesWritten);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Write failed: %u\r\n", ff_result);
    fatfs_error(ff_result);
    return;
  }*/

  ff_result = f_close(&file);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to close file: %u\r\n", ff_result);
    fatfs_error(ff_result);
    NRF_LOG_FLUSH();
    return;
  }
  // NRF_LOG_INFO("Done");
}

static void fatfs_uninit(void) {
  // NRF_LOG_INFO("Un-initializing filesystem on disk (QSPI)");
  UNUSED_RETURN_VALUE(disk_uninitialize(0));
  fatfsInit = 0;
}
#else // USE_FATFS_QSPI
#define fatfs_init() false
#define fatfs_mkfs() \
  do {               \
  } while (0)
#define fatfs_ls() \
  do {             \
  } while (0)
#define fatfs_file_create() \
  do {                      \
  } while (0)
#define fatfs_uninit() \
  do {                 \
  } while (0)
#endif

/**
 * @brief Class specific event handler.
 *
 * @param p_inst    Class instance.
 * @param event     Class specific event.
 */
static void msc_user_ev_handler(app_usbd_class_inst_t const *p_inst,
    app_usbd_msc_user_event_t event) {
  UNUSED_PARAMETER(p_inst);
  UNUSED_PARAMETER(event);
}

static bool usbConnected = 0;

/**
 * @brief USBD library specific event handler.
 *
 * @param event     USBD library event.
 */
static void usbd_user_ev_handler(app_usbd_event_type_t event) {
  switch (event) {
  case APP_USBD_EVT_POWER_DETECTED:
    NRF_LOG_INFO("APP_USBD_EVT_POWER_DETECTED");
    if (!nrf_drv_usbd_is_enabled()) {
      fatfs_uninit();
      app_usbd_enable();
    }
    usbConnected = true;
    break;
  case APP_USBD_EVT_POWER_READY:
    NRF_LOG_INFO("APP_USBD_EVT_POWER_READY");
    app_usbd_start();
    break;
  case APP_USBD_EVT_STARTED:
    NRF_LOG_INFO("APP_USBD_EVT_STARTED");
    break;
  case APP_USBD_EVT_DRV_SUSPEND:
    NRF_LOG_INFO("APP_USBD_EVT_DRV_SUSPEND");
    break;

    // case APP_USBD_EVT_DRV_RESUME: // case USB connection established

  case APP_USBD_EVT_POWER_REMOVED:
    NRF_LOG_INFO("APP_USBD_EVT_POWER_REMOVED");
    app_usbd_stop();
    break;
  case APP_USBD_EVT_DRV_RESUME:
    NRF_LOG_INFO("APP_USBD_EVT_DRV_RESUME");
    break;
  case APP_USBD_EVT_STOPPED:
    NRF_LOG_INFO("APP_USBD_EVT_STOPPED");
    fatfs_init();
    app_usbd_disable();
    usbConnected = false;
    break;
  default:
    break;
  }
}

/*static void bsp_event_callback(bsp_event_t ev) {
  switch (ev) {
  // Just set a flag to be processed in the main loop
  case CONCAT_2(BSP_EVENT_KEY_, BTN_RANDOM_FILE):
    UNUSED_RETURN_VALUE(nrf_atomic_u32_or(&m_key_events, KEY_EV_RANDOM_FILE_MSK));
    break;

  case CONCAT_2(BSP_EVENT_KEY_, BTN_LIST_DIR):
    UNUSED_RETURN_VALUE(nrf_atomic_u32_or(&m_key_events, KEY_EV_LIST_DIR_MSK));
    break;

  case CONCAT_2(BSP_EVENT_KEY_, BTN_MKFS):
    UNUSED_RETURN_VALUE(nrf_atomic_u32_or(&m_key_events, KEY_EV_MKFS_MSK));
    break;

  default:
    return; // no implementation needed
  }
}*/

static void usb_init(void) {
  static const app_usbd_config_t usbd_config = {
      .ev_state_proc = usbd_user_ev_handler};

  ret_code_t err_code = app_usbd_init(&usbd_config);
  APP_ERROR_CHECK(err_code);
  app_usbd_class_inst_t const *class_inst_msc = app_usbd_msc_class_inst_get(&m_app_msc);
  err_code = app_usbd_class_append(class_inst_msc);
  APP_ERROR_CHECK(err_code);
  if (fatfs_init()) {
    fatfs_ls();
    // fatfs_file_create();
  }
  if (USBD_POWER_DETECTION) {
    err_code = app_usbd_power_events_enable();
    APP_ERROR_CHECK(err_code);
  } else {
    NRF_LOG_INFO("No USB power detection enabled, starting USB now");
    app_usbd_enable();
    app_usbd_start();
    m_usb_connected = true;
  }
}

bool beepMuteFlag = 1;
bool usbEvent = 0;
static uint32_t ticksStartOfUsbEvent = 0;

void usb_check(void) {
  while (app_usbd_event_queue_process()) {
    // nothing to do
  }

  /*if (usbConnected) {
    if (!usbEvent) {
      usbEvent = 1;
      ticksStartOfUsbEvent = app_timer_cnt_get();
    }

    if ((app_timer_cnt_diff_compute(app_timer_cnt_get(), ticksStartOfUsbEvent) / 16.384) > 5000) {
      beepMuteFlag = 0;
    } else {
      beepMuteFlag = 1;
    }
  } else {
    usbEvent = 0;
  }*/
}

//------------------------------------------------------------------------------------
// ADC functions part
//------------------------------------------------------------------------------------

static volatile int16_t result = 0;
static volatile float precise_result = 0;
static volatile float final_result = 0;

static void adc_init(void) {
  // Configure SAADC singled-ended channel, Internal reference (0.6V) and 1/6 gain.
  NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
                            (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
                            (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
                            (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
                            (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |
                            (SAADC_CH_CONFIG_TACQ_3us << SAADC_CH_CONFIG_TACQ_Pos);

  // Configure the SAADC channel with VDD as positive input, no negative input(single ended).
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput6 << SAADC_CH_PSELP_PSELP_Pos;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

  // Configure the SAADC resolution.
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos;

  // Configure result to be put in RAM at the location of "result" variable.
  NRF_SAADC->RESULT.MAXCNT = 1;
  NRF_SAADC->RESULT.PTR = (uint32_t)&result;

  // No automatic sampling, will trigger with TASKS_SAMPLE.
  NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

  // Enable SAADC (would capture analog pins if they were used in CH[0].PSELP)
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

  // Calibrate the SAADC (only needs to be done once in a while)
  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
  while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0)
    ;
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos))
    ;
}

volatile float adc_sample(void) {
  // Start the SAADC and wait for the started event.
  NRF_SAADC->TASKS_START = 1;
  while (NRF_SAADC->EVENTS_STARTED == 0)
    ;
  NRF_SAADC->EVENTS_STARTED = 0;

  // Do a SAADC sample, will put the result in the configured RAM buffer.
  NRF_SAADC->TASKS_SAMPLE = 1;
  while (NRF_SAADC->EVENTS_END == 0)
    ;
  NRF_SAADC->EVENTS_END = 0;

  // Convert the result to voltage
  // precise_result = [V(p) - V(n)] * GAIN/REFERENCE * 2^(RESOLUTION)
  // precise_result = (Vin - 0) * ((1/6) / 0.6) * 2^14
  // precise_result = result / 4551.1
  precise_result = (float)result / 4551.1f;
  precise_result; // to get rid of set, but not used warning
  // Now convert read votage to the actual voltage supplied to the resistor ladder on PCB
  // Result = precise_result * R2 nominal / (R1 nominal + R2 nominal)
  final_result = (float)precise_result * (5.76f + 3.9f) / 3.9f;

  // Stop the SAADC, since it's not used anymore.
  NRF_SAADC->TASKS_STOP = 1;
  while (NRF_SAADC->EVENTS_STOPPED == 0)
    ;
  NRF_SAADC->EVENTS_STOPPED = 0;

  return (final_result);

  NRF_LOG_INFO("Battery voltage = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(final_result));
}

//------------------------------------------------------------------------------------
// UART part
//------------------------------------------------------------------------------------

/*#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define UART_TX_BUF_SIZE 256 // < UART TX buffer size
#define UART_RX_BUF_SIZE 256 // Number of attempts before giving up the connection parameter negotiation
#define DEAD_BEEF 0xDEADBEEF // Value used as error code on stack dump, can be used to identify stack location on stack unwind

// @brief   Function for handling app_uart events.
//
// @details This function will receive a single character from the app_uart module and append it to
//          a string. The string will be be sent over BLE when the last character received was a
//          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
//
//@snippet [Handling the data received over UART]

void uart_event_handle(app_uart_evt_t *p_event) {
  static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
  static uint8_t index = 0;
  uint32_t err_code;

  switch (p_event->evt_type) {
  case APP_UART_DATA_READY:
    UNUSED_VARIABLE(app_uart_get(&data_array[index]));
    index++;

    if ((data_array[index - 1] == '\n') ||
        (data_array[index - 1] == '\r') ||
        (index >= m_ble_nus_max_data_len)) {
      if (index > 1) {
        NRF_LOG_DEBUG("Ready to send data over BLE NUS");
        NRF_LOG_HEXDUMP_DEBUG(data_array, index);

        do {
          uint16_t length = (uint16_t)index;
          err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
          if ((err_code != NRF_ERROR_INVALID_STATE) &&
              (err_code != NRF_ERROR_RESOURCES) &&
              (err_code != NRF_ERROR_NOT_FOUND)) {
            APP_ERROR_CHECK(err_code);
          }
        } while (err_code == NRF_ERROR_RESOURCES);
      }

      index = 0;
    }
    break;

  case APP_UART_COMMUNICATION_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_communication);
    break;

  case APP_UART_FIFO_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_code);
    break;

  default:
    break;
  }
}

static void uart_init(void) {
  uint32_t err_code;
  app_uart_comm_params_t const comm_params =
  {
    .rx_pin_no = RX_PIN_NUMBER,
    .tx_pin_no = TX_PIN_NUMBER,
    .rts_pin_no = RTS_PIN_NUMBER,
    .cts_pin_no = CTS_PIN_NUMBER,
    .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
    .use_parity = false,
#if defined(UART_PRESENT)
    .baud_rate = NRF_UART_BAUDRATE_115200
#else
    .baud_rate = NRF_UARTE_BAUDRATE_115200
#endif
  };

  APP_UART_FIFO_INIT(&comm_params,
      UART_RX_BUF_SIZE,
      UART_TX_BUF_SIZE,
      uart_event_handle,
      APP_IRQ_PRIORITY_LOWEST,
      err_code);
  APP_ERROR_CHECK(err_code);
}*/

//------------------------------------------------------------------------------------
// PWM part
//------------------------------------------------------------------------------------

// #define BTN 3 // Pin 0.03 definition, defined in custom_board.h
// #define LED 27  // defined within board definitions in custom_board.h
#define LOAD_EN 43 // Pin 1.11 definition
#define SND_EN 45  // Pin 1.13 definition
#define SND 46     // Pin 1.14 definition (sound)

static uint32_t led_pwm_freq = 10000; // [Hz]
static uint32_t snd_pwm_freq = 1000;  // [Hz]

#define MEDIUM 10 // brightness of LED in % of pwm duty cycle
#define BRIGHT 50 // brightness of LED in % of pwm duty cycle

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0); // instance for LED
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1); // instance for SND (sound)

static bool led_pwm_uninit_flag = 0; // flag to not perform uninit twice (that causes freese)
static bool sndPwmInitFlag = 0;      // flag to not perform uninit twice (that causes freese)

nrf_pwm_values_individual_t seq_values[] = {0, 0, 0, 0}; // declare variables holding PWM sequence values

APP_TIMER_DEF(millis_timer);

static uint32_t millisRolloverCounter = 0;

static const uint32_t timerMaxRolloverSize = 0x1000;

uint32_t millis(void) {
  return ((millisRolloverCounter * (timerMaxRolloverSize) + app_timer_cnt_get()) / 16.384); // 16.384 is according to APP_TIMER_CONFIG_RTC_FREQUENCY from sdk_config.h
}

uint32_t timestamp_ms_ = 0;

uint32_t new_millis(uint32_t ticks_from) {
  uint32_t ticks_diff = 0;
  uint32_t ms_diff = 0;
  uint32_t ticks_to = NRF_RTC0->COUNTER; // app_timer_cnt_get();

  ticks_diff = app_timer_cnt_diff_compute(ticks_to, ticks_from);

  ms_diff = APP_TIMER_MS(ticks_diff, APP_TIMER_PRESCALER); //  see timestamp.h
                                                           //  ms_diff = (ticks_diff * (APP_TIMER_PRESCALER + 1) * 1000) / APP_TIMER_CLOCK_FREQ;
  // timestamp_ms_ += ms_diff;

  // return timestamp_ms_;
  return ms_diff;
}

static void millis_timer_handler(void *p_context) { // handler for millis_timer expiration
  millisRolloverCounter++;

  /*ret_code_t err_code = app_timer_create(&millis_timer, APP_TIMER_MODE_SINGLE_SHOT, millis_timer_handler);
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_start(millis_timer, APP_TIMER_TICKS(timerMaxRolloverSize), NULL); // restarts repeatedly every APP_TIMER_TICKS // app_timer_cnt_get() is 24 bit long, so millis() rolls over every (0xFFFFFF / 16.384)= 0xF9FFF ticks
  APP_ERROR_CHECK(err_code);*/

  /*if (millisRolloverCounter >= 0x1062) { // uint32_t overflow happens after 0xFFFFFFFF / 0xF9FFF itterations
    millisRolloverCounter = 0;
  }*/
}

nrf_pwm_sequence_t const seq =
    {
        .values.p_individual = seq_values,
        .length = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats = 0,
        .end_delay = 0};

void led_pwm_update_duty_cycle(uint8_t led_pwm_duty) { // set duty cycle between 0 and 100% for LED
  if (led_pwm_uninit_flag == 1) {                      // if the instance has been init
    if (led_pwm_duty >= 100) {                         // check if value is outside of range; if so, set to 100%
      seq_values->channel_0 = 100;
    } else {
      seq_values->channel_0 = led_pwm_duty;
    }

    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    // NRF_LOG_INFO("led_pwm_update_duty_cycle()");
  }
}

void snd_pwm_update_duty_cycle(uint8_t snd_pwm_duty) { // set duty cycle between 0 and 100% for buzzer
  if (sndPwmInitFlag == 1) {                           // if the instance has been init
    if (snd_pwm_duty >= 100) {                         // check if value is outside of range; if so, set to 100%
      seq_values->channel_1 = 100;
    } else {
      seq_values->channel_1 = snd_pwm_duty;
    }

    nrf_drv_pwm_simple_playback(&m_pwm1, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
  }
}

static void led_pwm_init(void) {
  // NRF_LOG_INFO("led_pwm_init()");
  uint32_t value_pwm_freq = 500000 / led_pwm_freq; // 500000 is according to NRF_PWM_CLK_500kHz

  nrf_drv_pwm_config_t const config0 =
      {
          .output_pins =
              {
                  LED,                      //
                  NRF_DRV_PWM_PIN_NOT_USED, // channel 1
                  NRF_DRV_PWM_PIN_NOT_USED, // channel 2
                  NRF_DRV_PWM_PIN_NOT_USED, // channel 3
              },
          .irq_priority = APP_IRQ_PRIORITY_LOWEST,
          .base_clock = NRF_PWM_CLK_500kHz, // pwm frequency = .base_clock [Hz] / .top_value , maximum .top_value is 0x7FFF = 32767
          .count_mode = NRF_PWM_MODE_UP,
          .top_value = value_pwm_freq, // (?)1kHz(?) don't recall where is 1 kHz from
          .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
          .step_mode = NRF_PWM_STEP_AUTO};

  if (led_pwm_uninit_flag == 0) {
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL)); // init PWM without error handler
    led_pwm_uninit_flag = 1;                                    // allowed to perform uninit
    led_pwm_update_duty_cycle(MEDIUM);
  }
}

static void snd_pwm_init(uint32_t pwmFreq) { // incoming parameter is frequency
  // NRF_LOG_INFO("snd_pwm_init()");
  uint32_t value_pwm_freq = 500000 / pwmFreq; // 500000 is according to NRF_PWM_CLK_500kHz

  nrf_drv_pwm_config_t const config0 =
      {
          .output_pins =
              {
                  NRF_DRV_PWM_PIN_NOT_USED, //
                  SND,                      // channel 1
                  NRF_DRV_PWM_PIN_NOT_USED, // channel 2
                  NRF_DRV_PWM_PIN_NOT_USED, // channel 3
              },
          .irq_priority = APP_IRQ_PRIORITY_LOW, // used to be LOWEST
          .base_clock = NRF_PWM_CLK_500kHz,     // pwm frequency = .base_clock [Hz] / .top_value , maximum .top_value is 0x7FFF = 32767
          .count_mode = NRF_PWM_MODE_UP,
          .top_value = value_pwm_freq, //
          .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
          .step_mode = NRF_PWM_STEP_AUTO};
  if (sndPwmInitFlag == 0) {
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm1, &config0, NULL)); // init PWM without error handler
    sndPwmInitFlag = 1;
    snd_pwm_update_duty_cycle(50); // shall the duty cycle always be 50%
  }
}

static void led_pwm_uninit(void) {
  // NRF_LOG_INFO("led_pwm_uninit()");
  if (led_pwm_uninit_flag == 1) {
    nrfx_pwm_uninit(&m_pwm0);
    nrf_gpio_cfg_input(LED, NRF_GPIO_PIN_NOPULL); // turns off LED
    led_pwm_uninit_flag = 0;                      // performing uninit single time preventing freeze
  }
}

static uint32_t sndMillis = 0;
static uint32_t sndDeltaT = 0;
static uint32_t sndDurT = 0;
static uint32_t sndPlaysFlag = 0;
static bool sndStopFlag = 0;

static void snd_pwm_uninit(void) {
  // NRF_LOG_INFO("snd_pwm_uninit()");
  if (sndPwmInitFlag == 1) {
    nrfx_pwm_uninit(&m_pwm1);
    nrf_gpio_cfg_input(SND, NRF_GPIO_PIN_PULLDOWN); // a quick work-around for abnormal noise after a play
    sndPwmInitFlag = 0;                             // performing uninit single time preventing freeze
    sndPlaysFlag = 0;
  }
}

static uint32_t ticksSndNow = 0;
static uint32_t ticksSndPrevious = 0;

static void snd_play(uint32_t sndFreq, uint32_t sndDur) {
  // NRF_LOG_INFO("snd_play()");
  snd_pwm_uninit(); // to stop previously initiated vario "beep" if it took place
  snd_pwm_init(sndFreq);
  ticksSndPrevious = app_timer_cnt_get();

  // sndMillis = millis();
  sndDurT = sndDur; // populate the duration through global variable
  sndPlaysFlag = 1;
}

static void snd_start(uint32_t sndFreq) {
  // NRF_LOG_INFO("snd_play()");
  snd_pwm_init(sndFreq);
  ticksSndPrevious = app_timer_cnt_get();

  // sndMillis = millis();
  sndDurT = 250; // minimum duration of vario "beep"
  sndPlaysFlag = 2;
  sndStopFlag = 0;
}

static float climbrateKalman, climbThresholdRate;

static bool snd_check() {
  // NRF_LOG_INFO("snd_check");
  if (sndPwmInitFlag == 1 && sndPlaysFlag > 0) { // if sound has been init
    ticksSndNow = app_timer_cnt_get();
    sndDeltaT = app_timer_cnt_diff_compute(ticksSndNow, ticksSndPrevious) / 16.384; // count the time

    // sndDeltaT = millis() - sndMillis;            // find the time for which the sound has been played already
    if (sndDeltaT > sndDurT) { // when the play lenght has been reached
      if (sndPlaysFlag == 1) {
        snd_pwm_uninit(); // turns off sound
        return (1);
      } else if (sndPlaysFlag == 2) {
        // nothing yet
        if (sndStopFlag == 1) {
          snd_pwm_uninit(); // turns off sound
        }
        if (sndDeltaT > 1000 && climbrateKalman >= climbThresholdRate) { // if vario beep sounds unexpectedly long
          snd_pwm_uninit();                                              // turns off sound
        }
        return (1);
      }
    }
  } else {
    return (0);
  }
}

static bool snd_stop() {
  sndStopFlag = 1;
  if (sndPwmInitFlag == 1 && (sndPlaysFlag == 0 || sndPlaysFlag == 2)) { // if sound has been init and not by snd_play function
    snd_pwm_uninit();                                                    // turns off sound
    return (1);
  } else {
    return (0);
  }
}

//------------------------------------------------------------------------------------
// TWI (I2C) part
//------------------------------------------------------------------------------------

#define TWI_INSTANCE_ID_0 0
#define TWI_ADDRESSES 127 // number of possible TWI addresses

static const nrf_drv_twi_t m_twi_baro = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID_0); // TWI instance
uint8_t twi_address = 0;
uint8_t twi_data_tx = 0;
uint8_t twi_data_rx = 0;

// MS5637 I2C (TWI) barometer
#define MS5637_ADDR 0x76 // address of barometer
#define MS5637_RESET 0x1E
#define MS5637_CONVERT_D1 0x40 // base command for pressure conversion
#define MS5637_CONVERT_D2 0x50 // base command for temperature conversion
#define MS5637_ADC_READ 0x00
#define OSR_256 0x00 // summand to the base command, which defines oversampling rate
#define OSR_512 0x02
#define OSR_1024 0x04
#define OSR_2048 0x06
#define OSR_4096 0x08
#define OSR_8192 0x0A

#define OSR OSR_2048

static unsigned char n_crc;                     // calculated check sum to ensure PROM integrity
static uint16_t pcal[8];                        // calibration constants from MS5637 PROM registers
float t_conv;                                   // ADC conversion time [us]
unsigned long t_ms5637 = 0;                     // timing variable [us]
float t_intg_ms5637;                            // barometer data integration time [s]
uint64_t d1, d2;                                // raw MS5637 pressure and temperature data
uint64_t dT, offset, sens, t2, sens2, off2;     // general pressure calculation constants
bool command_rotation_flag;                     // defines sequence of base commands for pressure / temperature conversions
static float temp, pressureBaro, alt, prev_alt; // general altitude calculation variables
float array_alt[50];                            // array for simple moving average filtration of altitude data
float NaN = 0. / 0.;                            // represents not-a-number
uint8_t i_alt = 0;                              // index of variable
float velocity_alt;                             // value of the vertical speed, given by differentiation of altitude change (initialized as nan)

static void ms5637_reset(void) {
  twi_data_tx = MS5637_RESET;
  ret_code_t err_code;
  err_code = nrf_drv_twi_tx(&m_twi_baro, MS5637_ADDR, &twi_data_tx, sizeof(twi_data_tx), true);
}

static void ms5637_prom_read() { // read callibration data
  ret_code_t err_code;
  static uint8_t twi_rx_buffer[2] = {0, 0};

  for (uint8_t ii = 0; ii < 7; ii++) {
    twi_data_tx = ((0xA0) | ii << 1); // PROM READ commands are 0xA0 for C1, 0xA2 for C2, 0xA4 for C3, 0xA6 for C4, 0xA8 for C5, 0xAA for C6, 0xAE for serial number and CRC check sum
    err_code = nrf_drv_twi_tx(&m_twi_baro, MS5637_ADDR, &twi_data_tx, sizeof(twi_data_tx), true);
    err_code = nrf_drv_twi_rx(&m_twi_baro, MS5637_ADDR, &twi_rx_buffer[0], sizeof(twi_rx_buffer));
    pcal[ii] = (twi_rx_buffer[0] << 8) | twi_rx_buffer[1];
  }
  // NRF_LOG_INFO("MS5637 PROM C1 = %d", pcal[0]);
  // NRF_LOG_INFO("MS5637 PROM C2 = %d", pcal[1]);
  // NRF_LOG_INFO("MS5637 PROM C3 = %d", pcal[2]);
  // NRF_LOG_INFO("MS5637 PROM C4 = %d", pcal[3]);
  // NRF_LOG_INFO("MS5637 PROM C5 = %d", pcal[4]);
  // NRF_LOG_INFO("MS5637 PROM C6 = %d", pcal[5]);
  // NRF_LOG_INFO("MS5637 PROM C7 = %d", pcal[6]);
  //  NRF_LOG_FLUSH();
}

static unsigned char ms5637_check_crc(uint16_t n_prom[]) // сalculate check sum from MS5637 PROM register contents to ensure data integrity
{
  int cnt;
  unsigned int n_rem = 0;
  unsigned char n_bit;

  n_prom[0] = ((n_prom[0]) & 0x0FFF); // replace CRC byte by 0 for checksum calculation
  n_prom[7] = 0;
  for (cnt = 0; cnt < 16; cnt++) {
    if (cnt % 2 == 1)
      n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
    else
      n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
    for (n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000)
        n_rem = (n_rem << 1) ^ 0x3000;
      else
        n_rem = (n_rem << 1);
    }
  }
  n_rem = ((n_rem >> 12) & 0x000F);
  return (n_rem ^ 0x00);
}

static void ms5637_process() // read raw pressure data,send request for the next conversion
{
  //
  // Read ADC, requested in previous cycle
  //
  static uint8_t twi_rx_buffer[3] = {0, 0, 0}; // RX buffer
  static uint8_t i = 0;
  twi_data_tx = MS5637_ADC_READ;
  ret_code_t err_code;
  err_code = nrf_drv_twi_tx(&m_twi_baro, MS5637_ADDR, &twi_data_tx, sizeof(twi_data_tx), true);
  err_code = nrf_drv_twi_rx(&m_twi_baro, MS5637_ADDR, &twi_rx_buffer[0], sizeof(twi_rx_buffer)); // the read of ADC has been performed here

  //
  // Initiate next ADC conversion, rotating pressure and temperature commands
  //
  if (command_rotation_flag == 0) { // request pressure conversion
    twi_data_tx = MS5637_CONVERT_D2 | OSR;
    err_code = nrf_drv_twi_tx(&m_twi_baro, MS5637_ADDR, &twi_data_tx, sizeof(twi_data_tx), true);

    d1 = twi_rx_buffer[0] << 16 | twi_rx_buffer[1] << 8 | twi_rx_buffer[2]; // construct raw temperature data from RX buffer
    command_rotation_flag = 1;                                              // invert command on the next step
    // NRF_LOG_INFO("MS5637 D1 = %d", d1);
  } else {
    twi_data_tx = MS5637_CONVERT_D1 | OSR;
    err_code = nrf_drv_twi_tx(&m_twi_baro, MS5637_ADDR, &twi_data_tx, sizeof(twi_data_tx), true);

    d2 = twi_rx_buffer[0] << 16 | twi_rx_buffer[1] << 8 | twi_rx_buffer[2]; // construct raw pressure data from RX buffer
    command_rotation_flag = 0;                                              // invert command on the next step
    // NRF_LOG_INFO("MS5637 D2 = %d", d2);
  }
}

float ms5637_calc() { // calculate pressure altitude
  if (d1 > 0 && d2 > 0) {
    // pcal[1] = 46372;  // coefficients from datasheet example
    // pcal[2] = 43981;
    // pcal[3] = 29059;
    // pcal[4] = 27842;
    // pcal[5] = 31553;
    // pcal[6] = 28165;

    // NRF_LOG_INFO("MS5637 C1 = %d", pcal[1]);
    // NRF_LOG_INFO("MS5637 C2 = %d", pcal[2]);
    // NRF_LOG_INFO("MS5637 C3 = %d", pcal[3]);
    // NRF_LOG_INFO("MS5637 C4 = %d", pcal[4]);
    // NRF_LOG_INFO("MS5637 C5 = %d", pcal[5]);
    // NRF_LOG_INFO("MS5637 C6 = %d", pcal[6]);

    // d1 = 6465444;
    // d2 = 8077636;

    // NRF_LOG_INFO("MS5637 D1 = %d", d1);
    // NRF_LOG_INFO("MS5637 D2 = %d", d2);

    int64_t dT = d2 - pcal[5] * pow(2, 8);           // these variable's size does not exceed int32; however, use int64 to avoid further math errors with next int64 variables
    int64_t temp = 2000 + dT * pcal[6] / pow(2, 23); // int32
    int64_t off = pcal[2] * pow(2, 17) + (pcal[4] * dT) / pow(2, 6);
    int64_t sens = pcal[1] * pow(2, 16) + (pcal[3] * dT) / pow(2, 7);
    int64_t p = (d1 * sens / pow(2, 21) - off) / pow(2, 15); // int32

    // NRF_LOG_INFO("MS5637 dT = %d", dT);
    // NRF_LOG_INFO("MS5637 TEMP = %d", temp);
    //  NRF_LOG_INFO("MS5637 OFF = %d", off);  // not valid output - NRF_LOG works with int32-sized variables
    //  NRF_LOG_INFO("MS5637 SENS = %d", sens);  // not valid output - NRF_LOG works with int32-sized variables
    // NRF_LOG_INFO("MS5637 P = %d", p);

    if (temp > 2000) {
      t2 = 5 * dT * dT / pow(2, 38); // correction for high temperatures
      off2 = 0;
      sens2 = 0;
    }
    if (temp < 2000) // correction for low temperature
    {
      t2 = 3 * dT * dT / pow(2, 33);
      off2 = 61 * (temp - 2000) * (temp - 2000) / 16;
      sens2 = 29 * (temp - 2000) * (temp - 2000) / 16;
    }
    if (temp < -1500) // correction for very low temperature
    {
      off2 = off2 + 17 * (temp + 1500) * (temp + 1500);
      sens2 = sens2 + 9 * (temp + 1500) * (temp + 1500);
    }
    temp = temp - t2 / 100; // second order temperature
    off = off - off2;
    sens = sens - sens2;
    p = (d1 * sens / pow(2, 21) - off) / pow(2, 15); // temperature compensated pressure
    pressureBaro = (float)p;
    alt = 44330.769 * (1 - pow((pressureBaro / 100013.25), 0.190284)); // pressure altitude above mean sea level in meters

    // NRF_LOG_INFO("BAROMETER Temperature = " NRF_LOG_FLOAT_MARKER " C", NRF_LOG_FLOAT(temp / 100.0));
    // NRF_LOG_INFO("BAROMETER Pressure = " NRF_LOG_FLOAT_MARKER " hPa", NRF_LOG_FLOAT(p / 100.0));
    // NRF_LOG_INFO("BAROMETER Altitude = " NRF_LOG_FLOAT_MARKER " m", NRF_LOG_FLOAT(alt));
  }
}

static void twi_scan(void) { // scanning for TWI devices
  ret_code_t err_code;
  bool detected_twi_device = false;

  twi_data_tx = 0x01; // random data

  for (twi_address = 0; twi_address < TWI_ADDRESSES; twi_address++) {
    err_code = nrf_drv_twi_tx(&m_twi_baro, twi_address, &twi_data_tx, sizeof(twi_data_tx), true);
    if (err_code == NRF_SUCCESS) {
      detected_twi_device = true;
      NRF_LOG_INFO("I2C device detected at address 0x%x", twi_address); // TWI is two wire interface - unlicensed I2C
      NRF_LOG_FLUSH();
    }
  }
  if (!detected_twi_device) {
    NRF_LOG_INFO("No device was found");
    NRF_LOG_FLUSH();
  }
}

static bool baroOk = 0;

static void baro_init(void) { // TWI init
  ret_code_t err_code;
  const nrf_drv_twi_config_t twi_config = {
      .scl = 5,
      .sda = 4,
      .frequency = NRF_DRV_TWI_FREQ_100K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .clear_bus_init = true};

  err_code = nrf_drv_twi_init(&m_twi_baro, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi_baro);

  ms5637_reset();
  nrf_delay_ms(50);
  ms5637_prom_read();
  unsigned char ref_crc = pcal[0] >> 12; // extract MS5637 PROM cheksum
  n_crc = ms5637_check_crc(pcal);
  // NRF_LOG_INFO("pcal[0] = 0x%x", pcal[0]);

  if (ref_crc > 0) {
    if (ref_crc == n_crc) {
      baroOk = 1;
      NRF_LOG_INFO("MS5637 initialization SUCCESS"); // we has succesfully checked that checksum calculated and read from the memory are equal - that means baro is OK
    } else {
      twi_scan();
      NRF_LOG_INFO("MS5637 PROM checksum WRONG (should be 0x%x): 0x%x", ref_crc, n_crc);
    }
  } else {
    twi_scan();
    NRF_LOG_INFO("MS5637 initialization ERROR");
  }
}

//------------------------------------------------------------------------------------
// ICM20948 part
//------------------------------------------------------------------------------------

#define MPU_INT_PIN 14

#define USE_ACC 1
#define USE_GYRO 1
#define USE_MAG 0
#define USE_GRAVITY 1
#define USE_LINACC 1
#define USE_RV 0   // 1   // requires COMPASS
#define USE_TILT 0 //  *****test

// #define CHIP_AWAKE          (0x01)
// #define CHIP_LP_ENABLE      (0x02)

/*
#define USE_RAW_ACC  1
#define USE_RAW_GYR  1
#define USE_RAW_MAG  1
#define USE_GRV      0
#define USE_CAL_ACC  0
#define USE_CAL_GYR  0
#define USE_CAL_MAG  0
#define USE_UCAL_GYR 0
#define USE_UCAL_MAG 0
#define USE_GEORV    0  // requires COMPASS
#define USE_ORI      0  // requires COMPASS
#define USE_STEPC    0
#define USE_STEPD    0
#define USE_SMD      0
#define USE_BAC      0
#define USE_TILT     0
#define USE_PICKUP   0
#define USE_GRAVITY  0
#define USE_B2S      0
*/

// https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/examples/index.htm
//  https://www.vcalc.com/wiki/vCalc/Quaternion+Multiplication
//  https://www.andre-gaschler.com/rotationconverter/

bool use_rv = 0; // 1   // requires COMPASS
bool use_linacc = 1;
bool new_data;
bool timer_event;
bool start_sensor = false;

int rc = 0; // for ICM20948 code
uint8_t whoami = 0xFF;
uint8_t DMP_pointer = 0;
uint8_t DMP_acq = 0; // 0 = off, x = nb of sample

// FSR configurations
int32_t cfg_acc_fsr = 4;   // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 500; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

APP_TIMER_DEF(m_timer); // new** for timer event

int twi_init_imu(void);
static void my_sensor_event_cb(const inv_sensor_event_t *event, void *arg);

// states for icm20948 device object
static inv_device_icm20948_t device_icm20948;
static inv_device_t *device; // just a handy variable to keep the handle to device object
static inv_sensor_event_t inv_event;

// corresponds to quat  qxyz : 0,0.7071067811865475,0,0.7071067811865475
static const float cfg_mounting_matrix[9] = {
    0, 0, 1.f,
    0, -1.f, 0,
    1.f, 0, 0};

// corresponds to quat  qxyz : 0,-0.7071067811865475,0,0.7071067811865475
static const float compass_mounting_matrix[9] = {
    0, 0, -1.f,
    0, 1.f, 0,
    1.f, 0, 0};

// a listener object will handle sensor events
static const inv_sensor_listener_t sensor_listener = {
    my_sensor_event_cb, // * callback that will receive sensor events * /
    0                   // * some pointer passed to the callback * /
};

static uint8_t dmp3_image[] = {
#include "icm20948_img.dmp3a.h"
};

const char *activityName(int act) {
  switch (act) {
  case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN:
    return "BEGIN IN_VEHICLE";
  case INV_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN:
    return "BEGIN WALKING";
  case INV_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN:
    return "BEGIN RUNNING";
  case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN:
    return "BEGIN ON_BICYCLE";
  case INV_SENSOR_BAC_EVENT_ACT_TILT_BEGIN:
    return "BEGIN TILT";
  case INV_SENSOR_BAC_EVENT_ACT_STILL_BEGIN:
    return "BEGIN STILL";
  case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_END:
    return "END IN_VEHICLE";
  case INV_SENSOR_BAC_EVENT_ACT_WALKING_END:
    return "END WALKING";
  case INV_SENSOR_BAC_EVENT_ACT_RUNNING_END:
    return "END RUNNING";
  case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_END:
    return "END ON_BICYCLE";
  case INV_SENSOR_BAC_EVENT_ACT_TILT_END:
    return "END TILT";
  case INV_SENSOR_BAC_EVENT_ACT_STILL_END:
    return "END STILL";
  default:
    return "unknown activity!";
  }
}

static void icm20948_apply_mounting_matrix(void) {
  /*int ii;

  for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++) {
          inv_icm20948_set_matrix(&device_icm20948.icm20948_states, cfg_mounting_matrix, ii);*/

  inv_device_icm20948_set_sensor_mounting_matrix(device, INV_SENSOR_TYPE_GYROSCOPE, cfg_mounting_matrix);
  inv_device_icm20948_set_sensor_mounting_matrix(device, INV_SENSOR_TYPE_MAGNETOMETER, compass_mounting_matrix);
}

// serif_hal object that abstract low level serial interface between host and device
static int my_serif_read_reg(void *context, uint8_t reg, uint8_t *data, uint32_t len) {
  twi_read_reg(reg, data, len);
  return 0;
}

static int my_serif_write_reg(void *context, uint8_t reg, const uint8_t *data, uint32_t len) {
  twi_write_reg(reg, (uint8_t *)data, len);
  return 0;
}

const inv_serif_hal_t serif_instance = {
    my_serif_read_reg,      // user read_register() function that reads a register over the serial interface
    my_serif_write_reg,     // user write_register() function that writes a register over the serial interface
    1024 * 16,              // 128,                    // maximum number of bytes allowed per read transaction
    1024 * 16,              // 128,                    // maximum number of bytes allowed per write transaction
    INV_SERIF_HAL_TYPE_I2C, // type of the serial interface used between SPI or I2C
    (void *)0xDEADBEEF      // some context pointer passed to read/write callbacks
};

uint32_t ticks_from = 0;
uint32_t timestamp_ms = 0;

uint32_t timestamp_func(uint32_t newtime) {
  uint32_t ticks_diff = 0;
  uint32_t ms_diff = 0;
  uint32_t ticks_to = NRF_RTC0->COUNTER; // app_timer_cnt_get();

  ticks_diff = app_timer_cnt_diff_compute(ticks_to, ticks_from);
  ticks_from = ticks_to;

  ms_diff = APP_TIMER_MS(ticks_diff, APP_TIMER_PRESCALER); //  see timestamp.h
                                                           //  ms_diff = (ticks_diff * (APP_TIMER_PRESCALER + 1) * 1000) / APP_TIMER_CLOCK_FREQ;
  if (newtime != NULL) {
    timestamp_ms = newtime;
    // printf( "timestamp_ms = %d %d\n", newtime , timestamp_ms);
  } else
    timestamp_ms += ms_diff;

  return timestamp_ms;
}

void timer_start(uint16_t duration) {                                         // NRF_LOG_INFO("apptimer start");
  APP_ERROR_CHECK(app_timer_start(m_timer, APP_TIMER_TICKS(duration), NULL)); // APP_ERROR_CHECK(err_code);
  nrf_delay_ms(2);
}

void timer_stop() { app_timer_stop(m_timer); }

void inv_icm20948_sleep_us(int us) { nrf_delay_us(us); }

uint64_t inv_icm20948_get_time_us(void) {
  // uint64_t ticks = (uint64_t)((uint64_t)overflows << (uint64_t)24) | (uint64_t)(NRF_RTC1->COUNTER);
  // return (ticks * 1000000) / 32768;
  return (timestamp_func(NULL));
}

static float rawAccVal[3] = {0}; // 3 values are for: x, y, z
static float rawGyroVal[3] = {0};
static float accVal[3] = {0};
static float gyroVal[3] = {0};
static float magVal[3] = {0};
static float accBias[3] = {0};
static float gyroBias[3] = {0};
static float magBias[3] = {0};
static float uncalAccVal[3] = {0};
static float uncalGyroVal[3] = {0};
static float uncalMagVal[3] = {0};
static float linearAccVal[3] = {0};
static float gravVal[3] = {0};
static float gameQuatVal[4] = {0};   // Game Rotation Vector, 6-axis fusion (no magnetometer) - quaternion
static float geomagQuatVal[4] = {0}; // Geomagnetic Rotation Vector, 6-axis fusion (no gyroscope)  - quaternion
static float quatVal[4] = {0};       // Rotation Vector, 9-axis fusion  - quaternion
static float orientVal[3] = {0};     // based on 9-axis fusion - euler angles

uint8_t gyroAccuracy = 0; // ranges from 0 to 3 where 3 is fully calibrated most accurate sensor, 0 is not calibrated and strictly raw data
uint8_t magAccuracy = 0;
uint8_t uncalAccAccuracy = 0;
uint8_t uncalGyroAccuracy = 0;
uint8_t uncalMagAccuracy = 0;
uint8_t linearAccAccuracy = 0;
uint8_t gravAccuracy = 0;
uint8_t gameQuatAccuracy = 0;
uint8_t geomagQuatAccuracy = 0;
uint8_t quatAccuracy = 0;
uint8_t orientAccuracy = 0;

void vertical_acceleration_process(void);

// Callback called upon sensor event reception. This function is called in the same function than inv_device_poll()
static void my_sensor_event_cb(const inv_sensor_event_t *event, void *arg) // data are in....
{
  bool gyro_event = 0;
  bool rv_event = 0;
  // NRF_LOG_INFO("my_sensor_event_cb");

  // ... do something with event
  if (event->status == INV_SENSOR_STATUS_DATA_UPDATED) {
    switch (INV_SENSOR_ID_TO_TYPE(event->sensor)) {
    case INV_SENSOR_TYPE_RAW_ACCELEROMETER: {
      rawAccVal[0] = event->data.acc.vect[0];
      rawAccVal[1] = event->data.acc.vect[1];
      rawAccVal[2] = event->data.acc.vect[2];
      // rawAccAccuracy = event->data.acc.accuracy_flag;
      // NRF_LOG_INFO("ACCELEROMETER x: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(rawAccVal[0]));
      // NRF_LOG_INFO("ACCELEROMETER y: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(rawAccVal[1]));
      // NRF_LOG_INFO("ACCELEROMETER z: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(rawAccVal[2]));
      // NRF_LOG_INFO("ACCELEROMETER accuracy: %d", rawAccAccuracy); // shall be always 0
    } break;
    case INV_SENSOR_TYPE_RAW_GYROSCOPE: {
      rawGyroVal[0] = event->data.gyr.vect[0];
      rawGyroVal[1] = event->data.gyr.vect[1];
      rawGyroVal[2] = event->data.gyr.vect[2];
      // rawGyroAccuracy = event->data.gyr.accuracy_flag;
      // NRF_LOG_INFO("UNCALIBRATED GYROSCOPE x: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(rawGyroVal[0]));
      // NRF_LOG_INFO("UNCALIBRATED GYROSCOPE y: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(rawGyroVal[1]));
      // NRF_LOG_INFO("UNCALIBRATED GYROSCOPE z: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(rawGyroVal[2]));
      // NRF_LOG_INFO("UNCALIBRATED GYROSCOPE accuracy: %d", rawGyroAccuracy); // shall be always 0
    } break;
    case INV_SENSOR_TYPE_ACCELEROMETER: {
      accVal[0] = event->data.acc.vect[0];
      accVal[1] = event->data.acc.vect[1];
      accVal[2] = event->data.acc.vect[2];

      accBias[0] = event->data.acc.bias[0];
      accBias[1] = event->data.acc.bias[1];
      accBias[2] = event->data.acc.bias[2];

      // accAccuracy = event->data.acc.accuracy_flag;

      // NRF_LOG_INFO("ACCELEROMETER x: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(accVal[0]));
      // NRF_LOG_INFO("ACCELEROMETER y: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(accVal[1]));
      // NRF_LOG_INFO("ACCELEROMETER z: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(accVal[2]));

      // NRF_LOG_INFO("ACCELEROMETER bias x: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(accBias[0]));
      // NRF_LOG_INFO("ACCELEROMETER bias y: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(accBias[1]));
      // NRF_LOG_INFO("ACCELEROMETER bias z: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(accBias[2]));

      // NRF_LOG_INFO("ACCELEROMETER accuracy: %d", accAccuracy); // shall be always 0, accelerometer is raw accelerometer value scaled to output value in g; associated accuracy will always be 0 since not mapped to calibrated accelerometer data
    } break;
    case INV_SENSOR_TYPE_LINEAR_ACCELERATION: {
      linearAccVal[0] = event->data.acc.vect[0];
      linearAccVal[1] = event->data.acc.vect[1];
      linearAccVal[2] = event->data.acc.vect[2];
      linearAccAccuracy = event->data.acc.accuracy_flag;
      // NRF_LOG_INFO("LINEAR ACCELERATION x: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(linearAccVal[0]));
      // NRF_LOG_INFO("LINEAR ACCELERATION y: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(linearAccVal[1]));
      // NRF_LOG_INFO("LINEAR ACCELERATION z: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(linearAccVal[2]));
      //  NRF_LOG_INFO("LINEAR ACCELERATION accuracy: %d", linearAccAccuracy);
    } break;
    case INV_SENSOR_TYPE_GRAVITY: {
      gravVal[0] = event->data.acc.vect[0];
      gravVal[1] = event->data.acc.vect[1];
      gravVal[2] = event->data.acc.vect[2];
      gravAccuracy = event->data.acc.accuracy_flag;
      // NRF_LOG_INFO("GRAVITY x: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(gravVal[0]));
      // NRF_LOG_INFO("GRAVITY y: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(gravVal[1]));
      // NRF_LOG_INFO("GRAVITY z: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(gravVal[2]));
      // NRF_LOG_INFO("GRAVITY accuracy: %d", gravAccuracy);
    } break;
    case INV_SENSOR_TYPE_GYROSCOPE: {
      gyroVal[0] = event->data.gyr.vect[0];
      gyroVal[1] = event->data.gyr.vect[1];
      gyroVal[2] = event->data.gyr.vect[2];

      gyroBias[0] = event->data.gyr.bias[0];
      gyroBias[1] = event->data.gyr.bias[1];
      gyroBias[2] = event->data.gyr.bias[2];

      gyroAccuracy = event->data.gyr.accuracy_flag;

      // NRF_LOG_INFO("GYROSCOPE x: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gyroVal[0]));
      // NRF_LOG_INFO("GYROSCOPE y: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gyroVal[1]));
      // NRF_LOG_INFO("GYROSCOPE z: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gyroVal[2]));

      // NRF_LOG_INFO("GYROSCOPE bias x: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gyroBias[0]));
      // NRF_LOG_INFO("GYROSCOPE bias y: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gyroBias[1]));
      // NRF_LOG_INFO("GYROSCOPE bias z: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gyroBias[2]));

      // NRF_LOG_INFO("GYROSCOPE accuracy: %d", gyroAccuracy);
    } break;
    case INV_SENSOR_TYPE_MAGNETOMETER: {
      magVal[0] = event->data.mag.vect[0];
      magVal[1] = event->data.mag.vect[1];
      magVal[2] = event->data.mag.vect[2];
      magAccuracy = event->data.mag.accuracy_flag;
      // NRF_LOG_INFO("MAGNETOMETER x: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(magVal[0]));
      // NRF_LOG_INFO("MAGNETOMETER y: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(magVal[1]));
      // NRF_LOG_INFO("MAGNETOMETER z: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(magVal[2]));
      // NRF_LOG_INFO("MAGNETOMETER accuracy: %d", magAccuracy);
    } break;
    case INV_SENSOR_TYPE_UNCAL_GYROSCOPE: {
      uncalGyroVal[0] = event->data.gyr.vect[0];
      uncalGyroVal[1] = event->data.gyr.vect[1];
      uncalGyroVal[2] = event->data.gyr.vect[2];
      uncalGyroAccuracy = event->data.gyr.accuracy_flag;
      // NRF_LOG_INFO("UNCALIBRATED GYROSCOPE x: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(uncalGyroVal[0]));
      // NRF_LOG_INFO("UNCALIBRATED GYROSCOPE y: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(uncalGyroVal[1]));
      // NRF_LOG_INFO("UNCALIBRATED GYROSCOPE z: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(uncalGyroVal[2]));
      // NRF_LOG_INFO("UNCALIBRATED GYROSCOPE accuracy: %d", uncalGyroAccuracy);
    } break;
    case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER: {
      // magBias[0] = event->data.mag.bias[0];
      // magBias[1] = event->data.mag.bias[1];
      // magBias[2] = event->data.mag.bias[2];
      // NRF_LOG_INFO("UNCAL_MAGNETOMETER_BIAS x: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(magBias[0]));
      // NRF_LOG_INFO("UNCAL_MAGNETOMETER_BIAS y: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(magBias[1]));
      // NRF_LOG_INFO("UNCAL_MAGNETOMETER_BIAS z: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(magBias[2]));

      uncalMagVal[0] = event->data.mag.vect[0];
      uncalMagVal[1] = event->data.mag.vect[1];
      uncalMagVal[2] = event->data.mag.vect[2];
      uncalMagAccuracy = event->data.mag.accuracy_flag;
      // NRF_LOG_INFO("UNCALIBRATED MAGNETOMETER x: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(uncalMagVal[0]));
      // NRF_LOG_INFO("UNCALIBRATED MAGNETOMETER y: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(uncalMagVal[1]));
      // NRF_LOG_INFO("UNCALIBRATED MAGNETOMETER z: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(uncalMagVal[2]));
      // NRF_LOG_INFO("UNCALIBRATED MAGNETOMETER accuracy: %d", magAccuracy);
    } break;
    case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR: { // 6 axis (accelerometer + gyroscope)
      gameQuatVal[0] = event->data.quaternion.quat[0];
      gameQuatVal[1] = event->data.quaternion.quat[1];
      gameQuatVal[2] = event->data.quaternion.quat[2];
      gameQuatVal[3] = event->data.quaternion.quat[3];
      gameQuatAccuracy = event->data.quaternion.accuracy_flag;
      // NRF_LOG_INFO("GAME ROTATION VECTOR q1: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(gameQuatVal[0]));
      // NRF_LOG_INFO("GAME ROTATION VECTOR q2: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(gameQuatVal[1]));
      // NRF_LOG_INFO("GAME ROTATION VECTOR q3: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(gameQuatVal[2]));
      // NRF_LOG_INFO("GAME ROTATION VECTOR q4: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(gameQuatVal[2]));
      // NRF_LOG_INFO("GAME ROTATION VECTOR accuracy: %d", gameQuatAccuracy);
    } break;
    case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR: { // 6 axis (accelerometer + magnetometer)
      geomagQuatVal[0] = event->data.quaternion.quat[0];
      geomagQuatVal[1] = event->data.quaternion.quat[1];
      geomagQuatVal[2] = event->data.quaternion.quat[2];
      geomagQuatVal[3] = event->data.quaternion.quat[3];
      geomagQuatAccuracy = event->data.quaternion.accuracy_flag;
      // NRF_LOG_INFO("GEOMAG ROTATION VECTOR q1: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(geomagQuatVal[0]));
      // NRF_LOG_INFO("GEOMAG ROTATION VECTOR q2: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(geomagQuatVal[1]));
      // NRF_LOG_INFO("GEOMAG ROTATION VECTOR q3: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(geomagQuatVal[2]));
      // NRF_LOG_INFO("GEOMAG ROTATION VECTOR q4: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(geomagQuatVal[2]));
      // NRF_LOG_INFO("GEOMAG ROTATION VECTOR accuracy: %d", geomagQuatAccuracy);
    } break;
    case INV_SENSOR_TYPE_ROTATION_VECTOR: { // 9 axis (accelerometer + gyroscope + magnetometer)
      quatVal[0] = event->data.quaternion.quat[0];
      quatVal[1] = event->data.quaternion.quat[1];
      quatVal[2] = event->data.quaternion.quat[2];
      quatVal[3] = event->data.quaternion.quat[3];
      quatAccuracy = event->data.quaternion.accuracy_flag;
      // NRF_LOG_INFO("ROTATION VECTOR q1: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(quatVal[0]));
      // NRF_LOG_INFO("ROTATION VECTOR q2: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(quatVal[1]));
      // NRF_LOG_INFO("ROTATION VECTOR q3: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(quatVal[2]));
      // NRF_LOG_INFO("ROTATION VECTOR q4: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(quatVal[2]));
      // NRF_LOG_INFO("ROTATION VECTOR accuracy: %d", quatAccuracy);
    } break;
    case INV_SENSOR_TYPE_ORIENTATION: {
      orientVal[0] = event->data.acc.vect[0];
      orientVal[1] = event->data.acc.vect[1];
      orientVal[2] = event->data.acc.vect[2];
      orientAccuracy = event->data.orientation.accuracy_flag;
      // NRF_LOG_INFO("ORIENTATION x: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(orientVal[0]));
      // NRF_LOG_INFO("ORIENTATION y: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(orientVal[1]));
      // NRF_LOG_INFO("ORIENTATION z: " NRF_LOG_FLOAT_MARKER,  NRF_LOG_FLOAT(orientVal[2]));
      // NRF_LOG_INFO("ORIENTATION accuracy: %d", orientAccuracy);
    } break;
    case INV_SENSOR_TYPE_BAC: {
    } break;
    case INV_SENSOR_TYPE_STEP_COUNTER: {
    } break;
    case INV_SENSOR_TYPE_TILT_DETECTOR: {
    } break;
    case INV_SENSOR_TYPE_PICK_UP_GESTURE: {
    } break;
    case INV_SENSOR_TYPE_STEP_DETECTOR: {
    } break;
    case INV_SENSOR_TYPE_SMD: {
    } break;
    case INV_SENSOR_TYPE_B2S: {
    } break;
    default: {
      // NRF_LOG_INFO("data event %s : ...", inv_sensor_str(event->sensor));
    } break;
    } // switch

    // next send or store RV data
    if (DMP_acq == 1) { // continuous mode, send RV according to gyro_event freq -  RV and LIN_acc are fixed at 56 HZ
      if (gyro_event == 1) {
        gyro_event = 0;
      }
    } else if ((DMP_acq == 2) && (rv_event == 1)) { // continuous mode  send RV according to gyro_event freq -  RV and LIN_acc are fixed at 56 HZ
      rv_event = 0;
    }
  }
}

void data_ready_cb(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  new_data = true;
  // NRF_LOG_INFO("data ready");
}

// Function for initiating the GPIOTE module enabling an interrupt on a Low-To-High event on pin MPU_INT_PIN
static ret_code_t twi_gpiote_init(void) {
  nrfx_err_t err_code;

  if (!nrfx_gpiote_is_init()) {
    err_code = nrfx_gpiote_init();
    VERIFY_SUCCESS(err_code);
  }

  nrfx_gpiote_in_config_t imu_interupt_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
  err_code = nrf_drv_gpiote_in_init(MPU_INT_PIN, &imu_interupt_config, data_ready_cb);
  APP_ERROR_CHECK(err_code);

  nrfx_gpiote_in_event_enable(MPU_INT_PIN, true); // attach interrupt to INT pin of ICM20948 (MPU_INT_PIN)
}

static void timer_handler(void *p_context) {
  // NRF_LOG_INFO("timer event");
  timer_event = true;
}

void imu_set_bias() {
  int mbias[3] = {0};
  int abias[3] = {0};
  int gbias[3] = {0};

  uint8_t err_code;

  /*mbias[0] = -417248; // from original code
  mbias[1] = -2138048;
  mbias[2] = 123232;
  abias[0] = -290;
  abias[1] = -790;
  abias[2] = 1090;
  gbias[0] = -69264;
  gbias[1] = 62744;
  gbias[2] = -4480;*/

  err_code = inv_icm20948_set_bias(&device_icm20948.icm20948_states, INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, &mbias);
  err_code += inv_icm20948_set_bias(&device_icm20948.icm20948_states, INV_ICM20948_SENSOR_ACCELEROMETER, &abias);
  err_code += inv_icm20948_set_bias(&device_icm20948.icm20948_states, INV_ICM20948_SENSOR_GYROSCOPE, &gbias);

  // inv_device_icm20948_set_sensor_config(device , INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,INV_DEVICE_ICM20948_CONFIG_OFFSET,&mbias, 3);
  //  to be tested
}

void store_offsets(void) {
  int rc = 0;
  uint8_t i, idx = 0;
  int gyro_bias_q16[6] = {0}, acc_bias_q16[6] = {0};

  uint8_t sensor_bias[84] = {0};

  // strore self-test bias in NV memory
  rc = inv_device_get_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE, INV_DEVICE_ICM20948_CONFIG_OFFSET, gyro_bias_q16, sizeof(gyro_bias_q16));
  NRF_LOG_INFO("Gyro_bias %d, %d, %d, %d, %d, %d", gyro_bias_q16[0], gyro_bias_q16[1], gyro_bias_q16[2], gyro_bias_q16[3], gyro_bias_q16[4], gyro_bias_q16[5]);

  // for(i = 0; i < 6; i++)
  // inv_dc_int32_to_little8(gyro_bias_q16[i], &sensor_bias[i * sizeof(uint32_t)]);
  // idx += sizeof(gyro_bias_q16);

  rc = inv_device_get_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_DEVICE_ICM20948_CONFIG_OFFSET, acc_bias_q16, sizeof(acc_bias_q16));
  NRF_LOG_INFO("Acc_bias %d, %d, %d, %d, %d, %d, %d", acc_bias_q16[0], acc_bias_q16[1], acc_bias_q16[2], acc_bias_q16[3], acc_bias_q16[4], acc_bias_q16[5]);

  // for(i = 0; i < 6; i++)
  // inv_dc_int32_to_little8(acc_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
  // idx += sizeof(acc_bias_q16);

  // flash_manager_write_data(sensor_bias);
}

static bool imuOk = 0;

void imu_init(void) {
  ret_code_t err_code;

  twi_gpiote_init(); // init GPIO interrupt
  twi_init_imu();
  inv_device_icm20948_init2(&device_icm20948, &serif_instance, &sensor_listener, dmp3_image, sizeof(dmp3_image)); // init I2C icm20948Setup
  device = inv_device_icm20948_get_base(&device_icm20948);
  inv_device_icm20948_reset(device); // cleanup + setup  and  prevent hang on restart
  nrf_delay_ms(50);

  inv_device_icm20948_whoami(device, &whoami);
  if (whoami > 0) {
    if (whoami == 0x0EA) {
      imuOk = 1;
      NRF_LOG_INFO("ICM20948 initialization SUCCESS"); // we has succesfully checked that whoami returns correct value for the chip - that means IMU is OK
    } else {
      NRF_LOG_INFO("ICM who am I WRONG (should be 0xEA): 0X%02X", whoami);
    }
  } else {
    NRF_LOG_INFO("ICM20948 initialisation ERROR");
  }

  // err_code = inv_device_self_test (device, INV_SENSOR_TYPE_MAGNETOMETER);  // err_code = 0 if OK // self test requires 10 seconds during which device is unresponsive
  // err_code = inv_device_self_test (device, INV_SENSOR_TYPE_GYROSCOPE );  // err_code = 0 if OK
  // err_code = inv_device_self_test (device, INV_SENSOR_TYPE_MAGNETOMETER );  // err_code = 0 if OK

  inv_icm20948_set_lowpower_or_highperformance(&device_icm20948.icm20948_states, 0); // 1=high-performance, 0=low-power
  nrf_delay_ms(50);
  icm20948_apply_mounting_matrix();
  nrf_delay_ms(50);
  imu_set_bias(); // some bias change with scale - set scale first
  nrf_delay_ms(50);

  err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_RAW_ACCELEROMETER, 5); // period in ms, 1 to 225Hz, 5ms = 200Hz
  inv_device_start_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
  nrf_delay_ms(50);

  err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_RAW_GYROSCOPE, 5); // period in ms, 1 to 225Hz, 5ms = 200Hz
  inv_device_start_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
  nrf_delay_ms(50);

  // err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_RAW_MAGNETOMETER, 15); // period in ms , 1 to 70Hz, 15ms = 66Hz
  // inv_device_start_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
  // nrf_delay_ms(50);

  // err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ACCELEROMETER, 5); // period in ms, 1 to 225Hz, 5ms = 200Hz
  // inv_device_start_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
  // nrf_delay_ms(50);

  // err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GYROSCOPE, 5); // period in ms, 1 to 225Hz, 5ms = 200Hz
  // inv_device_start_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
  // nrf_delay_ms(50);

  // err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_MAGNETOMETER, 5); // period in ms , 1 to 70Hz, 15ms = 66Hz
  // inv_device_start_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
  // nrf_delay_ms(50);

  // err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 20); // period in ms, 50 to 225Hz, 5ms = 200Hz
  // inv_device_start_sensor(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR);
  // nrf_delay_ms(50);

  err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_LINEAR_ACCELERATION, 5); // period in ms, 50 to 225Hz, 5ms = 200Hz
  inv_device_start_sensor(device, INV_SENSOR_TYPE_LINEAR_ACCELERATION);
  nrf_delay_ms(50);

  err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GRAVITY, 5); // period in ms, 50 to 225Hz, 5ms = 200Hz
  inv_device_start_sensor(device, INV_SENSOR_TYPE_GRAVITY);
  nrf_delay_ms(50);

  // err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 5); // period in ms, 50 to 225Hz, 5ms = 200Hz
  // inv_device_start_sensor(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR);
  // nrf_delay_ms(50);

  // err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR, 5); // period in ms, 1 to 225Hz, 5ms = 200Hz
  // inv_device_start_sensor(device, INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR);
  // nrf_delay_ms(50);

  // err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ROTATION_VECTOR, 5); // period in ms, 50 to 225Hz, 5ms = 200Hz
  // inv_device_start_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
  // nrf_delay_ms(50);

  // err_code = inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ORIENTATION, 20); // period in ms, 50 to 225Hz, 5ms = 200Hz
  // inv_device_start_sensor(device, INV_SENSOR_TYPE_ORIENTATION);
  // nrf_delay_ms(50);

  nrfx_gpiote_in_event_enable(MPU_INT_PIN, true); // enable GPIO interrupt
  nrf_delay_ms(50);
}

void icm20948_process(void) {

  // NRF_LOG_INFO("ICM 20948 processing");
  if (new_data == true) {
    // led_pwm_update_duty_cycle(BRIGHT);
    new_data = false;
    rc = inv_device_poll(device);
  }
}

static float vA = 0.0f; // vertical acceleration acrding to imu

void icm20948_calc(void) {
  vA = (linearAccVal[0] * gravVal[0] + linearAccVal[1] * gravVal[1] + linearAccVal[2] * gravVal[2]); // vertical acceleration in G, given by scalar product of linear acceleration vector and unit vector of vertical direction
  // NRF_LOG_INFO("vA = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(vA * 1000.0f));
}

//------------------------------------------------------------------------------------
// Custom functions part
//------------------------------------------------------------------------------------

APP_TIMER_DEF(vario_loop_timer);

static void vario_loop_timer_handler(void *p_context);

static bool varioLoopFlag = 0;

static void vario_loop_timer_handler(void *p_context) { // handler for timer expiration
  varioLoopFlag = 1;
}

APP_TIMER_DEF(millis_rough_timer);

static void millis_rough_timer_handler(void *p_context);

uint32_t millisCount = 0;

static void millis_rough_timer_handler(void *p_context) { // handler for timer expiration
  millisCount++;
}

static void timers_init(void) {
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handler); // from ICM20948 code
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_create(&millis_timer, APP_TIMER_MODE_REPEATED, millis_timer_handler);
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_start(millis_timer, APP_TIMER_TICKS(0xFFFFFF), NULL); // restarts repeatedly every APP_TIMER_TICKS // app_timer_cnt_get() is 24 bit long, so millis() rolls over every (0xFFFFFF / 16.384) = 0xF9FFF milliseconds
  APP_ERROR_CHECK(err_code);

  /*err_code = app_timer_create(&vario_loop_timer, APP_TIMER_MODE_REPEATED, vario_loop_timer_handler);
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_start(vario_loop_timer, APP_TIMER_TICKS(16.384 * 0.34), NULL); // restarts repeatedly every APP_TIMER_TICKS // app_timer_cnt_get() is 24 bit long, so millis() rolls over every (0xFFFFFF / 16.384)= 0xF9FFF milliseconds
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&millis_rough_timer, APP_TIMER_MODE_REPEATED, millis_rough_timer_handler);
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_start(millis_rough_timer, APP_TIMER_TICKS(1), NULL); // restarts repeatedly every APP_TIMER_TICKS // app_timer_cnt_get() is 24 bit long, so millis() rolls over every (0xFFFFFF / 16.384)= 0xF9FFF milliseconds
  APP_ERROR_CHECK(err_code);*/

  NRF_CLOCK->TASKS_HFCLKSTART = 1;
}

int twi_uninit_imu(void);

static void sleep(void) {
  ret_code_t err_code;
  led_pwm_uninit(); // turn off LED

  err_code = sd_power_gpregret_clr(1, 0xFF); // clearing the general purpose retained (non-volatile) register
  err_code = sd_power_gpregret_set(1, 0x01); // setting the flag to let know that a sleep condition took place
  APP_ERROR_CHECK(err_code);
  nrf_gpio_pin_clear(LOAD_EN); // turns off secondary power output
  nrf_gpio_pin_clear(SND_EN);  // turns off sound amplifier
  // app_timer_stop(millis_timer);
  // err_code = nrf_sdh_disable_request();       // turns off softdevice
  // APP_ERROR_CHECK(err_code);
  // #ifdef NDEBUG
  // nrf_drv_clock_uninit();                    // turns off clock
  // #endif

  // nrf_drv_twi_uninit(&m_twi);                // turns off twi, must be after nrf_drv_clock_uninit()
  //*(volatile uint32_t *)0x40004FFC = 0;
  //*(volatile uint32_t *)0x40004FFC;
  //*(volatile uint32_t *)0x40004FFC = 1;

  // #if (__FPU_USED == 1)
  //__set_FPSCR(__get_FPSCR() & ~(0x0000009F));
  //(void) __get_FPSCR();
  //  NVIC_ClearPendingIRQ(FPU_IRQn);
  // #endif

  if (baroOk == 1) {                 // if baro was init
    nrf_drv_twi_uninit(&m_twi_baro); // turns off twi, must be after nrf_drv_clock_uninit()
  }
  if (imuOk == 1) { // if imu was init
    nrf_drv_gpiote_in_event_disable(MPU_INT_PIN);
    twi_uninit_imu(); // turns off twi, must be after nrf_drv_clock_uninit()
    nrfx_gpiote_init();
  }

  nrf_gpio_cfg_input(5, NRF_GPIO_PIN_NOPULL); // turn off
  nrf_gpio_cfg_input(4, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(13, NRF_GPIO_PIN_NOPULL); // from ICM20948
  nrf_gpio_cfg_input(14, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(15, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(16, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(24, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(37, NRF_GPIO_PIN_NOPULL); // from GPS
  nrf_gpio_cfg_input(39, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(41, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(43, NRF_GPIO_PIN_NOPULL);

  nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_STAY_IN_SYSOFF); // deep sleep
}

static uint32_t gpregret_value = 0; // the variable is used to signal reset from wakeup

static void wake_up() {
  ret_code_t err_code;
  uint32_t i = 0;

  err_code = sd_power_gpregret_get(1, &gpregret_value);
  APP_ERROR_CHECK(err_code);

  if (gpregret_value == 1) { // if we came from reset by waking up
    led_pwm_update_duty_cycle(BRIGHT);

    while (nrf_gpio_pin_read(BUTTON)) { // while button is pressed (note that the touch button's integrated circuit will release the signal after 12 seconds in HI state)
      nrf_delay_ms(50);
      i++;
      if (i >= 18) { // after approx 2.5 sec
        led_pwm_update_duty_cycle(MEDIUM);
      }
    }
    if (i < 18 || i > 45) { // if short or too long press
      // NRF_LOG_INFO("led_pwm_uninit() @ wake_up");
      led_pwm_uninit();
      nrf_gpio_pin_clear(LOAD_EN); // turns off secondary power output (which was turned on for QSPI)
      nrf_gpio_pin_clear(13);      // from ICM20948
      nrf_gpio_pin_clear(15);      // from ICM20948
      nrf_gpio_pin_clear(24);      // from ICM20948

      nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_STAY_IN_SYSOFF); // back to sleep
    } else {                                                       // if decided to wake up then turn on loads
      nrf_gpio_cfg_input(13, NRF_GPIO_PIN_PULLDOWN);               // sets ICM20948 address as 0x68
      nrf_gpio_cfg_input(15, NRF_GPIO_PIN_PULLUP);                 // performed prior to powering up ICM20948
      nrf_gpio_cfg_input(24, NRF_GPIO_PIN_PULLUP);                 // performed prior to powering up ICM20948

      nrf_gpio_cfg_output(LOAD_EN);
      nrf_gpio_pin_set(LOAD_EN); // turns on secondary power output
      nrf_gpio_cfg_output(SND_EN);
      nrf_gpio_pin_set(SND_EN); // turns on sound amplifier
    }
  } else {                                         // if the firmware was just loaded then we need to skip button actions and turn on loads
    nrf_gpio_cfg_input(13, NRF_GPIO_PIN_PULLDOWN); // sets ICM20948 address as 0x68
    nrf_gpio_cfg_input(15, NRF_GPIO_PIN_PULLUP);   // performed prior to powering up ICM20948
    nrf_gpio_cfg_input(24, NRF_GPIO_PIN_PULLUP);   // performed prior to powering up ICM20948

    nrf_gpio_cfg_output(LOAD_EN);
    nrf_gpio_pin_set(LOAD_EN); // turns on secondary power output
    nrf_gpio_cfg_output(SND_EN);
    nrf_gpio_pin_set(SND_EN); // turns on sound amplifier
  }

  volatile float batt_sample = adc_sample();
  NRF_LOG_INFO("Battery voltage: " NRF_LOG_FLOAT_MARKER " V ", NRF_LOG_FLOAT(batt_sample));
  if (batt_sample < 3.7) {
    snd_play(3750, 1000); // do 1 second beep if battery is less then 30% charged (discharged)
    while (snd_check()) { // wait while the ending tone is still playing
      snd_check();
    }
  } else if (batt_sample > 3.7) {
    nrf_delay_ms(175);
    snd_play(3750, 150);  // do two beeps if battery is charged over 30% (charged)
    while (snd_check()) { // wait while the ending tone is still playing
      snd_check();
    }
    nrf_delay_ms(175);
    snd_play(3750, 150);
    while (snd_check()) {
      snd_check();
    }
  }
  if (batt_sample > 3.9) {
    nrf_delay_ms(175);
    snd_play(3750, 150);  // do another short beep if battery is charged over 80% (fully charged)
    while (snd_check()) { // wait while the ending tone is still playing
      snd_check();
    }
  }

  err_code = sd_power_gpregret_clr(1, 0xFF); // sets wake up flag
  APP_ERROR_CHECK(err_code);
}

static uint32_t timeoutMillis = 0;
static uint32_t timeoutDeltaT = 0;
static uint32_t timeoutCriterionT = 600000; // criterion for going sleep if nothing happens [ms], default 600000 equal 10 minutes
static bool timeoutPermit = 1;              // takes "true" when allowed to go sleep, user actions may make it "false" delaying sleep countdown

static float climbrateKalman = 0;
static float climbrateSmoothed = 0;
static float altitudeKalman = 0;

static int i_sleep_check = 0;

static uint32_t timeGPSFullPower = 0;
static uint32_t timeGPSLowPower = 0;
static uint32_t timeGPSFullPowerStart = 0;
static uint32_t timeGPSLowPowerStart = 0;
volatile static uint32_t timeGPSLowPowerTotal = 1; // to not divide by 0
volatile static uint32_t timeGPSFullPowerTotal = 1;
volatile uint32_t numberGPSForcedTransitions = 0;
static float powerRatioGPS = 0.0;

static char fatfsString[256] = {0};
float velocityGPS;

static uint32_t ticksTimeoutNow = 0;
static uint32_t ticksTimeoutPrevious = 0;

static void sleep_check(void) { // function to check if we shall be sleeping saving battery
  if (timeoutPermit == 1) {     // climbrateSmoothed < 0.5f && climbrateSmoothed > -0.5f && (velocityGPS * 3.6f) > 15.0f && timeoutPermit == 1) { // if vertical speed is less then +0.5 m/s and more then -0.5 m/s or horizontal speed is less then 15 kph
    // NRF_LOG_INFO("timeoutPermit = %d", timeoutPermit);
    // NRF_LOG_INFO("timeoutDeltaT = %u", timeoutDeltaT);
    // NRF_LOG_FLUSH();

    ticksTimeoutNow = app_timer_cnt_get();
    timeoutDeltaT = app_timer_cnt_diff_compute(ticksTimeoutNow, ticksTimeoutPrevious) / 16.384; // count the time

    // timeoutDeltaT = millis() - timeoutMillis;
    if (timeoutDeltaT > timeoutCriterionT) { // if condition is held for more then 10 sec
      sleep();                               // sleep because of no flying has been happening for a long period - we must be saving battery
    }
  } else {
    ticksTimeoutPrevious = app_timer_cnt_get(); // the timestamp will be updated when vario is active, the last timestamp will be used when the vario will become still
  }
  timeoutPermit = 1; // update permit

  if (adc_sample() < 3.4) {

    /*NRF_LOG_INFO("Total time of log: %d min ", (uint32_t)(millis() / 60000));
    memset(fatfsString, 0, sizeof(fatfsString));
    (void)snprintf(fatfsString, sizeof(fatfsString), "Total time of log session: %d min ", (uint32_t)(millis() / 60000));
    fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString)));
    fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));

    NRF_LOG_INFO("Voltage: " NRF_LOG_FLOAT_MARKER " V ", NRF_LOG_FLOAT(adc_sample()));
    memset(fatfsString, 0, sizeof(fatfsString));
    (void)snprintf(fatfsString, sizeof(fatfsString), "Voltage: %.2f V ", (adc_sample()));
    fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString)));
    fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));

    NRF_LOG_INFO("Time for GPS in full power total = %d milliseconds ", timeGPSFullPowerTotal);
    memset(fatfsString, 0, sizeof(fatfsString));
    (void)snprintf(fatfsString, sizeof(fatfsString), "Time for GPS in full power total: %d milliseconds ", timeGPSFullPowerTotal);
    fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString) - 1));
    fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));

    NRF_LOG_INFO("Time for GPS in low power total = %d milliseconds ", timeGPSLowPowerTotal);
    memset(fatfsString, 0, sizeof(fatfsString));
    (void)snprintf(fatfsString, sizeof(fatfsString), "Time for GPS in low power total: %d milliseconds ", timeGPSLowPowerTotal);
    fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString)));
    fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));

    NRF_LOG_INFO("Number of forced GPS transitions to low power: %d ", numberGPSForcedTransitions);
    memset(fatfsString, 0, sizeof(fatfsString));
    (void)snprintf(fatfsString, sizeof(fatfsString), "Number of forced GPS transitions to low power: %d ", numberGPSForcedTransitions);
    fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString)));
    fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));

    NRF_LOG_INFO("Ratio for GPS - full power / low power: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(powerRatioGPS));
    memset(fatfsString, 0, sizeof(fatfsString));
    (void)snprintf(fatfsString, sizeof(fatfsString), "Ratio for GPS full power / low power: %.2f ", powerRatioGPS);
    fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString)));
    fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));

    NRF_LOG_INFO("Log end ");
    fatfs_append_line("log.txt", "Log end ", sizeof("Log end "));
    fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));
    fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));

    NRF_LOG_FLUSH();*/

    sleep(); // sleep, this part will need more advanced handle when advanced functions will emerge (like, GPS tracking - to finish the track)
  }
}

static bool endOfInit = 0;

void miscellaneous_init(void) { // function for gathering miscellaneous instances that needs to be executed once at startap
  ticksTimeoutPrevious = app_timer_cnt_get();

  // timeoutMillis = millis();     // the timestamp for sleep if nothing happens needs to be updated at the first start (wake up)
  fatfs_create_file("log.txt");

  FILINFO fno;
  ret_code_t ff_result = f_stat("log.txt", &fno);
  if (ff_result != FR_OK) {
    NRF_LOG_ERROR("Unable to get info about the file: %u\r\n", ff_result);
  }
  fileSize = fno.fsize;
  // fatfs_uninit();
  // fatfs_init();
  nrf_delay_ms(50);

  fatfs_append_line("log.txt", "\n\nLog start\n", sizeof("\n\nLog start\n"));
  memset(fatfsString, 0, sizeof(fatfsString));
  (void)snprintf(fatfsString, sizeof(fatfsString), "Voltage: %.2f V\n", (adc_sample()));
  fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString)));
}

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(3) // delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks)

static uint32_t buttonMillis = 0;
static uint32_t buttonDeltaT = 0;
static uint32_t longPressT = 1500;       // criterion for detecting a long button press in ms
static uint32_t accidentalPressT = 2500; // button press is considered accidental if time exeeds this value in ms
static uint8_t buttonState;
static bool buttonCheckFlag = 0; // this flag is to not uninit led when the interrupt comes when button_check() is running

static uint32_t ticksButtonNow = 0;
static uint32_t ticksButtonStart = 0;
static float buttonPressTime = 0;

static void button_check(void) {
  if (buttonState == 1) {
    ticksButtonNow = app_timer_cnt_get();
    buttonPressTime = app_timer_cnt_diff_compute(ticksButtonNow, ticksButtonStart) / 16.384;
    if (buttonPressTime > longPressT) {
      if (!buttonCheckFlag) { // execute once
        snd_play(3750, 200);
        // NRF_LOG_INFO("led_pwm_uninit() @ button_check");
        led_pwm_uninit();    // turns off LED
        buttonCheckFlag = 1; // cock the flag
      }
    }
  }
}

static void button_event_handler(uint8_t pin_no, uint8_t button_action) {
  // NRF_LOG_INFO("button_event_handler()");
  ret_code_t err_code;
  buttonState = button_action;
  switch (button_action) { // 0 - button released; 1 - button pressed.
  case 0: {                // case button has been released
    if (!buttonCheckFlag) {
      snd_pwm_uninit(); // stop the button sound as quick as release occurs
    }
    if (buttonPressTime < longPressT) { // if it was a short press
      led_pwm_update_duty_cycle(MEDIUM);
      timeoutPermit = 0;                             // signal that user's action took place - delay energy-saving sleep
    } else if (buttonPressTime > accidentalPressT) { // if it was too long, accidental press
      led_pwm_init();
    } else {                // if it was an intentional long press
      while (snd_check()) { // wait while the ending tone is still playing
        snd_check();
      }
      sleep();
    }
  } break;
  case 1: { // case button has been pressed
    led_pwm_update_duty_cycle(BRIGHT);
    ticksButtonStart = app_timer_cnt_get(); // sample the start time for button press
    buttonCheckFlag = 0;
    snd_play(3750, 250);
    if (endOfInit == 0) {
      // while (snd_check() || nrf_gpio_pin_read(BUTTON)) { // wait while the tone is still playing
      //   button_check();
      //   snd_check();
      // }
    }
  } break;
  }
}

static void button_init(void) {
  ret_code_t err_code;

  static app_button_cfg_t buttons[] = // the array must be static because a pointer to it will be saved in the button handler module
      {{BUTTON, APP_BUTTON_ACTIVE_HIGH, NRF_GPIO_PIN_NOPULL, button_event_handler}};

  err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
      BUTTON_DETECTION_DELAY);

  APP_ERROR_CHECK(err_code);

  // nrf_drv_clock_lfclk_request(NULL);
  // err_code = nrf_sdh_enable_request(); // enabling softdevice cause app_timer uses rtc clock...
  // APP_ERROR_CHECK(err_code);

  err_code = app_button_enable();
  APP_ERROR_CHECK(err_code);
}

#define battVoltArraySize 200
static float battVoltArray[battVoltArraySize] = {0.0f};
static uint32_t battVoltArrayIndex = 0;

uint32_t batt_status(void) { // returns charge ststus in %
  if (battVoltArrayIndex < battVoltArraySize) {
    battVoltArray[battVoltArrayIndex] = adc_sample();
    battVoltArrayIndex++;
  } else {
    battVoltArrayIndex = 0;
    battVoltArray[battVoltArrayIndex] = adc_sample();
  }

  float battVoltArraySum = 0.0f;
  for (int i = 0; i < battVoltArraySize; i++) {
    battVoltArraySum = battVoltArraySum + battVoltArray[i];
  }

  float battVolt = battVoltArraySum / battVoltArraySize;

  // float battVolt = adc_sample();

  if (battVolt > 4.177)
    return 100;
  else if (battVolt <= 4.177 && battVolt > 4.171)
    return 99;
  else if (battVolt <= 4.171 && battVolt > 4.164)
    return 98;
  else if (battVolt <= 4.164 && battVolt > 4.157)
    return 97;
  else if (battVolt <= 4.157 && battVolt > 4.150)
    return 96;
  else if (battVolt <= 4.150 && battVolt > 4.144)
    return 95;
  else if (battVolt <= 4.144 && battVolt > 4.137)
    return 94;
  else if (battVolt <= 4.137 && battVolt > 4.130)
    return 93;
  else if (battVolt <= 4.130 && battVolt > 4.123)
    return 92;
  else if (battVolt <= 4.123 && battVolt > 4.116)
    return 91;
  else if (battVolt <= 4.116 && battVolt > 4.109)
    return 90;
  else if (battVolt <= 4.109 && battVolt > 4.103)
    return 89;
  else if (battVolt <= 4.096 && battVolt > 4.089)
    return 87;
  else if (battVolt <= 4.089 && battVolt > 4.082)
    return 86;
  else if (battVolt <= 4.082 && battVolt > 4.076)
    return 85;
  else if (battVolt <= 4.076 && battVolt > 4.069)
    return 84;
  else if (battVolt <= 4.069 && battVolt > 4.062)
    return 83;
  else if (battVolt <= 4.062 && battVolt > 4.056)
    return 82;
  else if (battVolt <= 4.056 && battVolt > 4.049)
    return 81;
  else if (battVolt <= 4.049 && battVolt > 4.042)
    return 80;
  else if (battVolt <= 4.042 && battVolt > 4.036)
    return 79;
  else if (battVolt <= 4.036 && battVolt > 4.029)
    return 78;
  else if (battVolt <= 4.029 && battVolt > 4.022)
    return 77;
  else if (battVolt <= 4.022 && battVolt > 4.016)
    return 76;
  else if (battVolt <= 4.016 && battVolt > 4.009)
    return 75;
  else if (battVolt <= 4.009 && battVolt > 4.002)
    return 74;
  else if (battVolt <= 4.002 && battVolt > 3.996)
    return 73;
  else if (battVolt <= 3.996 && battVolt > 3.989)
    return 72;
  else if (battVolt <= 3.989 && battVolt > 3.983)
    return 71;
  else if (battVolt <= 3.983 && battVolt > 3.976)
    return 70;
  else if (battVolt <= 3.976 && battVolt > 3.969)
    return 69;
  else if (battVolt <= 3.969 && battVolt > 3.963)
    return 68;
  else if (battVolt <= 3.963 && battVolt > 3.956)
    return 67;
  else if (battVolt <= 3.956 && battVolt > 3.950)
    return 66;
  else if (battVolt <= 3.950 && battVolt > 3.943)
    return 65;
  else if (battVolt <= 3.943 && battVolt > 3.937)
    return 64;
  else if (battVolt <= 3.937 && battVolt > 3.930)
    return 63;
  else if (battVolt <= 3.930 && battVolt > 3.924)
    return 62;
  else if (battVolt <= 3.924 && battVolt > 3.917)
    return 61;
  else if (battVolt <= 3.917 && battVolt > 3.911)
    return 60;
  else if (battVolt <= 3.911 && battVolt > 3.904)
    return 59;
  else if (battVolt <= 3.904 && battVolt > 3.898)
    return 58;
  else if (battVolt <= 3.898 && battVolt > 3.892)
    return 57;
  else if (battVolt <= 3.892 && battVolt > 3.885)
    return 56;
  else if (battVolt <= 3.885 && battVolt > 3.879)
    return 55;
  else if (battVolt <= 3.879 && battVolt > 3.872)
    return 54;
  else if (battVolt <= 3.872 && battVolt > 3.866)
    return 53;
  else if (battVolt <= 3.866 && battVolt > 3.860)
    return 52;
  else if (battVolt <= 3.860 && battVolt > 3.853)
    return 51;
  else if (battVolt <= 3.853 && battVolt > 3.847)
    return 50;
  else if (battVolt <= 3.847 && battVolt > 3.841)
    return 49;
  else if (battVolt <= 3.841 && battVolt > 3.834)
    return 48;
  else if (battVolt <= 3.834 && battVolt > 3.828)
    return 47;
  else if (battVolt <= 3.828 && battVolt > 3.822)
    return 46;
  else if (battVolt <= 3.822 && battVolt > 3.815)
    return 45;
  else if (battVolt <= 3.815 && battVolt > 3.809)
    return 44;
  else if (battVolt <= 3.809 && battVolt > 3.803)
    return 43;
  else if (battVolt <= 3.803 && battVolt > 3.796)
    return 42;
  else if (battVolt <= 3.796 && battVolt > 3.790)
    return 41;
  else if (battVolt <= 3.790 && battVolt > 3.784)
    return 40;
  else if (battVolt <= 3.784 && battVolt > 3.778)
    return 39;
  else if (battVolt <= 3.778 && battVolt > 3.771)
    return 38;
  else if (battVolt <= 3.771 && battVolt > 3.765)
    return 37;
  else if (battVolt <= 3.765 && battVolt > 3.759)
    return 36;
  else if (battVolt <= 3.759 && battVolt > 3.753)
    return 35;
  else if (battVolt <= 3.753 && battVolt > 3.747)
    return 34;
  else if (battVolt <= 3.747 && battVolt > 3.740)
    return 33;
  else if (battVolt <= 3.740 && battVolt > 3.734)
    return 32;
  else if (battVolt <= 3.734 && battVolt > 3.728)
    return 31;
  else if (battVolt <= 3.728 && battVolt > 3.722)
    return 30;
  else if (battVolt <= 3.722 && battVolt > 3.716)
    return 29;
  else if (battVolt <= 3.716 && battVolt > 3.710)
    return 28;
  else if (battVolt <= 3.710 && battVolt > 3.704)
    return 27;
  else if (battVolt <= 3.704 && battVolt > 3.697)
    return 26;
  else if (battVolt <= 3.697 && battVolt > 3.691)
    return 25;
  else if (battVolt <= 3.691 && battVolt > 3.685)
    return 24;
  else if (battVolt <= 3.685 && battVolt > 3.679)
    return 23;
  else if (battVolt <= 3.679 && battVolt > 3.673)
    return 22;
  else if (battVolt <= 3.673 && battVolt > 3.667)
    return 21;
  else if (battVolt <= 3.667 && battVolt > 3.661)
    return 20;
  else if (battVolt <= 3.661 && battVolt > 3.655)
    return 19;
  else if (battVolt <= 3.655 && battVolt > 3.649)
    return 18;
  else if (battVolt <= 3.649 && battVolt > 3.643)
    return 17;
  else if (battVolt <= 3.643 && battVolt > 3.637)
    return 16;
  else if (battVolt <= 3.637 && battVolt > 3.631)
    return 15;
  else if (battVolt <= 3.631 && battVolt > 3.625)
    return 14;
  else if (battVolt <= 3.625 && battVolt > 3.619)
    return 13;
  else if (battVolt <= 3.619 && battVolt > 3.613)
    return 12;
  else if (battVolt <= 3.613 && battVolt > 3.607)
    return 11;
  else if (battVolt <= 3.607 && battVolt > 3.601)
    return 10;
  else if (battVolt <= 3.601 && battVolt > 3.595)
    return 9;
  else if (battVolt <= 3.595 && battVolt > 3.589)
    return 8;
  else if (battVolt <= 3.589 && battVolt > 3.583)
    return 7;
  else if (battVolt <= 3.583 && battVolt > 3.577)
    return 6;
  else if (battVolt <= 3.577 && battVolt > 3.571)
    return 5;
  else if (battVolt <= 3.571 && battVolt > 3.566)
    return 4;
  else if (battVolt <= 3.566 && battVolt > 3.560)
    return 3;
  else if (battVolt <= 3.560 && battVolt > 3.554)
    return 2;
  else if (battVolt <= 3.554 && battVolt > 4.548)
    return 1;
  else if (battVolt <= 3.542)
    return 0;
}

//------------------------------------------------------------------------------------
// Variometer part
//------------------------------------------------------------------------------------

// Low-pass filter №1 for kalman loop time prediction
float lpfBeta_1 = 0.99f; // 0<ß<1 // 0.8

float lpf_1(float lpfInput_1) { // low-pass filter for Kalman filter start
  // LPF: Y(n) = (1-ß)*Y(n-1) + (ß*X(n))) = Y(n-1) - (ß*(Y(n-1)-X(n)));
  float lpfResult_1 = lpfResult_1 - (lpfBeta_1 * (lpfResult_1 - lpfInput_1));
  return lpfResult_1;
}

// Low-pass filter №2 for altitude
float lpfBeta_2 = 0.5f; // 0<ß<1 // 0.6

float lpf_2(float lpfInput_2) { // low-pass filter for Kalman vertical velocity
  // LPF: Y(n) = (1-ß)*Y(n-1) + (ß*X(n))) = Y(n-1) - (ß*(Y(n-1)-X(n)));
  float lpfResult_2 = lpfResult_2 - (lpfBeta_2 * (lpfResult_2 - lpfInput_2));
  return lpfResult_2;
}

// Kalman filter №1 for altitude and vertical velocity
static uint32_t kfMillis_1, kfMillisPrevious_1; // time counter for Kalman filter in ms
static float kfDeltaT_1;                        // time interval between Kalman filter iterations in ms
float kfElapsedTimeSecs_1;                      // time interval between Kalman filter iterations in s
float kZ_1, kV_1;                               // kalman filter output, namely, vertical position in cm and vertical speed in cm/s

// Kalman filter state being tracked
float z_1_;     // position
float v_1_;     // velocity
float aBias_1_; // acceleration

// Kalman filter 3x3 State Covariance matrix
float Pzz_1_;
float Pzv_1_;
float Pza_1_;
float Pvz_1_;
float Pvv_1_;
float Pva_1_;
float Paz_1_;
float Pav_1_;
float Paa_1_;

float zAccelBiasVariance_1_; // assumed fixed
float zAccelVariance_1_;     // dynamic acceleration variance
float zVariance_1_;          // z measurement noise variance fixed

#define Z_VARIANCE_1 1000.0f
#define ZACCEL_VARIANCE_1 1.0f
#define ZACCELBIAS_VARIANCE_1 0.01f

#define CLAMP(x, mn, mx) \
  {                      \
    if (x <= (mn))       \
      x = (mn);          \
    else if (x >= (mx))  \
      x = (mx);          \
  }

float configure_kalman_1(float zVariance_1, float zAccelVariance_1, float zAccelBiasVariance_1,
    float zInitial_1, float vInitial_1, float aBiasInitial_1) {

  zAccelVariance_1_ = zAccelVariance_1;
  zAccelBiasVariance_1_ = zAccelBiasVariance_1;
  zVariance_1_ = zVariance_1;

  z_1_ = zInitial_1;
  v_1_ = vInitial_1;
  aBias_1_ = aBiasInitial_1;
  Pzz_1_ = 1.0f;
  Pzv_1_ = 0.0f;
  Pza_1_ = 0.0f;

  Pvz_1_ = 0.0f;
  Pvv_1_ = 1.0f;
  Pva_1_ = 0.0f;

  Paz_1_ = 0.0f;
  Pav_1_ = 0.0f;
  Paa_1_ = 1.0f; // 100000.0f;
}

static uint32_t ticksKalmanNow = 0;
static uint32_t ticksKalmanPrevious = 0;

static uint32_t kfMillisNow, kfMillisPrevious = 0;

// Updates state given a sensor measurement of z, acceleration a,
// and the time in seconds dt since the last measurement.
// 19uS on Navspark @81.84MHz
void update_kalman_1(float z_1, float a_1, float pZ_1, float pV_1) {
  // Timing calculation
  ticksKalmanNow = app_timer_cnt_get();
  kfDeltaT_1 = app_timer_cnt_diff_compute(ticksKalmanNow, ticksKalmanPrevious) / 16.384;
  ticksKalmanPrevious = ticksKalmanNow;
  kfElapsedTimeSecs_1 = kfDeltaT_1 / 1000.0f;
  // NRF_LOG_INFO("kfDeltaT_1 = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(kfDeltaT_1));
  // NRF_LOG_FLUSH();

  /*kfMillisNow = millisCount;
  kfDeltaT_1 = kfMillisNow - kfMillisPrevious;
  kfMillisPrevious = kfMillisNow;*/

  kfElapsedTimeSecs_1 = kfDeltaT_1 / 1000.0f;

  // NRF_LOG_INFO("kfDeltaT_1 = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(kfDeltaT_1));

  // float dt_1 = lpf_1(kfElapsedTimeSecs_1);
  float dt_1 = kfElapsedTimeSecs_1;

  // Predict state
  float a_1_ = a_1 - aBias_1_;
  v_1_ += a_1_ * dt_1;
  z_1_ += v_1_ * dt_1;
  // NRF_LOG_INFO("a_1 =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(aBias_1_));
  // NRF_LOG_INFO("aBias_1_ =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(aBias_1_));
  // NRF_LOG_INFO("dt_1 =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(dt_1));
  // NRF_LOG_INFO("a_1_ =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(aBias_1_));
  // NRF_LOG_INFO("v_1_ =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(aBias_1_));
  // NRF_LOG_INFO("z_1_ =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(aBias_1_));

  zAccelVariance_1_ = fabs(a_1_) / 50.0f; // not sure if it makes any better
  CLAMP(zAccelVariance_1_, 1.0f, 50.0f);

  // Predict State Covariance matrix
  float t00_1, t01_1, t02_1;
  float t10_1, t11_1, t12_1;
  float t20_1, t21_1, t22_1;

  float dt2div2_1 = dt_1 * dt_1 / 2.0f;
  float dt3div2_1 = dt2div2_1 * dt_1;
  float dt4div4_1 = dt2div2_1 * dt2div2_1;

  t00_1 = Pzz_1_ + dt_1 * Pvz_1_ - dt2div2_1 * Paz_1_;
  t01_1 = Pzv_1_ + dt_1 * Pvv_1_ - dt2div2_1 * Pav_1_;
  t02_1 = Pza_1_ + dt_1 * Pva_1_ - dt2div2_1 * Paa_1_;

  t10_1 = Pvz_1_ - dt_1 * Paz_1_;
  t11_1 = Pvv_1_ - dt_1 * Pav_1_;
  t12_1 = Pva_1_ - dt_1 * Paa_1_;

  t20_1 = Paz_1_;
  t21_1 = Pav_1_;
  t22_1 = Paa_1_;

  Pzz_1_ = t00_1 + dt_1 * t01_1 - dt2div2_1 * t02_1;
  Pzv_1_ = t01_1 - dt_1 * t02_1;
  Pza_1_ = t02_1;

  Pvz_1_ = t10_1 + dt_1 * t11_1 - dt2div2_1 * t12_1;
  Pvv_1_ = t11_1 - dt_1 * t12_1;
  Pva_1_ = t12_1;

  Paz_1_ = t20_1 + dt_1 * t21_1 - dt2div2_1 * t22_1;
  Pav_1_ = t21_1 - dt_1 * t22_1;
  Paa_1_ = t22_1;

  Pzz_1_ += dt4div4_1 * zAccelVariance_1_;
  Pzv_1_ += dt3div2_1 * zAccelVariance_1_;

  Pvz_1_ += dt3div2_1 * zAccelVariance_1_;
  Pvv_1_ += dt_1 * dt_1 * zAccelVariance_1_;

  Paa_1_ += zAccelBiasVariance_1_;

  // Kalman error
  float innov_1 = z_1 - z_1_;
  float sInv_1 = 1.0f / (Pzz_1_ + zVariance_1_);

  // Kalman gains
  float kz_1 = Pzz_1_ * sInv_1;
  float kv_1 = Pvz_1_ * sInv_1;
  float ka_1 = Paz_1_ * sInv_1;
  // NRF_LOG_INFO("kz_1 =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(kz_1));
  // NRF_LOG_INFO("kv_1 =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(kv_1));
  // NRF_LOG_INFO("ka_1 =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(ka_1));

  // Update state
  z_1_ += kz_1 * innov_1;
  v_1_ += kv_1 * innov_1;
  aBias_1_ += ka_1 * innov_1;

  kZ_1 = z_1_;
  kV_1 = v_1_;
  // NRF_LOG_INFO("kZ_1 =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(kZ_1));
  // NRF_LOG_INFO("kV_1 =" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(kV_1));
  // NRF_LOG_INFO("--------------------------");

  // Update state covariance matrix
  Paz_1_ -= ka_1 * Pzz_1_;
  Pav_1_ -= ka_1 * Pzv_1_;
  Paa_1_ -= ka_1 * Pza_1_;

  Pvz_1_ -= kv_1 * Pzz_1_;
  Pvv_1_ -= kv_1 * Pzv_1_;
  Pva_1_ -= kv_1 * Pza_1_;

  Pzz_1_ -= kz_1 * Pzz_1_;
  Pzv_1_ -= kz_1 * Pzv_1_;
  Pza_1_ -= kz_1 * Pza_1_;
}

void gps_check(void);

static uint32_t ticksEndOfInit = 0;
static uint32_t endOfInitMillis = 0;
static uint32_t varioLoopMillisPrevious;
static uint32_t imuLoopMillisPrevious;
static uint32_t baroLoopMillisPrevious;

void handle_gps_buffer();

void vario_configure(void) {
  float lpfIMU = 0.0f;
  int i = 0;
  while (i < 5) {
    i++;
    nrf_delay_ms(5);
    ms5637_process();
    ms5637_calc();
    NRF_LOG_INFO("Altitude = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(alt));

    handle_gps_buffer(); // required here to not overflow uart buffer
    button_check();
    snd_check();
  }
  i = 0;
  while (i < 5) {
    i++;
    nrf_delay_ms(5);
    icm20948_process();
    icm20948_calc();
    NRF_LOG_INFO("Acceleration = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(vA * 1000.0f));

    handle_gps_buffer(); // required here to not overflow uart buffer
    button_check();
    snd_check();
  }

  NRF_LOG_INFO("Configuration baro altitude = " NRF_LOG_FLOAT_MARKER " m ", NRF_LOG_FLOAT(alt));
  NRF_LOG_INFO("Configuration acceleration = " NRF_LOG_FLOAT_MARKER " mg ", NRF_LOG_FLOAT(vA * 1000.0f));

  // configure_kalman_1(0.001, 0.001, 0.001, alt * 100.0f, vA * 1000.0f, 0.0f);
  configure_kalman_1(Z_VARIANCE_1, ZACCEL_VARIANCE_1, ZACCELBIAS_VARIANCE_1, alt * 100.0f, vA * 1000.0f, 0.0f);

  i = 0;
  while (i < 100) {
    i++;
    ms5637_process();
    ms5637_calc();
    icm20948_process();
    icm20948_calc();

    alt = lpf_1(alt);
    vA = lpf_1(vA);
    update_kalman_1(alt * 100.0f, vA * 1000.0f, kZ_1, kV_1);

    altitudeKalman = (float)(z_1_ / 100.0f);
    v_1_ = lpf_2(v_1_); // note this is added for smooth Kalman start
    // climbrateKalman = (float)(v_1_ / 100.0f);

    // NRF_LOG_INFO("Altitude = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(alt));
    NRF_LOG_INFO("Acceleration = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(vA * 1000.0f));
    // NRF_LOG_INFO("Kalman climbrate = " NRF_LOG_FLOAT_MARKER " m/s ", NRF_LOG_FLOAT(climbrateKalman));

    handle_gps_buffer(); // required here to not overflow uart buffer
    button_check();
    snd_check();

    nrf_delay_ms(3);
  }

  // configure_kalman_1(Z_VARIANCE_1, ZACCEL_VARIANCE_1, ZACCELBIAS_VARIANCE_1, alt * 100.0f, vA * 1000.0f, 0.0f);

  ticksEndOfInit = app_timer_cnt_get();
  // endOfInitMillis = millis();
  varioLoopMillisPrevious = endOfInitMillis;
  imuLoopMillisPrevious = endOfInitMillis;
  baroLoopMillisPrevious = endOfInitMillis;
  kfMillisPrevious_1 = endOfInitMillis;
  timeGPSFullPowerStart = endOfInitMillis;
  timeGPSLowPowerStart = endOfInitMillis;
}

#define SINK_THRESHOLD_RATE -2.5f     // -m/s
#define CLIMB_THRESHOLD_RATE 0.3f     // m/s
#define CLIMB_THRESHOLD_DISTANCE 0.5f // m

#define CLIMB_BASE_FREQ 400
#define CLIMB_FREQ_COEFF 225
#define SINK_BASE_FREQ 250
#define SINK_FREQ_COEFF 60

static float climbThresholdRate = CLIMB_THRESHOLD_RATE;
static float sinkThresholdRate = SINK_THRESHOLD_RATE;

static int32_t toneVario = 0;
static uint32_t varioPrevMillis = 0;
static float varioElapsedTimeSec = 0;
static float heightVarioDelta = 0;
static float heightVarioPrevious = 0;
static bool thresholdFlag = 0;
static bool beepPatternFlag = 1;
static bool beepStartFlag = 1;
static bool gisterezisFlag = 0;

static uint32_t ticksSilenceNow = 0;
static uint32_t ticksBeepStart = 0;
static uint32_t ticksBeepLast = 0;
static float lastBeepTime = 0.0f;

static bool onceVarioAudioFlag = 0;

static uint32_t millisBeepLast, millisSilenceNow = 0;

static float altitudeKalmanTest = 0;

static float beepDuration = 0.0f;

void vario_audio(void) {
  /*climbrateKalman = 1; // unit test
  altitudeKalman = altitudeKalmanTest + 5 / 200.0f;
  altitudeKalmanTest = altitudeKalman;*/

  if (climbrateKalman >= climbThresholdRate || climbrateKalman <= sinkThresholdRate) { // climbing / sinking rate threshold
    thresholdFlag = 1;
    // NRF_LOG_INFO("Kalman height = " NRF_LOG_FLOAT_MARKER " m ", NRF_LOG_FLOAT(altitudeKalman));
    // NRF_LOG_INFO("Delta height = " NRF_LOG_FLOAT_MARKER " m ", NRF_LOG_FLOAT(heightVarioDelta));
    // NRF_LOG_INFO("Kalman velocity = " NRF_LOG_FLOAT_MARKER " m/s ", NRF_LOG_FLOAT(climbrateKalman));
  } else if (thresholdFlag) {
    beepDuration = app_timer_cnt_diff_compute(app_timer_cnt_get(), ticksBeepStart) / 16.384;
    if (beepDuration > 250.0f) { // short artefact beeps are not allowed
      snd_stop();
      thresholdFlag = 0;
      beepPatternFlag = 1;
      beepStartFlag = 1;
      ticksBeepLast = app_timer_cnt_get();
      // millisBeepLast = millisCount;
    }
  }

  if (!thresholdFlag && gisterezisFlag) {
    ticksSilenceNow = app_timer_cnt_get();
    // millisSilenceNow = millisCount;
    lastBeepTime = app_timer_cnt_diff_compute(ticksSilenceNow, ticksBeepLast) / 16.384;
    // lastBeepTime = millisSilenceNow - millisBeepLast;
    if (lastBeepTime > 250.0f) { // don't start beep more often then once in 250 milliseconds
      gisterezisFlag = 0;
    }
  }

  if (thresholdFlag) {
    heightVarioDelta = altitudeKalman - heightVarioPrevious;
  } else {
    heightVarioDelta = 0;
    heightVarioPrevious = altitudeKalman;
  }

  if (heightVarioDelta >= CLIMB_THRESHOLD_DISTANCE && thresholdFlag) {
    heightVarioDelta = 0;
    heightVarioPrevious = altitudeKalman;
    beepPatternFlag = !beepPatternFlag;
    gisterezisFlag = 0;
  }

  if (beepPatternFlag && thresholdFlag) {
    if (climbrateKalman >= climbThresholdRate) {
      toneVario = CLIMB_BASE_FREQ + (float)(climbrateKalman * CLIMB_FREQ_COEFF);
      if (toneVario > 4000) {
        toneVario = 4000; // frequency can't be too high
      }
    } else if (climbrateKalman <= sinkThresholdRate) {
      toneVario = SINK_BASE_FREQ + climbrateKalman * SINK_FREQ_COEFF;
      if (toneVario < 100) {
        toneVario = 100; // frequency can't be null
      }
    }
  }

  // if ((millis() - endOfInitMillis) > 20000 && onceVarioAudioFlag == 0) {
  /*if (((app_timer_cnt_diff_compute(app_timer_cnt_get(), ticksEndOfInit) / 16.384) > 20000) && onceVarioAudioFlag == 0) { // wait for 20 seconds to skip artefact beeps
    snd_play(3750, 200);
    onceVarioAudioFlag = 1;
  }*/

  // NRF_LOG_INFO("beepMuteFlag = %u", beepMuteFlag);

  if (beepMuteFlag) {
      if ((app_timer_cnt_diff_compute(app_timer_cnt_get(), ticksEndOfInit) / 16.384) > 10000) { // to skip artefact beeps after init
        beepMuteFlag = 0;
      }
    }

    if (beepPatternFlag && beepStartFlag && thresholdFlag && gisterezisFlag == 0) {
      if (!beepMuteFlag) { // to skip artefact beeps after init
      beepStartFlag = 0;
      // if ((millis() - endOfInitMillis) > 20000) { // skip artefact beeps after "init" sound
      // if ((app_timer_cnt_diff_compute(app_timer_cnt_get(), ticksEndOfInit) / 16.384) > 5000) { // skip artefact beeps even after init
      snd_start(toneVario);
      gisterezisFlag = 1;
      ticksBeepStart = app_timer_cnt_get();
      }
    } else if (!beepPatternFlag && !beepStartFlag) {
      gisterezisFlag = 0;
      beepStartFlag = 1;
      snd_stop();
      ticksBeepLast = app_timer_cnt_get();
      // millisBeepLast = millisCount;
    }
  }

  static void bluetooth_app_send(void);

  static float varioLoopTime = 0;
  static uint32_t varioLoopMillis = 0;
  static uint32_t varioLoopMillisPrevious = 0;

  static uint32_t imuLoopTime = 0;
  static uint32_t imuLoopMillis = 0;
  static uint32_t imuLoopMillisPrevious = 0;

  static uint32_t baroLoopTime = 0;
  static uint32_t baroLoopMillis = 0;
  static uint32_t baroLoopMillisPrevious = 0;

  static float imuAccelSum = 0.0f;
  static float imuAccelAverage = 0.0f;
  static uint32_t imuAccelCount = 1; // actually, should be 0; but avoid division by 0 at imuAccelSum / imuAccelCount

  static float altLowPass = 0.0f;

  static uint32_t ticksVarioNow = 0;
  static uint32_t ticksVarioPrevious = 0;

  static float vAprevious = 0.0f;
  static uint32_t varioLoopCount = 0;

  static float pressureBaroArray[] = {0.0f};
  static float pressureBaroAverage = 0.0f;

  static void vario_check(void) {
    ticksVarioNow = app_timer_cnt_get();
    varioLoopTime = app_timer_cnt_diff_compute(ticksVarioNow, ticksVarioPrevious) / 16.384;
    if (varioLoopTime > 5.5f) {
      // if (varioLoopFlag == 1) {
      //   varioLoopFlag = 0;
      ticksVarioNow = app_timer_cnt_get();
      varioLoopTime = app_timer_cnt_diff_compute(ticksVarioNow, ticksVarioPrevious) / 16.384;

      ticksVarioPrevious = ticksVarioNow;
      ms5637_process();
      ms5637_calc();
      icm20948_process();
      icm20948_calc();

      // float altLowPass = lpf_2(alt);
      // update_kalman_1(altLowPass * 100.0f, vA * 500.0f, kZ_1, kV_1); // fast responce kalman filter
      alt = lpf_1(alt);
      vA = lpf_1(vA);
      update_kalman_1(alt * 100.0f, vA * 500.0f, kZ_1, kV_1); // fast responce kalman filter

      altitudeKalman = (float)(z_1_ / 100.0f);
      climbrateKalman = (float)(v_1_ / 100.0f);
      climbrateKalman = lpf_2(climbrateKalman);

      // NRF_LOG_INFO("varioLoopTime = " NRF_LOG_FLOAT_MARKER " ms ", NRF_LOG_FLOAT(varioLoopTime));

      // NRF_LOG_INFO("Baro altitude = " NRF_LOG_FLOAT_MARKER " m ", NRF_LOG_FLOAT(altLowPass));
      // NRF_LOG_INFO("Acceleration = " NRF_LOG_FLOAT_MARKER " mg ", NRF_LOG_FLOAT(vA * 1000.0f));
      // NRF_LOG_INFO("Kalman altitude = " NRF_LOG_FLOAT_MARKER " m ", NRF_LOG_FLOAT(altitudeKalman));
      // NRF_LOG_INFO("Kalman climbrate = " NRF_LOG_FLOAT_MARKER " m/s ", NRF_LOG_FLOAT(climbrateKalman));

      // if (climbrateKalman > 0.3) {
      //    NRF_LOG_INFO("Kalman climbrate = " NRF_LOG_FLOAT_MARKER " m/s ", NRF_LOG_FLOAT(climbrateKalman));
      // }

      vAprevious = vA;

      vario_audio();
      bluetooth_app_send();

      /*pressureBaroArray[varioLoopCount] = pressureBaro;
      varioLoopCount++;
      if (varioLoopCount > 10) { // do at 10 Hz
        varioLoopCount = 0;

        float pressureBaroSum = 0.0f;
        int i = 0;
        while (i < 10) {
          pressureBaroSum = pressureBaroSum + pressureBaroArray[i];
          i++;
        }
        pressureBaroAverage = pressureBaroSum / 10;
        bluetooth_app_send();
      }*/

      if (climbrateKalman > 0.6f || climbrateKalman < -0.6f) { // enable this check after test
        timeoutPermit = 0;
      }
    }
  }

  //------------------------------------------------------------------------------------
  // UART-GPS
  //------------------------------------------------------------------------------------

  bool gpsOk = 0; // GPS health flag

  bool checkBufferFull = false;
  uint8_t tempBuffer[2000];
  uint8_t dataGPS[500];

  uint32_t tempDataLength = 0;
  uint32_t tempDataLengthNew = 0;
  uint32_t tempDataLengthPrevious = 0;

  uint8_t gpggaNMEA[500];
  uint8_t gpgsaNMEA[500];
  uint8_t gpgsvNMEA[500];
  uint8_t gprmcNMEA[500];

  char gpggaArray[25][25];
  char gpgsaArray[25][25];
  char gpgsvArray[25][25];
  char gprmcArray[25][25];

  float altitudeGPS = 0;
  float velocityGPS = 0;
  float azimuthGPS = 0;

  NRF_LIBUARTE_ASYNC_DEFINE(libuarte, 0, 2, NRF_LIBUARTE_PERIPHERAL_NOT_USED, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 255, 5);

  void extract_gps(void) { // NMEA parsing
    char S[sizeof(dataGPS)] = {0};
    snprintf(S, sizeof(S), "%s", dataGPS);

    int i = 0;
    char *p = strtok(S, "\r\n");
    char *extractNMEA[sizeof(dataGPS)];

    while (p != NULL) {
      extractNMEA[i++] = p;
      p = strtok(NULL, "\r\n");
    }

    for (int ii = 0; ii < i; ++ii) {
      if ((strncmp(extractNMEA[ii], "$GP", 3) == 0) | (strncmp(extractNMEA[ii], "$GN", 3) == 0)) {
        if (strncmp(&extractNMEA[ii][3], "GGA", 3) == 0) {
          memcpy(gpggaNMEA, extractNMEA[ii], strnlen(extractNMEA[ii], sizeof(gpggaNMEA) - 1));
        }
        if (strncmp(&extractNMEA[ii][3], "GSA", 3) == 0) {
          memcpy(gpgsaNMEA, extractNMEA[ii], strnlen(extractNMEA[ii], sizeof(gpgsaNMEA) - 1));
        }
        if (strncmp(&extractNMEA[ii][3], "GSV", 3) == 0) {
          memcpy(gpgsvNMEA, extractNMEA[ii], strnlen(extractNMEA[ii], sizeof(gpgsvNMEA) - 1));
        }
        if (strncmp(&extractNMEA[ii][3], "RMC", 3) == 0) {
          memcpy(gprmcNMEA, extractNMEA[ii], strnlen(extractNMEA[ii], sizeof(gprmcNMEA) - 1));
        }
      }
    }

    uint32_t gpggaLength = strnlen(gpggaNMEA, sizeof(gpggaNMEA));
    if (gpggaLength > 0) {
      char S[sizeof(gpggaNMEA)] = {0};
      snprintf(S, sizeof(S), "%s", gpggaNMEA);

      // NRF_LOG_INFO("%s", gpggaNMEA);
      // NRF_LOG_FLUSH();

      char delim[1] = ",";
      char *str, *istr;
      str = strdup(S);
      int i = 0;
      while ((istr = strsep(&str, ",")) != NULL) {
        // NRF_LOG_INFO("%s", istr);
        // NRF_LOG_FLUSH();

        strncpy(gpggaArray[i], istr, 24);
        i++;
      }

      // fatfs_append_line("log.txt", gpggaNMEA, gpggaLength);
      // fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));

      altitudeGPS = atof(gpggaArray[9]); // extract altitude
    }

    uint32_t gpgsaLength = strnlen(gpgsaNMEA, sizeof(gpgsaNMEA));
    if (gpgsaLength > 0) {
      char S[sizeof(gpgsaNMEA)] = {0};
      snprintf(S, sizeof(S), "%s", gpgsaNMEA);

      // NRF_LOG_INFO("%s", gpgsaNMEA);
      // NRF_LOG_FLUSH();

      char delim[1] = ",";
      char *str, *istr;
      str = strdup(S);
      int i = 0;
      while ((istr = strsep(&str, ",")) != NULL) {
        // NRF_LOG_INFO("%s", istr);
        // NRF_LOG_FLUSH();

        strncpy(gpgsaArray[i], istr, 24);
        i++;
      }

      // fatfs_append_line("log.txt", gpgsaNMEA, gpgsaLength);
      // fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));
    }

    uint32_t gpgsvLength = strnlen(gpgsvNMEA, sizeof(gpgsvNMEA));
    if (gpgsvLength > 0) {
      char S[sizeof(gpgsvNMEA)] = {0};
      snprintf(S, sizeof(S), "%s", gpgsvNMEA);

      // NRF_LOG_INFO("%s", gpgsvNMEA);
      // NRF_LOG_FLUSH();

      char delim[1] = ",";
      char *str, *istr;
      str = strdup(S);
      int i = 0;
      while ((istr = strsep(&str, ",")) != NULL) {
        // NRF_LOG_INFO("%s", istr);
        // NRF_LOG_FLUSH();

        strncpy(gpgsvArray[i], istr, 24);
        i++;
      }

      // fatfs_append_line("log.txt", gpgsvNMEA, gpgsvLength);
      // fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));

      azimuthGPS = atof(gpgsvArray[6]); // extract azimuth
    }

    uint32_t gprmcLength = strnlen(gprmcNMEA, sizeof(gprmcNMEA));
    if (gprmcLength > 0) {
      char S[sizeof(gprmcNMEA)] = {0};
      snprintf(S, sizeof(S), "%s", gprmcNMEA);

      // NRF_LOG_INFO("%s", gprmcNMEA);
      // NRF_LOG_FLUSH();

      char delim[1] = ",";
      char *str, *istr;
      str = strdup(S);
      int i = 0;
      while ((istr = strsep(&str, ",")) != NULL) {
        // NRF_LOG_INFO("%s", istr);
        // NRF_LOG_FLUSH();

        strncpy(gprmcArray[i], istr, 24);
        i++;
      }

      // fatfs_append_line("log.txt", gprmcNMEA, gprmcLength);
      // fatfs_append_line("log.txt", "\r\n", sizeof("\r\n"));

      float x = atof(gprmcArray[7]);      // extract speed over ground in knots
      velocityGPS = x * 0.51444f;         // convert knots to m/s
      if ((velocityGPS * 3.6f) > 15.0f) { // konvert m/s to km/h
        timeoutPermit = 0;
      }
    }

    memset(gpggaNMEA, 0, sizeof(gpggaNMEA));
    memset(gpgsaNMEA, 0, sizeof(gpggaNMEA));
    memset(gpgsvNMEA, 0, sizeof(gpggaNMEA));
    memset(gprmcNMEA, 0, sizeof(gprmcNMEA));
  }

  void handle_gps_buffer() {
    tempDataLength = tempDataLengthNew - tempDataLengthPrevious;
    tempDataLengthPrevious = tempDataLengthNew;
    if (tempDataLengthNew > 4000000000) { // avoid uint32_t overflow
      tempDataLengthNew = 0;
      tempDataLengthPrevious = 0;
    }

    if (checkBufferFull == false) {
      strncat(dataGPS, tempBuffer, tempDataLength);
    } else {
      char S[sizeof(dataGPS)] = {0};
      snprintf(S, sizeof(dataGPS), "%s", dataGPS);
      NRF_LOG_INFO("%s", S); // NMEA strings output <---------------------------------------------------------- outputps NMEA GPS data
      // NRF_LOG_HEXDUMP_INFO(dataGPS, 100); // raw hex output
      NRF_LOG_FLUSH();
      extract_gps();
      strncat(dataGPS, tempBuffer, tempDataLength);
      memset(dataGPS, 0, sizeof(dataGPS));
      checkBufferFull = false;
    }

    /*if (tempDataLength > 0) {
    NRF_LOG_INFO("tempDataLength: %d", tempDataLength);
    NRF_LOG_INFO("tempBuffer:");
    NRF_LOG_HEXDUMP_INFO(tempBuffer, 50); // raw hex output
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("dataGPS:");
    NRF_LOG_HEXDUMP_INFO(dataGPS, 50); // raw hex output
    NRF_LOG_FLUSH();
    }*/

    memset(tempBuffer, 0, sizeof(tempBuffer));
  }

  void uart_event_handler(void *context, nrf_libuarte_async_evt_t *p_evt) {
    nrf_libuarte_async_t *p_libuarte = (nrf_libuarte_async_t *)context;
    ret_code_t ret;

    switch (p_evt->type) {
    case NRF_LIBUARTE_ASYNC_EVT_ERROR:
      // NRF_LOG_INFO("UART ERR");
      break;
    case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
      // NRF_LOG_INFO("UART RX");
      gpsOk = 1;

      strncat(tempBuffer, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
      tempDataLengthNew = tempDataLengthNew + p_evt->data.rxtx.length;
      if (p_libuarte->p_ctrl_blk->sub_rx_count == 0) {
        checkBufferFull = true;
      }
      nrf_libuarte_async_rx_free(p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
      break;
    case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
      // NRF_LOG_INFO("UART TX");
      break;
    default:
      break;
    }
  }

  void uart_init(void) {
    nrf_libuarte_async_config_t nrf_libuarte_async_config_115200 = {
        .tx_pin = TX_PIN_NUMBER,
        .rx_pin = RX_PIN_NUMBER,
        .baudrate = NRF_UARTE_BAUDRATE_115200,
        .parity = NRF_UARTE_PARITY_EXCLUDED,
        .hwfc = NRF_UARTE_HWFC_DISABLED,
        .timeout_us = 100,
        .int_prio = APP_IRQ_PRIORITY_LOW_MID};
    ret_code_t err_code = nrf_libuarte_async_init(&libuarte, &nrf_libuarte_async_config_115200, uart_event_handler, (void *)&libuarte); // switch UART to 115200 baud
    APP_ERROR_CHECK(err_code);
    nrf_libuarte_async_enable(&libuarte);
  }

  void uart_uninit() {
    nrf_libuarte_async_uninit(&libuarte);
    // APP_ERROR_CHECK(err_code);
  }

  static bool turnOnGpsFlag = 0;
  static uint32_t turnOnGpsMillis, turnOnGpsMillisPrevious = 0;
  static uint32_t turnOnGpsTime = 0;

  static uint32_t ticksTurnOnGpsPrevious = 0;

  void turn_on_GPS(void) {
    nrf_gpio_pin_set(39); // try to turn on here and check it in the gps_check() function
    nrf_delay_ms(3);
    nrf_gpio_pin_clear(39);

    turnOnGpsFlag = 1;

    ticksTurnOnGpsPrevious = app_timer_cnt_get();
    // turnOnGpsMillisPrevious = millis();
  }

  static bool turnOffGpsFlag = 0;

  void turn_off_GPS(void) {
    turnOffGpsFlag = 1;

    static uint8_t commandMID100[] = "$PSRF117,16*0B\r\n"; // is "turn off" command
    static uint8_t command_size = sizeof(commandMID100);
    ret_code_t err_code = nrf_libuarte_async_tx(&libuarte, commandMID100, command_size);
    while (err_code > 0) {
      handle_gps_buffer();                                                      // empty the buffer before turning off GPS
      err_code = nrf_libuarte_async_tx(&libuarte, commandMID100, command_size); // try to turn off here and check it in the gps_check() function
    }
    APP_ERROR_CHECK(err_code);
  }

  static uint32_t ticksGpsFullPowerStart = 0;

  void gps_init(void) {
    nrf_libuarte_async_config_t nrf_libuarte_async_config_4800 = {
        .tx_pin = TX_PIN_NUMBER,
        .rx_pin = RX_PIN_NUMBER,
        .baudrate = NRF_UARTE_BAUDRATE_4800,
        .parity = NRF_UARTE_PARITY_EXCLUDED,
        .hwfc = NRF_UARTE_HWFC_DISABLED,
        .timeout_us = 100,
        .int_prio = APP_IRQ_PRIORITY_LOW_MID};
    ret_code_t err_code = nrf_libuarte_async_init(&libuarte, &nrf_libuarte_async_config_4800, uart_event_handler, (void *)&libuarte);
    APP_ERROR_CHECK(err_code);
    nrf_libuarte_async_enable(&libuarte);

    nrf_gpio_cfg_input(37, NRF_GPIO_PIN_PULLDOWN); // for UART interface selection
    nrf_gpio_cfg_output(43);
    nrf_gpio_pin_set(43);
    nrf_gpio_cfg_input(41, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_output(39);
    nrf_gpio_pin_clear(39);
    nrf_delay_ms(250);
    nrf_gpio_pin_set(39);
    nrf_delay_ms(3);
    nrf_gpio_pin_clear(39);

    while (gpsOk < 1) {
      // wait
      ticksGpsFullPowerStart = app_timer_cnt_get();
      // timeGPSFullPowerStart = millis();
    }

    // NMEA message MID100 to switch GPS reciever to OSP commands with UART baudrate 115200, for checksum visit https://nmeachecksum.eqth.net/
    // static uint8_t commandMID100[] = "$PSRF100,0,4800,8,1,0*0F\r\n";
    // static uint8_t commandMID100[] = "$PSRF100,0,57600,8,1,0*37\r\n";
    static uint8_t commandMID100[] = "$PSRF100,0,115200,8,1,0*04\r\n";
    static uint8_t command_size = sizeof(commandMID100);
    err_code = nrf_libuarte_async_tx(&libuarte, commandMID100, command_size);
    while (err_code > 0) {
      err_code = nrf_libuarte_async_tx(&libuarte, commandMID100, command_size);
    }
    APP_ERROR_CHECK(err_code);

    while (nrf_gpio_pin_read(37) > 0) { // turn off GPS
      nrf_gpio_pin_set(39);
      nrf_delay_ms(3);
      nrf_gpio_pin_clear(39);
      nrf_delay_ms(500);
    }

    nrf_libuarte_async_uninit(&libuarte);
    nrf_libuarte_async_config_t nrf_libuarte_async_config_115200 = {
        .tx_pin = TX_PIN_NUMBER,
        .rx_pin = RX_PIN_NUMBER,
        .baudrate = NRF_UARTE_BAUDRATE_115200,
        .parity = NRF_UARTE_PARITY_EXCLUDED,
        .hwfc = NRF_UARTE_HWFC_DISABLED,
        .timeout_us = 100,
        .int_prio = APP_IRQ_PRIORITY_LOW_MID};
    err_code = nrf_libuarte_async_init(&libuarte, &nrf_libuarte_async_config_115200, uart_event_handler, (void *)&libuarte); // switch UART to 115200 baud
    APP_ERROR_CHECK(err_code);
    nrf_libuarte_async_enable(&libuarte);

    while (nrf_gpio_pin_read(37) < 1) { // turn on GPS
      nrf_gpio_pin_set(39);
      nrf_delay_ms(3);
      nrf_gpio_pin_clear(39);
      nrf_delay_ms(500);
    }

    // Send Session Closing Request. See OSP message MID 213, SID 2 from OriginGps OSP Manual
    /*uint8_t commandMID213SID2[] = {0xA0, 0xA2, 0x00, 0x03, 0xD5, 0x02, 0x00, 0x00, 0xD7, 0xB0, 0xB3, 0x0D, 0x0A}; // session closing request
    command_size = sizeof(commandMID213SID2);
    err_code = nrf_libuarte_async_tx(&libuarte, commandMID213SID2, command_size);
    uint8_t ii = 0;
    while (err_code > 0 || ii < 10) {
      err_code = nrf_libuarte_async_tx(&libuarte, commandMID213SID2, command_size);
      ii++;
    }
    APP_ERROR_CHECK(err_code);

    // Send Switch Operating Modes command to enter test mode 4. See OSP message MID 150 from OriginGps OSP Manual
    // uint8_t commandMID150[] = {0xA0, 0xA2, 0x00, 0x07, 0x96, 0x1E, 0x54, 0x00, 0x06, 0x00, 0x3C, 0x01, 0x4A, 0xB0, 0xB3, 0x0D, 0x0A}; // Enter test mode 4
    uint8_t commandMID150[] = {0xA0, 0xA2, 0x00, 0x07, 0x96, 0x1E, 0x51, 0x00, 0x06, 0x00, 0x1E, 0x01, 0x29, 0xB0, 0xB3, 0x0D, 0x0A}; // Enter test mode 1
    command_size = sizeof(commandMID150);
    err_code = nrf_libuarte_async_tx(&libuarte, commandMID150, command_size);
    int ii = 0;
    while (err_code > 0 || ii < 10) {
      err_code = nrf_libuarte_async_tx(&libuarte, commandMID150, command_size);
      ii++;
    }
    APP_ERROR_CHECK(err_code);*/

    // Send Initialze Data Source command. See OSP message MID 128 from OriginGps OSP Manual
    // uint8_t commandMID128[] = {0xA0, 0xA2, 0x00, 0x19, 0x80, 0x00, 0x2A, 0x3E, 0x7F, 0x00, 0x18, 0xAC, 0x37, 0x00, 0x53, 0xD9, 0xF7, 0x00, 0x01, 0x24, 0xF8, 0x01, 0x07, 0xAC, 0x00, 0x08, 0xF7, 0x0C, 0x33, 0x07, 0x94, 0xB0, 0xB3, 0x0D, 0x0A}; // initialize data source, the location is St.Petersburg
    uint8_t commandMID128[] = {0xA0, 0xA2, 0x00, 0x19, 0x80, 0x00, 0x2A, 0x3E, 0x7F, 0x00, 0x18, 0xAC, 0x37, 0x00, 0x53, 0xD9, 0xF7, 0x00, 0x01, 0x24, 0xF8, 0x00, 0x83, 0xD6, 0x00, 0x08, 0xF7, 0x0C, 0x86, 0x08, 0x8C, 0xB0, 0xB3, 0x0D, 0x0A}; // initialize data source, full system reset
    // uint8_t commandMID128[] = {0xA0, 0xA2, 0x00, 0x19, 0x80, 0xFF, 0xD7, 0x00, 0xF9, 0xFF, 0xBE, 0x52, 0x66, 0x00, 0x3A, 0xC5, 0x7A, 0x00, 0x01, 0x24, 0xF8, 0x00, 0x83, 0xD6, 0x00, 0x03, 0x9C, 0x0C, 0x33, 0x0A, 0x91, 0xB0, 0xB3, 0x0D, 0x0A}; // initialize data source, the location is San Jose

    command_size = sizeof(commandMID128);
    err_code = nrf_libuarte_async_tx(&libuarte, commandMID128, command_size);
    int ii = 0;
    while (err_code > 0 || ii < 10) {
      err_code = nrf_libuarte_async_tx(&libuarte, commandMID128, command_size);
      ii++;
    }
    APP_ERROR_CHECK(err_code);

    // Below is example of checksum calcuation for OSP messages
    uint8_t i = 0;
    uint32_t checkSum = 0;
    // uint8_t value[] = {0x97, 0x11, 0x11, 0x00, 0xC8, 0x00, 0x00, 0x00, 0xC8}; // MID 151, checksum 0x02 0x49 / PTF on
    // uint8_t value[] = {0x97, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0xC8}; // MID 151, checksum 0x02 0x27 / 1 sec 20% ATP
    // uint8_t value[] = {0x97, 0x00, 0x00, 0x01, 0x2C, 0x00, 0x00, 0x01, 0x2C}; // MID 151, checksum 0x00 0xF1 / 1 sec 30% ATP
    // uint8_t value[] = {0x97, 0x00, 0x00, 0x01, 0x90, 0x00, 0x00, 0x01, 0x90}; // MID 151, checksum 0x01 0xB9 / 1 sec 40% ATP
    // uint8_t value[] = {0x97, 0x00, 0x00, 0x01, 0x2C, 0x00, 0x00, 0x02, 0x58}; // MID 151, checksum 0x01 0x1E / 2 sec 30% ATP
    // uint8_t value[] = {0x97, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x01, 0xF4}; // MID 151, checksum 0x01 0xF0 / 5 sec 10% ATP
    // uint8_t value[] = {0xA7, 0x00, 0x00, 0x75, 0x30, 0x00, 0x01, 0xD4, 0xC0, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00}; // MID 167, checksum 0x03 0x1D / 60 sec PTF
    // uint8_t value[] = {0xA7, 0x00, 0x00, 0x75, 0x30, 0x00, 0x01, 0xD4, 0xC0, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00};  // MID 167, checksum 0x02 0xEB / 10 sec PTF
    // uint8_t value[] = {0xA7, 0x00, 0x00, 0x75, 0x30, 0x00, 0x01, 0xD4, 0xC0, 0x00, 0x00, 0x00, 0x0A, 0x11, 0x11}; // MID 167, checksum 0x03 0x0D / ATP on
    // uint8_t value[] = {0x81, 0x02, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x05, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0xE1, 0x00}; // MID 129, checksum 0x01 0x76 / switch to NMEA protocol, 57600 baud
    // uint8_t value[] = {0x81, 0x02, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x05, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0xC2, 0x00}; // MID 129, checksum 0x01 0x57 / switch to NMEA protocol, 115200 baud
    // uint8_t value[] = {0xD5, 0x02, 0x00}; // MID 213 SID 2, checksum 0x00 0xD7 / session closing request
    // uint8_t value[] = {0x80, 0x00, 0x2A, 0x3E, 0x7F, 0x00, 0x18, 0xAC, 0x37, 0x00, 0x53, 0xD9, 0xF7, 0x00, 0x01, 0x24, 0xF8, 0x00, 0x83, 0xD6, 0x00, 0x08, 0xF7, 0x0C, 0x33}; // MID 128, checksum 0x08 0x39 / initialize data source with coordinates 59.9067 N, 30.2871 E, 20 m altitude corresponding to ECEF XYZ 2768511 m, 1616951 m, 5495287 m, clock offset of 75000 Hz, time of week of 86400 * 100 sec, week number of 2295 corresponding to the 31st December of 2023
    // uint8_t value[] = {0x80, 0x00, 0x2A, 0x3E, 0x7F, 0x00, 0x18, 0xAC, 0x37, 0x00, 0x53, 0xD9, 0xF7, 0x00, 0x01, 0x24, 0xF8, 0x01, 0x07, 0xAC, 0x00, 0x08, 0xF7, 0x0C, 0x33}; // MID 128, checksum 0x07 0x94 / initialize data source with coordinates 59.9067 N, 30.2871 E, 20 m altitude corresponding to ECEF XYZ 2768511 m, 1616951 m, 5495287 m, clock offset of 75000 Hz, time of week of 172800 * 100 sec, week number of 2295 corresponding to the 1st January of 2024
    // uint8_t value[] = {0x80, 0x00, 0x2A, 0x3E, 0x7F, 0x00, 0x18, 0xAC, 0x37, 0x00, 0x53, 0xD9, 0xF7, 0x00, 0x01, 0x24, 0xF8, 0x00, 0x83, 0xD6, 0x00, 0x08, 0xF7, 0x0C, 0xB6}; // MID 128, checksum 0x08 0x8C vs 0x08 0xBC / full system reset
    uint8_t value[] = {0x96, 0x1E, 0x54, 0x00, 0x06, 0x00, 0x3C}; // MID 150, checksum 0x01, 0x4A / switch operation modes - test mode 4 for 60 seconds
    uint8_t size4 = sizeof(value);
    while (i < size4) { // this method for checksum is from OriginGPS OSP manual, SiRF protocol checksum example code
      checkSum = checkSum + value[i];
      checkSum = checkSum & (0x7FFF);
      i++;
    }
    uint32_t __checkSum = (checkSum >> 8) | (checkSum << 8); // move byte 1 to byte 0, byte 0 to byte 1
    // NRF_LOG_INFO("__checkSum = ");
    // NRF_LOG_HEXDUMP_INFO(&__checkSum, 2); // shows exact checksum two bytes

    // Send TriclePower parameters. See OSP message MID 151 from OriginGps OSP Manual
    // uint8_t commandMID151[] = {0xA0, 0xA2, 0x00, 0x09, 0x97, 0x11, 0x11, 0x00, 0xC8, 0x00, 0x00, 0x00, 0xC8, 0x02, 0x49, 0xB0, 0xB3, 0x0D, 0x0A}; // for PTF 10 sec
    // uint8_t commandMID151[] = {0xA0, 0xA2, 0x00, 0x09, 0x97, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0xC8, 0x02, 0x27, 0xB0, 0xB3, 0x0D, 0x0A}; // for ATP 1 sec 20%
    uint8_t commandMID151[] = {0xA0, 0xA2, 0x00, 0x09, 0x97, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x01, 0xF4, 0x01, 0xF0, 0xB0, 0xB3, 0x0D, 0x0A}; // for ATP 5 sec 10%
    // uint8_t commandMID151[] = {0xA0, 0xA2, 0x00, 0x09, 0x97, 0x00, 0x00, 0x01, 0x2C, 0x00, 0x00, 0x02, 0x58, 0x01, 0x1E, 0xB0, 0xB3, 0x0D, 0x0A}; // for ATP 1 sec 30%
    command_size = sizeof(commandMID151);
    err_code = nrf_libuarte_async_tx(&libuarte, commandMID151, command_size);
    ii = 0;
    while (err_code > 0 || ii < 10) {
      err_code = nrf_libuarte_async_tx(&libuarte, commandMID151, command_size);
      ii++;
    }
    APP_ERROR_CHECK(err_code);

    // Send low power aquisition parameteres. See OSP message MID 167 from OriginGps OSP Manual
    // uint8_t commandMID167[] = {0xA0, 0xA2, 0x00, 0x0F, 0xA7, 0x00, 0x00, 0x75, 0x30, 0x00, 0x01, 0xD4, 0xC0, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02, 0xEB, 0xB0, 0xB3, 0x0D, 0x0A}; // for PTF 10 sec
    uint8_t commandMID167[] = {0xA0, 0xA2, 0x00, 0x0F, 0xA7, 0x00, 0x00, 0x75, 0x30, 0x00, 0x01, 0xD4, 0xC0, 0x00, 0x00, 0x00, 0x0A, 0x11, 0x11, 0x03, 0x1D, 0xB0, 0xB3, 0x0D, 0x0A}; // for ATP 1 sec and 5 sec
    command_size = sizeof(commandMID167);
    err_code = nrf_libuarte_async_tx(&libuarte, commandMID167, command_size);
    ii = 0;
    while (err_code > 0 || ii < 10) {
      err_code = nrf_libuarte_async_tx(&libuarte, commandMID167, command_size);
      ii++;
    }
    APP_ERROR_CHECK(err_code);

    // Go back to NMEA output. See OSP message MID 129 from OriginGps OSP Manual
    static uint8_t commandMID129[] = {0xA0, 0xA2, 0x00, 0x18, 0x81, 0x02, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x05, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0xC2, 0x00, 0x01, 0x57, 0xB0, 0xB3, 0x0D, 0x0A}; // 115200 baud
    command_size = sizeof(commandMID129);
    err_code = nrf_libuarte_async_tx(&libuarte, commandMID129, command_size);
    ii = 0;
    while (err_code > 0 || ii < 10) {
      err_code = nrf_libuarte_async_tx(&libuarte, commandMID129, command_size);
      ii++;
    }
    APP_ERROR_CHECK(err_code);

    if (err_code == NRF_SUCCESS) {
      NRF_LOG_INFO("ORG1410 initialization SUCCESS"); // we has sucessfully sent all the commands via uart - that means GPS is OK
    }
  }

  static uint32_t timeGPSMaxSearch = 240000; // milliseconds
  static uint32_t timeGPSMaxSleep = 120000;  // milliseconds

  static bool forcedGPSTurnOff = 0;

  static uint32_t ticksGPSFullPowerStart = 0;
  static uint32_t ticksGPSLowPowerStart = 0;

  void handle_gps_power(void) {
    if (nrf_gpio_pin_read(37) > 0) { // GPS full power detected
      if (timeGPSLowPower > 0) {     // first moment of transition to full power
        ticksGpsFullPowerStart = app_timer_cnt_get();
        // timeGPSFullPowerStart = millis();
        timeGPSLowPowerTotal = timeGPSLowPowerTotal + timeGPSLowPower;
        powerRatioGPS = (float)timeGPSFullPowerTotal / (float)timeGPSLowPowerTotal;

        forcedGPSTurnOff = 0;
        timeGPSLowPower = 0; // to run this brackets once

        NRF_LOG_INFO("GPS transition to full power");
        NRF_LOG_INFO("Time GPS full power total = %u", timeGPSFullPowerTotal);
        NRF_LOG_INFO("Time GPS low power total = %u", timeGPSLowPowerTotal);
        NRF_LOG_INFO("GPS power ratio = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(powerRatioGPS));
        NRF_LOG_FLUSH();

        /*memset(fatfsString, 0, sizeof(fatfsString));
        (void)snprintf(fatfsString, sizeof(fatfsString), "\n \nGPS transition to full power\nTime GPS full power total = %u\nTime GPS low power total = %u\nGPS power ratio = %f\n", timeGPSFullPowerTotal, timeGPSLowPowerTotal, powerRatioGPS);
        fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString)));*/
      }

      timeGPSFullPower = app_timer_cnt_diff_compute(app_timer_cnt_get(), ticksGPSFullPowerStart) / 16.384;
      // timeGPSFullPower = millis() - timeGPSFullPowerStart;

      if (timeGPSFullPower > timeGPSMaxSearch) { // force GPS turn off if needed
        turn_off_GPS();
        forcedGPSTurnOff = 1;
      }
    }

    if (nrf_gpio_pin_read(37) < 1) { // GPS low power detected
      if (timeGPSFullPower > 0) {    // first moment of transition to low power
        ticksGPSLowPowerStart = app_timer_cnt_get();
        // timeGPSLowPowerStart = millis();
        timeGPSFullPowerTotal = timeGPSFullPowerTotal + timeGPSFullPower;
        powerRatioGPS = (float)timeGPSFullPowerTotal / (float)timeGPSLowPowerTotal;

        timeGPSFullPower = 0; // to run this brackets once

        if (forcedGPSTurnOff) { // if GPS turn off was forced by algorythm

          numberGPSForcedTransitions++;
          NRF_LOG_INFO("-----------------------------");
          NRF_LOG_INFO("GPS forced transition to low power");
          NRF_LOG_FLUSH();

          /*memset(fatfsString, 0, sizeof(fatfsString));
          (void)snprintf(fatfsString, sizeof(fatfsString), "\n-----------------------------\nGPS FORCED transition to low power\n");
          fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString)));*/
        } else { // if turn off was self-started by GPS
          NRF_LOG_INFO("GPS self-transition to low power");
          NRF_LOG_FLUSH();

          /*memset(fatfsString, 0, sizeof(fatfsString));
          (void)snprintf(fatfsString, sizeof(fatfsString), "\n-----------------------------\nGPS SELF-transition to low power\n");
          fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString)));*/
        }

        NRF_LOG_INFO("Time GPS full power total = %u", timeGPSFullPowerTotal);
        NRF_LOG_INFO("Time GPS low power total = %u", timeGPSLowPowerTotal);
        NRF_LOG_INFO("GPS power ratio = " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(powerRatioGPS));
        NRF_LOG_FLUSH();

        /*memset(fatfsString, 0, sizeof(fatfsString));
        (void)snprintf(fatfsString, sizeof(fatfsString), "\nTime GPS full power total = %u\nTime GPS low power total = %u\nGPS power ratio = %f\n", timeGPSFullPowerTotal, timeGPSLowPowerTotal, powerRatioGPS);
        fatfs_append_line("log.txt", fatfsString, strnlen(fatfsString, sizeof(fatfsString)));*/
      }

      timeGPSLowPower = app_timer_cnt_diff_compute(app_timer_cnt_get(), ticksGPSLowPowerStart) / 16.384;
      // timeGPSLowPower = millis() - timeGPSLowPowerStart;

      if (timeGPSLowPower > timeGPSMaxSleep) { // turn on GPS
        turn_on_GPS();
      }
    }
  }

  static uint32_t ticksTurnOnGps = 0;

  void gps_check(void) {
    handle_gps_power();
    handle_gps_buffer();

    if (turnOnGpsFlag == 1) {          // if GPS turn on was just requested
      if (nrf_gpio_pin_read(37) < 1) { // if GPS has not turned on yet after the initial try in gps_turn_on() function
        ticksTurnOnGps = app_timer_cnt_get();
        turnOnGpsTime = app_timer_cnt_diff_compute(ticksTurnOnGps, ticksTurnOnGpsPrevious) / 16.384;
        // turnOnGpsMillis = millis();
        // turnOnGpsTime = turnOnGpsMillis - turnOnGpsMillisPrevious;

        if (turnOnGpsTime > 100) { // 100 milliseconds between tries
          // NRF_LOG_INFO("Next attempt to turn on GPS");
          // NRF_LOG_FLUSH();
          nrf_gpio_pin_set(39);
          nrf_delay_ms(3);
          nrf_gpio_pin_clear(39);
          turnOnGpsMillisPrevious = turnOnGpsMillis;
        }
      } else { // if GPS has turned on
        turnOnGpsFlag = 0;
      }
    }

    if (turnOffGpsFlag == 1) {                                 // if GPS turn off was just requested
      if (nrf_gpio_pin_read(37) > 0) {                         // if GPS has not turned off yet after the initial try in gps_turn_off() function
        static uint8_t commandMID100[] = "$PSRF117,16*0B\r\n"; // is "turn off" command
        static uint8_t command_size = sizeof(commandMID100);
        ret_code_t err_code = nrf_libuarte_async_tx(&libuarte, commandMID100, command_size);
        while (err_code > 0) {
          handle_gps_buffer();                                                      // empty the buffer before turning off GPS
          err_code = nrf_libuarte_async_tx(&libuarte, commandMID100, command_size); // try to turn off here and check it in the gps_check() function
        }
        APP_ERROR_CHECK(err_code);
      } else { // if GPS has turned off
        turnOffGpsFlag = 0;
        uart_uninit();
      }
    }
  }

  //------------------------------------------------------------------------------------
  // Flying App part
  //------------------------------------------------------------------------------------

  static uint32_t nmea0183_checksum(char *nmea_data) {
    uint32_t crc = 0;
    uint32_t i;
    for (i = 0; i < strlen(nmea_data); i++) { // calculation for data between the first $ sign and the last * sign: for (i = 0; i < strlen(nmea_data); i++) /// OR /// calculation for full sentence excluding the $ sign, the last two bytes of original CRC and  the * sign: for (i = 1; i < strlen(nmea_data) - 3; i++)
      crc ^= nmea_data[i];
    }

    return crc; // note that crc in NMEA should be in HEX format
  }

  static void bluetooth_app_send(void) {
    char xtrcArray[25][25] = {0};

    /*strncpy(xtrcArray[0], "2015", 24); // valid data example
    strncpy(xtrcArray[1], "1", 24);
    strncpy(xtrcArray[2], "5", 24);
    strncpy(xtrcArray[3], "16", 24);
    strncpy(xtrcArray[4], "34", 24);
    strncpy(xtrcArray[5], "33", 24);
    strncpy(xtrcArray[6], "36", 24);
    strncpy(xtrcArray[7], "46.947508", 24);
    strncpy(xtrcArray[8], "7.453117", 24);
    strncpy(xtrcArray[9], "540.32", 24);
    strncpy(xtrcArray[10], "12.35", 24);
    strncpy(xtrcArray[11], "270.4", 24);
    strncpy(xtrcArray[12], "2.78", 24);
    strncpy(xtrcArray[13], "964.93", 24);
    strncpy(xtrcArray[14], "98", 24);*/

    char year[25] = {0};
    strncpy(year, gprmcArray[9] + 4, 2); // copy first 4 characters from 10-th line of NMEA GPRMS sentence (year)
                                         // if(strlen(year) > 0) {
    int32_t x = atoi(year);
    if (x > 22) {                                  // the GPS may initialise in veird date, like year 2011: check it's more than 2022
      snprintf(xtrcArray[0], 24, "20%s", year);    // format year from "23" to "2023"
      strncpy(xtrcArray[1], gprmcArray[9] + 2, 2); // month
      strncpy(xtrcArray[2], gprmcArray[9], 2);     // day
    }

    strncpy(xtrcArray[3], gpggaArray[1], 2);     // hour
    strncpy(xtrcArray[4], gpggaArray[1] + 2, 2); // minute
    strncpy(xtrcArray[5], gpggaArray[1] + 4, 2); // second
    strncpy(xtrcArray[6], gpggaArray[1] + 7, 3); // centisecond

    strncpy(xtrcArray[7], gpggaArray[2], 2);     // latitude, degrees part are the first two digits
    strcat(xtrcArray[7], ".");                   // add decimal point
    strncat(xtrcArray[7], gpggaArray[2] + 2, 2); // this is needed because of the difference between NMEA and XCTRC formats (DDmm.mm versus DD.mmmm)
    strncat(xtrcArray[7], gpggaArray[2] + 5, 5); // this is needed because of the difference between NMEA and XCTRC formats (DDmm.mm versus DD.mmmm)

    strncpy(xtrcArray[8], gpggaArray[4], 3);     // longitude, degrees part are the first three digits
    strcat(xtrcArray[8], ".");                   // add decimal point
    strncat(xtrcArray[8], gpggaArray[4] + 3, 2); // this is needed because of the difference between NMEA and XCTRC formats (DDmm.mm versus DD.mmmm)
    strncat(xtrcArray[8], gpggaArray[4] + 6, 5); // this is needed because of the difference between NMEA and XCTRC formats (DDDmm.mm versus DD.mmmm)

    snprintf(xtrcArray[9], 24, "%.2f", altitudeGPS);                    // altitude in m
    snprintf(xtrcArray[10], 24, "%.2f", velocityGPS);                   // horizontal speed in m/s
    snprintf(xtrcArray[11], 24, "%.2f", azimuthGPS);                    // azimuth in degrees
    snprintf(xtrcArray[12], 24, "%.2f", climbrateKalman);               // vertical speed in m/s
    snprintf(xtrcArray[13], 24, "%.2f", (float)(pressureBaro / 100.f)); // pressure in hPa
    snprintf(xtrcArray[14], 24, "%d", batt_status());                   // battery charge in %

    // NRF_LOG_INFO("%s", xtrcArray[0]);

    char S1[BLE_NUS_MAX_DATA_LEN] = {0};
    // snprintf(S1, sizeof(S1), "$XCTRC,2015,1,5,16,34,33,36,46.947508,7.453117,540.32,12.35,270.4,2.78,,,,964.93,98*79\n"); // $XCTRC,year,month,day,hour,minute,second,centisecond,latitude,longitude,altitude,speedoverground,course,climbrate,res,res,res,rawpressure,batteryindication*NMEAchecksum
    snprintf(S1, sizeof(S1), "XCTRC,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,,,,%s,%s",
        xtrcArray[0], xtrcArray[1], xtrcArray[2], xtrcArray[3], xtrcArray[4],
        xtrcArray[5], xtrcArray[6], xtrcArray[7], xtrcArray[8], xtrcArray[9],
        xtrcArray[10], xtrcArray[11], xtrcArray[12], xtrcArray[13], xtrcArray[14]);
    // strncpy(S1, xtrcArray, sizeof(xtrcArray));

    char checksum[25] = {0};
    snprintf(checksum, 24, "%x", nmea0183_checksum(S1));

    char S2[BLE_NUS_MAX_DATA_LEN] = {0};
    snprintf(S2, sizeof(S2), "$%s*%s\n", S1, checksum);
    // snprintf(S2, sizeof(S2), "$%s\n", xtrcArray);

    // NRF_LOG_INFO("%s", S2);

    bluetooth_send(S2); // bluetooth
  }

  //------------------------------------------------------------------------------------
  // Main part
  //------------------------------------------------------------------------------------
  int main(void) {
    ret_code_t err_code;

    nrf_gpio_cfg_output(LED);
    nrf_gpio_pin_clear(LED); // instantly lights LED for the case of wake up

    nrf_gpio_cfg_output(LOAD_EN);
    nrf_gpio_pin_set(LOAD_EN); // turns on power for SPI flash operation
    nrf_delay_ms(100);

    log_init();

    NRF_LOG_INFO(" ");
    NRF_LOG_INFO("Starting firmware...");

    timers_init();
    usb_init();        // must be after timers_init // custom add to SDK, see https://devzone.nordicsemi.com/f/nordic-q-a/36654/usbd_msd-disk-initialization-fails-in-usb-unplug-with-sdk15-0
    softdevice_init(); // must be after usb_init
    button_init();
    led_pwm_init();
    adc_init();
    wake_up(); // wakeup handling after going out of sleep, must be after softdevice init
    imu_init();
    baro_init(); // must be before ble_init
    // gps_init();
    ble_init();
    power_management_init();
    miscellaneous_init();

    vario_configure();

    NRF_LOG_INFO("Firmware has been started!");
    NRF_LOG_INFO(" ");

    for (;;) { // enter the main loop
      vario_check();
      usb_check();
      button_check();
      snd_check();
      // gps_check();
      sleep_check();

      idle_state_handle();

      NRF_LOG_FLUSH();
    }
  }
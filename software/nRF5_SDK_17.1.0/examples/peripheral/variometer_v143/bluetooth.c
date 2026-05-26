#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "ble_hci.h"
#include "ble_nus.h"
#include "ble_srv_common.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "diskio_blkdev.h"
#include "fds.h"
#include "ff.h"
#include "nordic_common.h"
#include "nrf.h"
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
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"

#define APP_BLE_CONN_CFG_TAG 1  // a tag identifying the SoftDevice BLE configuration
#define APP_BLE_OBSERVER_PRIO 3  // application's BLE observer priority. You shouldn't need to modify this value
#define APP_ADV_INTERVAL 64  // the advertising interval (in units of 0.625 ms. This value corresponds to 40 ms)
#define APP_ADV_DURATION 0  // the advertising duration in units of 10 milliseconds, 0 is endless
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)  // connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units
#define DEAD_BEEF 0xDEADBEEF // value used as error code on stack dump, can be used to identify stack location on stack unwind
//#define DEVICE_NAME "Ungravity Vario"  // name of device. Will be included in the advertising data
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) // time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds)
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)  // maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)  // minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units
#define MAX_CONN_PARAMS_UPDATE_COUNT 3
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) // time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds)
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN  // UUID type for the Nordic UART Service (vendor specific)
#define SEC_PARAM_BOND 1  // perform bonding
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE // no I/O capabilities
#define SEC_PARAM_KEYPRESS 0  // keypress notifications not enabled
#define SEC_PARAM_LESC 0  // LE Secure Connections not enabled
#define SEC_PARAM_MIN_KEY_SIZE 7  // minimum encryption key size
#define SEC_PARAM_MITM 0  // Man In The Middle protection not required
#define SEC_PARAM_MAX_KEY_SIZE 16  //  maximum encryption key size
#define SEC_PARAM_OOB 0  // Out Of Band data not available
#define SLAVE_LATENCY 0 // slave latency

BLE_ADVERTISING_DEF(m_advertising);  // advertising module instance
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);  // BLE NUS service instance
NRF_BLE_GATT_DEF(m_gatt);  // GATT module instance
NRF_BLE_QWR_DEF(m_qwr);  // context for the Queued Write module

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;  // handle of the current connection
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;  // maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  // universally unique service identifier

typedef void (* bluetooth_get_handle_t) (char S[BLE_NUS_MAX_DATA_LEN]);
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
    nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);  // softdevice was disabled before going into reset, inform bootloader to skip CRC on next boot
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);  // go to system off
  }
}

NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = { .handler = buttonless_dfu_sdh_state_observer, };

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

    ble_adv_modes_config_t config;  // prevent device from advertising on disconnect
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
  //uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  //APP_ERROR_CHECK(err_code);

  //err_code = bsp_btn_ble_sleep_mode_prepare();  // prepare wakeup buttons
  //APP_ERROR_CHECK(err_code);

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
    //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    //APP_ERROR_CHECK(err_code);
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
    //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);
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
    err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);  // pairing not supported
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_SYS_ATTR_MISSING:
    err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);  // no system attributes have been stored
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTC_EVT_TIMEOUT:
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);  // disconnect on GATT Client timeout event
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);  // disconnect on GATT Server timeout event
    APP_ERROR_CHECK(err_code);
    break;

  default:
    break;  //  no implementation needed
  }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
  ret_code_t err_code;

  //err_code = nrf_sdh_enable_request();
  //APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_sdh_ble_enable(&ram_start);  // enable BLE stack
  APP_ERROR_CHECK(err_code);

  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);  // register a handler for BLE events
}


static void nus_data_handler(ble_nus_evt_t *p_evt) {
  if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
    char S[BLE_NUS_MAX_DATA_LEN] = {0};
    for (int i = 0; i < p_evt->params.rx_data.length; i++) {S[i] = p_evt->params.rx_data.p_data[i];}
  
    bluetooth_get_handle(S); // bluetooth
  }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(char* DEVICE_NAME) {
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

  qwr_init.error_handler = nrf_qwr_error_handler;  // initialize Queued Write Module
  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

  dfus_init.evt_handler = ble_dfu_evt_handler; // DFU
  err_code = ble_dfu_buttonless_init(&dfus_init);

  memset(&nus_init, 0, sizeof(nus_init)); // initialize NUS
  nus_init.data_handler = nus_data_handler; // bluetooth
  err_code = ble_nus_init(&m_nus, &nus_init);
  APP_ERROR_CHECK(err_code);
}

static void bluetooth_init(char* DEVICE_NAME, bluetooth_get_handle_t bluetooth_get, ble_dfu_buttonless_evt_handler_t ble_dfu_evt_handler) {
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
    APP_ERROR_CHECK(err_code);
  }
}

static void bluetooth_send_value(uint32_t value) {
  char S[BLE_NUS_MAX_DATA_LEN] = {0};
  snprintf(S, sizeof(S), "PRS %x\n", value);
  
  bluetooth_send(S);  // bluetooth
}

static void ble_init(void) {
  bool erase_bonds;
  ble_stack_init();
  bluetooth_init("Ungravity Vario 1.2", bluetooth_get, ble_dfu_evt_handler); // firmware releaase version is in the name of Ungravity Vario
  advertising_init();
  conn_params_init();
  advertising_start();
}
/*
 * The library is not extensively tested and only
 * meant as a simple explanation and for inspiration.
 * NO WARRANTY of ANY KIND is provided.
 */

//#if defined(MPU_USES_TWI) // Use TWI drivers

#include "nrfx_twi.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_util_platform.h"
#include "nrf_gpio.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "twi_func.h"

#define E73
//#define Holyiot

#if defined Holyiot
#define MPU_TWI_SDA_PIN 26
#define MPU_TWI_SCL_PIN 27

#elif defined E73
#define MPU_TWI_SDA_PIN 16
#define MPU_TWI_SCL_PIN 15
#define ICM_AD0_PIN 13
#endif

#define MPU_ADDRESS 0x68
//#define MPU_ADDRESS     			0x69

#define MPU_AK99XX_MAGN_ADDRESS 0x0C

#define MPU_TWI_BUFFER_SIZE 14 // 14 byte buffers will suffice to read acceleromter, gyroscope and temperature data in one transmission.
#define MPU_TWI_TIMEOUT 10000

static const nrfx_twi_t m_twi_instance = NRFX_TWI_INSTANCE(1);
volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];

static void nrfx_mpu_twi_event_handler(nrfx_twi_evt_t const *p_event, void *p_context) {
  switch (p_event->type) {
  case NRFX_TWI_EVT_DONE:
    switch (p_event->xfer_desc.type) {
    case NRFX_TWI_XFER_TX:
      twi_tx_done = true;
      break;
    case NRFX_TWI_XFER_TXTX:
      twi_tx_done = true;
      break;
    case NRFX_TWI_XFER_RX:
      twi_rx_done = true;
      break;
    case NRFX_TWI_XFER_TXRX:
      twi_rx_done = true;
      break;
    default:
      break;
    }
    break;
  case NRFX_TWI_EVT_ADDRESS_NACK:
    break;
  case NRFX_TWI_EVT_DATA_NACK:
    break;
  default:
    break;
  }
}

static void twi_clear_bus() {
  nrf_gpio_pin_set(MPU_TWI_SCL_PIN);
  nrf_gpio_pin_set(MPU_TWI_SDA_PIN);

  nrf_delay_us(4);

  for (int i = 0; i < 9; i++) {
    if (nrf_gpio_pin_read(MPU_TWI_SDA_PIN)) {
      if (i == 0) {
        return;
      } else {
        break;
      }
    }
    nrf_gpio_pin_clear(MPU_TWI_SCL_PIN);
    nrf_delay_us(4);
    nrf_gpio_pin_set(MPU_TWI_SCL_PIN);
    nrf_delay_us(4);
  }
  nrf_gpio_pin_clear(MPU_TWI_SDA_PIN);
  nrf_delay_us(4);
  nrf_gpio_pin_set(MPU_TWI_SDA_PIN);
}

/**
 * @brief TWI initialization.
 * Just the usual way. Nothing special here
 */
int twi_init_imu(void) {
  //NRF_LOG_INFO("ICM20948 twi init");
  
  uint32_t err_code;
  const nrfx_twi_config_t twi_mpu_config = {
      .scl = MPU_TWI_SCL_PIN,
      .sda = MPU_TWI_SDA_PIN,
      .frequency = NRF_TWI_FREQ_400K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
      //.clear_bus_init     = false
  };

  //  twi_clear_bus;  //experiemntal SL
  err_code = nrfx_twi_init(&m_twi_instance, &twi_mpu_config, nrfx_mpu_twi_event_handler, NULL);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }
  // twi_clear_bus;  //not helping
  nrfx_twi_enable(&m_twi_instance);
  return NRF_SUCCESS;
}

int twi_uninit_imu(void) {
  nrfx_twi_disable(&m_twi_instance);
}

int twi_enable(void) { nrfx_twi_enable(&m_twi_instance); }

int twi_disable(void) { nrfx_twi_disable(&m_twi_instance); }

// The TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
// Hence we need to merge the MPU register address with the buffer and then transmit all as one transmission
static void merge_register_and_data(uint8_t *new_buffer, uint8_t reg, uint8_t *p_data, uint32_t length) {
  new_buffer[0] = reg;
  memcpy((new_buffer + 1), p_data, length);
}

int twi_write_reg(uint8_t reg, uint8_t *p_data, uint32_t length) {
  // This burst write function is not optimal and needs improvement.
  // The new SDK 11 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
  uint32_t err_code;
  uint32_t timeout = MPU_TWI_TIMEOUT;

  // Merging MPU register address and p_data into one buffer.
  merge_register_and_data(twi_tx_buffer, reg, p_data, length);

  // Setting up transfer
  nrfx_twi_xfer_desc_t xfer_desc;
  xfer_desc.address = MPU_ADDRESS;
  xfer_desc.type = NRFX_TWI_XFER_TX;
  xfer_desc.primary_length = length + 1;
  xfer_desc.p_primary_buf = twi_tx_buffer;

  // Transferring
  err_code = nrfx_twi_xfer(&m_twi_instance, &xfer_desc, 0);

  while ((!twi_tx_done) && --timeout);
  if (!timeout)
    return NRF_ERROR_TIMEOUT;
  twi_tx_done = false;

  return err_code;
}

int twi_read_reg(uint8_t reg, uint8_t *p_data, uint32_t length) {
  uint32_t err_code;
  uint32_t timeout = MPU_TWI_TIMEOUT;

  err_code = nrfx_twi_tx(&m_twi_instance, MPU_ADDRESS, &reg, 1, false);
  if (err_code != NRF_SUCCESS)
    return err_code;

  while ((!twi_tx_done) && --timeout)
    ;
  if (!timeout)
    return NRF_ERROR_TIMEOUT;
  twi_tx_done = false;

  err_code = nrfx_twi_rx(&m_twi_instance, MPU_ADDRESS, p_data, length);
  if (err_code != NRF_SUCCESS)
    return err_code;

  timeout = MPU_TWI_TIMEOUT;
  while ((!twi_rx_done) && --timeout)
    ;
  if (!timeout)
    return NRF_ERROR_TIMEOUT;
  twi_rx_done = false;

  return err_code;
}
/*
int twi_read_mag_reg(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    err_code = nrfx_twi_tx(&m_twi_instance, MPU_AK99XX_MAGN_ADDRESS, &reg, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    err_code = nrfx_twi_rx(&m_twi_instance, MPU_AK99XX_MAGN_ADDRESS, p_data, length);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = MPU_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_rx_done = false;

    return err_code;
}*/

/**
  @}
*/
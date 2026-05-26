#include "twi_func.h"

#include "nrf_delay.h"
#include "boards.h"
#include "nrfx_twim.h"
#include "nrf_gpio.h"

#include "app_util_platform.h"
#include "app_error.h"

#include "nrf_log.h"

#define MPU_TWI_TIMEOUT     1000 


#define MPU_TWI_BUFFER_SIZE     	14 // 14 byte buffers will suffice to read acceleromter, gyroscope and temperature data in one transmission.

#define MPU_TWI_SDA_PIN 26
#define MPU_TWI_SCL_PIN 27

/* TWI instance ID. */
#define TWI_INSTANCE_ID      0
#define MPU_ADDR             0x68
#define MAG_ADDR             0x0C

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(0);

//static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief TWI events handler.
*/
volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];

static void twi_event_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRFX_TWIM_XFER_TX:
                    twi_tx_done = true;
                    break;
                case NRFX_TWIM_XFER_TXTX:
                    twi_tx_done = true;
                    break;
                case NRFX_TWIM_XFER_RX:
                    twi_rx_done = true;
                    break;
                case NRFX_TWIM_XFER_TXRX:
                    twi_rx_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRFX_TWIM_EVT_ADDRESS_NACK:
            break;
        case NRFX_TWIM_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}
/*
static void twi_handler(nrfx_twim_evt_t const * p_event, void * p_context)    // not used
{
    m_xfer_done = true;
    //    switch (p_event->type)
    //    {
    //        case NRF_DRV_TWI_EVT_DONE:
    //            m_xfer_done = true;
    //            break;
    //        default:
    //            break;
    //    }
}
*/

int twi_init()
{
    ret_code_t err_code;
    

   const nrfx_twim_config_t twi_config = {
       .scl                = MPU_TWI_SCL_PIN,  //ARDUINO_SCL_PIN,
       .sda                = MPU_TWI_SDA_PIN, // ARDUINO_SDA_PIN,
       .frequency          = NRF_TWIM_FREQ_400K,
       .interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,                    
      .hold_bus_uninit     = NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT
       
    };

    err_code = nrfx_twim_init(&m_twi, &twi_config, NULL, NULL);
    nrf_delay_ms(1);  //important
    APP_ERROR_CHECK(err_code);
    
    nrfx_twim_enable(&m_twi);
}


int twi_read_reg(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;
    //uint8_t mpu_addr = MPU_ADDR;
   // uint8_t myreg = reg;
 

    err_code =  nrfx_twim_tx(&m_twi, MPU_ADDR, &reg, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;
 
    err_code = nrfx_twim_rx(&m_twi, MPU_ADDR, p_data, length);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_rx_done = false;

       // NRF_LOG_INFO("twi data 0x%02x : ", data[i]);
    
    
    return NRF_SUCCESS;
}
/*
int twi_write_reg(uint8_t reg, uint8_t *p_data, uint32_t length)
{
    
    uint8_t tx_data[(length+1)];

  //  NRF_LOG_INFO("twi write  0x%02x , 0x%02x",MPU_ADDR, reg);
    //NRF_LOG_FLUSH();
    
    tx_data[0] = reg;
    memcpy(&tx_data[1], p_data, length);
    
    //m_xfer_done = false;
    APP_ERROR_CHECK( nrfx_twim_tx(&m_twi, MPU_ADDR, tx_data, sizeof(tx_data), true) ); //no stop
    //while (m_xfer_done == false) { __WFE(); }
    
    nrf_delay_ms(1);
    
    return NRF_SUCCESS;
}
*/
int twi_read_mag_reg(uint8_t reg, uint8_t *p_data, uint32_t length)
{   //set INT_PIN_CFG to 02 before reading
    uint32_t err_code;

   // NRF_LOG_INFO("twi read mag 0x%02x , 0x%02x ",MAG_ADDR, reg);
   
    err_code =  nrfx_twim_tx(&m_twi, MAG_ADDR, &reg, 1, false);
    //if(err_code != NRF_SUCCESS) return err_code;

    err_code = nrfx_twim_rx(&m_twi, MAG_ADDR, p_data, length);
   // if(err_code != NRF_SUCCESS) return err_code;
   
  
    return NRF_SUCCESS;
}


//uint32_t  nrf_mpu_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t  * p_data)
//uint32_t  Sensors_I2C_WriteRegister(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t  * p_data)
int twi_write_reg(uint8_t reg, uint8_t *p_data, uint32_t length)
{
    // This burst write function is not optimal and needs improvement.
    // The new SDK 11 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    uint8_t twi_tx_buffer[length+1];
    twi_tx_buffer[0] = reg;
    memcpy(&twi_tx_buffer[1], p_data, length);

    // Merging MPU register address and p_data into one buffer.
    //merge_register_and_data(twi_tx_buffer, reg_addr, p_data, length);
     
    // Setting up transfer
    nrfx_twim_xfer_desc_t xfer_desc;
    xfer_desc.address = MPU_ADDR;
    xfer_desc.type = NRFX_TWIM_XFER_TX;
    xfer_desc.primary_length = length + 1;
    xfer_desc.p_primary_buf = twi_tx_buffer;

    // Transferring
    err_code = nrfx_twim_xfer(&m_twi, &xfer_desc, 0);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;
   
    return err_code;
}


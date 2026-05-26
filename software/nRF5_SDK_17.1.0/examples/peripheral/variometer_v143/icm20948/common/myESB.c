#include <stdint.h>
#include <string.h>
//#include <stdio.h>   // needed for uart_get and printf
#include <stdlib.h>     /* atoi */

/* from esb exemple */
#include "sdk_common.h"  
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "app_util.h"
#include "nrf_gpio.h"
/* from esb exemple */

#include "myESB.h"
#include "nrf_log.h"
#include "timestamping.h"
#include "nrf_delay.h"


 

#define   MYPIPE     1

//extern char ESBstring[64]; // to check 



/****************  esb ************/
/*uint32_t esbtime;
static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);
static nrf_esb_payload_t        rx_payload;

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{ char rx_string[32];
  bool rx_data = false;
  uint8_t rx_pipe;

  int length = 0;
  char rchar;

    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
           //  NRF_LOG_DEBUG("TX SUCCESS EVENT")
             //tx_success;
             break;

        case NRF_ESB_EVENT_TX_FAILED:
             NRF_LOG_INFO("TX FAILED EVENT");
             nrf_gpio_pin_set(18);
             nrf_gpio_pin_clear(20);  // red on
             (void) nrf_esb_flush_tx();
             (void) nrf_esb_start_tx();
             break;
        case NRF_ESB_EVENT_RX_RECEIVED:
             //NRF_LOG_DEBUG("RX RECEIVED EVENT");
            // while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
             if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
                  {  sprintf(rx_string,"%s", rx_payload.data);
                     rx_pipe = rx_payload.pipe;
                     rx_data = true;  

              //***
                    NRF_LOG_INFO("ESB_DATA_RX: %s ", rx_string);

                     if ((rx_string[0] == '+') && (strlen(rx_string) > 2))   // read time from cenral
                        { esbtime  = atoi(rx_string);
                          esbtime = timestamp_func(esbtime);
                          NRF_LOG_INFO("Newtime = %d\n", timestamp_func(NULL)); 
                          timer_start();
                        }  //not OK       
               
                     else if (strcmp(rx_string, "gt") == 0 )     //  ask time to central               
                        {  timer_stop();  // stop sending battery value
                           ESBsend("gt1");   //  ask central to put newtime in ack payload
                           nrf_delay_ms(200);  // < =================  not optimized   10 or 100 is the same
                           ESBsend("gt2");
                           //nrf_delay_ms(10);// get newtime ack payload
                           //timer_start();
                        }  

                     else if (strlen(rx_string) == 2)  // single character received
                        {// rchar = rx_string[0]; // OK//
                         // handle_char_input(rchar);
                          handle_char_input(rx_string[0]); //<========== process input  // ignore warning
                         }   
 
                  }
             break;
    } // switch

} // void






void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;   // dynamic payload length
    nrf_esb_config.payload_length           = 128;//SL  max lenght for nrf52  is 255  // use 80 min
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;
    nrf_esb_config.tx_output_power          = NRF_ESB_TX_POWER_NEG12DBM;  //  _NEG12DB default 0DBM  min neg30   30 = 2pi  12DBM = 30 pi
    //https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/include/nrf_esb.html
    
    tx_payload.noack = false;

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, NRF_ESB_PIPE_COUNT);
    VERIFY_SUCCESS(err_code);

    tx_payload.pipe = MYPIPE;     //  the pipe adress used for transmitting  fist declaration

    return err_code;
}


void ESBsend(char * strpnt)      
{  //  NRF_LOG_INFO("ESBstring + lenght : %s ; %d",strpnt, strlen(strpnt) + 1);

   // tx_payload.pipe = pipe;     //  the pipe adress used for transmitting  fist declaration
    tx_payload.length =  strlen(strpnt) + 1;  //payload lenght+1   // can use  n 
    strcpy(tx_payload.data, strpnt);
    nrf_esb_write_payload(&tx_payload);   
}*/
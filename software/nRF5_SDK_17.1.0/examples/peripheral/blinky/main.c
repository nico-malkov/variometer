#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#define LED 27

int main (void)
{
   nrf_gpio_cfg_output(LED);
   nrf_gpio_pin_set(LED);

   while(1){
     nrf_gpio_pin_clear(LED);
     nrf_delay_ms(500);
     nrf_gpio_pin_set(LED);
     nrf_delay_ms(500);
   }
}
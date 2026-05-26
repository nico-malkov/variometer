#include "timestamping.h"
#include "nrf_drv_clock.h"
#include "app_timer.h"

//#define APP_TIMER_PRESCALER    0  

void lfclk_init(void)
{
    uint32_t err_code;


    APP_ERROR_CHECK(nrf_drv_clock_init())  ;
/*
    err_code = nrf_drv_clock_init();
     if(err_code != NRF_SUCCESS)
	{ return err_code; }*/

    nrf_drv_clock_lfclk_request(NULL);
}

uint32_t ticks_from = 0;
uint32_t timestamp_ms = 0;

//uint32_t timestamp_func(void) {
uint32_t timestamp_func(uint32_t newtime)
{
    uint32_t ticks_diff = 0;
    uint32_t ms_diff = 0;
    uint32_t ticks_to = NRF_RTC0->COUNTER; //app_timer_cnt_get();

  //  if (newtime != NULL) timestamp_ms = newtime; 

   // APP_ERROR_CHECK(app_timer_cnt_diff_compute(ticks_to, ticks_from, &ticks_diff));
   ticks_diff = app_timer_cnt_diff_compute(ticks_to, ticks_from);
   ticks_from = ticks_to;

    ms_diff = APP_TIMER_MS(ticks_diff, APP_TIMER_PRESCALER);  //  see timestamp.h
  // ms_diff = (ticks_diff * (APP_TIMER_PRESCALER + 1) * 1000) / APP_TIMER_CLOCK_FREQ;

     if (newtime != NULL) 
      { timestamp_ms = newtime; 
       // printf( "timestamp_ms = %d %d\n", newtime , timestamp_ms);
      }
       else
      timestamp_ms += ms_diff;


    return timestamp_ms; 
}
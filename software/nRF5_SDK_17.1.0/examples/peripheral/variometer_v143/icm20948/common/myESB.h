
#ifndef _MYESB_H_
#define _MYESB_H_





void clocks_start( void );

uint32_t esb_init( void );


//void nrf_esb_event_handler(nrf_esb_evt_t const * p_event);

//void ESBsend(uint8_t pipe, char string[])  ;
void ESBsend(char * strpnt) ;

#endif  //#define _MYESB_H_

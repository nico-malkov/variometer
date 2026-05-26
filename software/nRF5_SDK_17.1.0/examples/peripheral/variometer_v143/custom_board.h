/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
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

// THIS IS CONFIGURATION FILE FOR UNGRAVITY VARIO PCB VERSION 1.5

#ifndef CUSTOM_H
#define CUSTOM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LOAD_EN            43 // Pin 1.11 enables 1.8 V supply for loads
#define LEDS_NUMBER        1

#define LED                NRF_GPIO_PIN_MAP(0,27) // Pin 0.27 is the LED
#define LED_START          LED
#define LED_STOP           LED

#define LEDS_ACTIVE_STATE  0 // Led is sourcing current, MCU lights LED when pin is pulled to ground

#define LEDS_LIST          { LED }

#define LEDS_INV_MASK      LEDS_MASK

#define BSP_LED            27 // Pin 0.27

#define BUTTONS_NUMBER     1

#define BUTTON             3 // Pin 0.3
#define BUTTON_PULL        NRF_GPIO_PIN_NOPULL // Pull up/down is not necessary

#define BUTTONS_ACTIVE_STATE   1 // Button is logic "1" (1.8 V) level when pressed

#define BUTTONS_LIST       { BUTTON }

//#define BSP_BUTTON         BUTTON

#define RX_PIN_NUMBER      40 // Pin 1.08
#define TX_PIN_NUMBER      8  // Pin 0.08
#define CTS_PIN_NUMBER     41 // Pin 0.11
#define RTS_PIN_NUMBER     11 // Pin 1.09
#define HWFC               false

#define BSP_QSPI_SCK_PIN   19 // Pin 0.19
#define BSP_QSPI_CSN_PIN   17 // Pin 0.17
#define BSP_QSPI_IO0_PIN   20 // Pin 0.20
#define BSP_QSPI_IO1_PIN   21 // Pin 0.21
#define BSP_QSPI_IO2_PIN   22 // Pin 0.22
#define BSP_QSPI_IO3_PIN   23 // Pin 0.23

#ifdef __cplusplus
}
#endif

#endif

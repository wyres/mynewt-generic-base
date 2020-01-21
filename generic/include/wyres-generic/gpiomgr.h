/**
 * Copyright 2019 Wyres
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, 
 * software distributed under the License is distributed on 
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
 * either express or implied. See the License for the specific 
 * language governing permissions and limitations under the License.
*/
#ifndef H_GPIOMGR_H
#define H_GPIOMGR_H

#include <inttypes.h>
#include <mcu/mcu.h>
#include <hal/hal_gpio.h>
#include "lowpowermgr.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 Manage GPIOs centrally to be able to deal with low power enter/exit
*/
#define GPIO_NAME_SZ    (7)
typedef enum  { GPIO_OUT, GPIO_IN, GPIO_IRQ, GPIO_ADC } GPIO_TYPE;

/**
 *  gpio creation mirrors hal calls with extra info
 */
// Define a gpio OUTPUT pin, with a name, an initial value, and the highest lowpower mode it should be active in
void* GPIO_define_out(const char* name, int8_t pin, uint8_t initialvalue, LP_MODE_t offmode);
void* GPIO_define_in(const char* name, int8_t pin,  hal_gpio_pull_t pull, LP_MODE_t offmode);
void* GPIO_define_adc(const char* name, int8_t pin, int adc_chan, LP_MODE_t offmode);
void* GPIO_define_irq(const char* name, int8_t pin, hal_gpio_irq_handler_t handler, void * arg, hal_gpio_irq_trig_t trig, hal_gpio_pull_t pull, LP_MODE_t offmode);

/** 
 * mirror calls for all other hal gpio functions, but that deal with low power operations
 */
void GPIO_release(int8_t pin);

void GPIO_irq_enable(int8_t pin);
void GPIO_irq_disable(int8_t pin);

int GPIO_write(int8_t pin, int val);
int GPIO_read(int8_t pin); 
int GPIO_readADC(int8_t pin); 
#ifdef __cplusplus
}
#endif

#endif  /* H_GPIOMGR_H */

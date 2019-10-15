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
#include "os/os.h"
#include "bsp.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/lowpowermgr.h"
#include "wyres-generic/uartselector.h"

static int8_t _gpio0;
static int8_t _gpio1;
static int8_t _current;


static void uart_selector_setup(int8_t gpio0, int8_t gpio1) {
    _gpio0 = gpio0;
    _gpio1 = gpio1;
    if (_gpio0<0 || _gpio1<0) {
        return;
    }
    GPIO_define_out("US0", gpio0, 0, LP_DEEPSLEEP);
    GPIO_define_out("US1", gpio1, 0, LP_DEEPSLEEP);
    // initialise in hiZ
    uart_select(MYNEWT_VAL(UART_SELECT_HIZ));
}

// Called from sysinit early on
void uart_selector_init(void) {
    // module to select uart switcher - tell it the io lines to control it
    uart_selector_setup(MYNEWT_VAL(UART_SELECT0), MYNEWT_VAL(UART_SELECT1));
}

int8_t uart_select(int8_t id) {
    // TODO deal with locking etc
    int8_t ret = _current;
    if (_gpio0<0 || _gpio1<0) {
        return -1;
    }
    if (id>=0) {
        _current = id;
        GPIO_write(_gpio0, (id & 0x01));
        GPIO_write(_gpio1, (id & 0x02)>1);
    }
    return ret;
}
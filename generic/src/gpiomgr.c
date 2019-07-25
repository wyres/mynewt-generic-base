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
/**
 * gpiomgr : this wraps the basic hal gpio calls with a concept of LPMODE, which defines the power modes of the MCU
 * It will then interface to a power manager, and auto deconfig/reconfig each gpio as the mode changes.
 */

#include <string.h>

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "syscfg/syscfg.h"

#include "bsp/bsp.h"

#include "wyres-generic/wutils.h"

#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/lowpowermgr.h"

#define MAX_GPIOS (MYNEWT_VAL(MAX_GPIOS))

typedef struct gpio {
    int8_t pin;
    GPIO_TYPE type;
    int adc_chan;
    int value;
    hal_gpio_irq_handler_t handler;
    void * arg;
    bool irqEn;
    hal_gpio_irq_trig_t trig;
    hal_gpio_pull_t pull;
    LP_MODE lpmode;
    bool lpEnabled;
    char name[GPIO_NAME_SZ+1];
} GPIO;

static GPIO _gpios[MAX_GPIOS];
static struct os_mutex _gpiomutex;

// function predefs
static GPIO* findGPIO(int8_t p);
static GPIO* allocGPIO(int8_t p);
static void releaseGPIO(GPIO* g);
static void onLPModeChange(LP_MODE current, LP_MODE next);
static void checkForNoADC();
static void init_hal(GPIO* p);
static void deinit_hal(GPIO* p);

void gpio_mgr_init(void) {
    // Initialise gpio array
    memset(&_gpios,0,sizeof(_gpios));
    for(int i=0;i<MAX_GPIOS;i++) {
        _gpios[i].pin = -1;      // all free
    }

    //initialise mutex
    os_mutex_init(&_gpiomutex);

    // Register with LP manager to get callback whenever LP mode changes
    LPMgr_register(&onLPModeChange);
}

// Define a gpio OUTPUT pin, with a name, an initial value, and the highest lowpower mode it should be active in
void* GPIO_define_out(const char* name, int8_t pin, uint8_t initialvalue, LP_MODE offmode) {
    // already setup?
    GPIO* p = allocGPIO(pin);
    if (p!=NULL) {
        strncpy(p->name, name, GPIO_NAME_SZ);
        p->type = GPIO_OUT;
        p->value = (initialvalue!=0?1:0);
        p->lpmode = offmode;
        p->lpEnabled = true;        // assume pin is alive in current lp mode!
        init_hal(p);
    }
    return p;
}

void* GPIO_define_in(const char* name, int8_t pin,  hal_gpio_pull_t pull, LP_MODE offmode) {
        // already setup?
    GPIO* p = allocGPIO(pin);
    if (p!=NULL) {
        strncpy(p->name, name, GPIO_NAME_SZ);
        p->type = GPIO_IN;
        p->pull = pull;
        p->lpmode = offmode;
        p->lpEnabled = true;        // assume pin is alive in current lp mode!
        init_hal(p);
        p->value = hal_gpio_read(pin);
    }
    return p;

}

void* GPIO_define_adc(const char* name, int8_t pin, int adc_chan, LP_MODE offmode) {
    // already setup?
    GPIO* p = allocGPIO(pin);
    if (p!=NULL) {

        strncpy(p->name, name, GPIO_NAME_SZ);
        p->type = GPIO_ADC;
        p->lpmode = offmode;
        p->lpEnabled = true;        // assume pin is alive in current lp mode!
        p->adc_chan = adc_chan;
        init_hal(p);
        p->value = hal_bsp_adc_readmV(adc_chan);
    }
    return p;

}
void* GPIO_define_irq(const char* name, int8_t pin, hal_gpio_irq_handler_t handler, void * arg, hal_gpio_irq_trig_t trig, hal_gpio_pull_t pull, LP_MODE offmode) {
    // already setup?
    GPIO* p = allocGPIO(pin);
    if (p!=NULL) {
        strncpy(p->name, name, GPIO_NAME_SZ);
        p->type = GPIO_IRQ;
        p->handler = handler;
        p->arg = arg;
        p->irqEn = false;           // irq initially disabled, caller must explicitly enable to say go
        p->pull = pull;
        p->trig = trig;
        p->lpmode = offmode;
        p->lpEnabled = true;        // assume pin is alive in current lp mode!
        init_hal(p);
        p->value = hal_gpio_read(pin);
    }
    return p;

}
void GPIO_release(int8_t pin) {
    GPIO* p = findGPIO(pin);
    if(p!=NULL) {
        deinit_hal(p);
        releaseGPIO(p);
    } // ignore if no such pin
}


void GPIO_irq_enable(int8_t pin) {
    GPIO* p = findGPIO(pin);
    assert(p!=NULL);
    assert(p->type==GPIO_IRQ);
    p->irqEn = true;
    if (p->lpEnabled) {
        hal_gpio_irq_enable(p->pin);
    }
}
void GPIO_irq_disable(int8_t pin) {
    GPIO* p = findGPIO(pin);
    assert(p!=NULL);
    assert(p->type==GPIO_IRQ);
    p->irqEn = false;
    if (p->lpEnabled) {
        hal_gpio_irq_disable(p->pin);
    }
}

int GPIO_write(int8_t pin, int val) {
    GPIO* p = findGPIO(pin);
    assert(p!=NULL);
    assert(p->type==GPIO_OUT);
    p->value = (val!=0?1:0);
    if (p->lpEnabled) {
        hal_gpio_write(p->pin, p->value);
        // should re-read the value?
        p->value = hal_gpio_read(p->pin);
    }
    return p->value;
}

int GPIO_read(int8_t pin) {
    GPIO* p = findGPIO(pin);
    assert(p!=NULL);
    // It is allowed to read an output pin...
    if (p->lpEnabled) {
        p->value = hal_gpio_read(p->pin);
    }
    return p->value;
}
// Note pin maps to a ADC channel which may or may not map to an external pin
int GPIO_readADCmV(int8_t pin) {
    GPIO* p = findGPIO(pin);
    assert(p!=NULL);
    // It is allowed to read an output pin...
    if (p->lpEnabled) {
        // Read value. Note we don't use the pin, but the adc channel
        p->value =hal_bsp_adc_readmV(p->adc_chan);
    }
    return p->value;
}
int GPIO_toggle(int8_t pin) {
    GPIO* p = findGPIO(pin);
    assert(p!=NULL);
    assert(p->type==GPIO_OUT);
    p->value = (p->value!=0)?0:1;        // Invert
    if (p->lpEnabled) {
        hal_gpio_write(p->pin, p->value);
        // should re-read the value?
        p->value = hal_gpio_read(p->pin);
    }
    return p->value;
}

// Internals
static GPIO* findGPIO(int8_t p) {
    // take MUTEX
    os_mutex_pend(&_gpiomutex, OS_TIMEOUT_NEVER);
    for(int i=0;i<MAX_GPIOS;i++) {
        if (_gpios[i].pin==p) {
            os_mutex_release(&_gpiomutex);
            return &_gpios[i];
        }
    }
    os_mutex_release(&_gpiomutex);
    return NULL;
}
static GPIO* allocGPIO(int8_t p) {
    // take MUTEX
    os_mutex_pend(&_gpiomutex, OS_TIMEOUT_NEVER);
    if (findGPIO(p)==NULL) {
        for(int i=0;i<MAX_GPIOS;i++) {
            if (_gpios[i].pin<0) {
                _gpios[i].pin = p;      // Mine now
                // release mutex
                os_mutex_release(&_gpiomutex);
                return &_gpios[i];
            }
        }
    }
    // release mutex
    os_mutex_release(&_gpiomutex);
    return NULL;    
}

// release the slot
static void releaseGPIO(GPIO* g) {
    assert(g!=NULL);
    // take MUTEX
    os_mutex_pend(&_gpiomutex, OS_TIMEOUT_NEVER);
    g->pin = -1;
    // release mutex
    os_mutex_release(&_gpiomutex);
}

// if no adc gpios, then deinit the adc to save power
static void checkForNoADC() {
    // take MUTEX
    os_mutex_pend(&_gpiomutex, OS_TIMEOUT_NEVER);
    for(int i=0;i<MAX_GPIOS;i++) {
        if (_gpios[i].pin>=0 && _gpios[i].type==GPIO_ADC && _gpios[i].lpEnabled==true) {
            // still one ADC that is active, stop looking coz ADC stays on
            // release mutex
            os_mutex_release(&_gpiomutex);
            return;
        }
    }
    // release mutex
    os_mutex_release(&_gpiomutex);
    // and got here so deinit adc subsystem
    hal_bsp_adc_deinit();
}

// setup pin in hal (at init or when coming back from low power)
static void init_hal(GPIO* p) {
    // if not active then noop
    if (p->lpEnabled) {
        switch (p->type) {
            case GPIO_OUT: {
                hal_gpio_init_out(p->pin, p->value);
                break;
            }
            case GPIO_IN: {
                hal_gpio_init_in(p->pin, p->pull);
                break;
            }
            case GPIO_IRQ: {
                hal_gpio_irq_init(p->pin, p->handler, p->arg, p->trig, p->pull);
                // If IRQ was enabled for it, enable it again
                if (p->irqEn) {
                    hal_gpio_irq_enable(p->pin);
                }
                break;
            }
            case GPIO_ADC: {
                // Setup ADC for simple polling use if not already done (can be called multiple times without issues)
                hal_bsp_adc_init();
                hal_bsp_adc_define(p->pin, p->adc_chan);
                break;
            }
            default: {
                break;
            }
        }
    }
}
// deinit pin in hal (when no longer used or when entering low power)
static void deinit_hal(GPIO* p) {
    // check it is currently active, noop if not
    if (p->lpEnabled) {
        // irq or adc pins require specific release actions first
        if (p->type==GPIO_IRQ) {
            // If IRQ is enabled, disable it while its in low power
            if (p->irqEn) {
                hal_gpio_irq_disable(p->pin);
            }
            hal_gpio_irq_release(p->pin);
        } else if (p->type==GPIO_ADC) {
            hal_bsp_adc_release(p->pin, p->adc_chan);
            checkForNoADC();
        }
        // and deinit will set it to floating for lowest power use
        hal_gpio_deinit(p->pin);
    }
}

// Callback from LP manager
static void onLPModeChange(LP_MODE current, LP_MODE next) {
    os_mutex_pend(&_gpiomutex, OS_TIMEOUT_NEVER);
    for(int i=0;i<MAX_GPIOS;i++) {
        if (_gpios[i].pin>=0) {
            // lp modes are in increasing order of low powerness, so if next one is > that the one for this pin it must shut down
            if (next > _gpios[i].lpmode) {
                // deconfigure all pins that are off in the new mode (including irq disable)
                deinit_hal(&_gpios[i]);
                _gpios[i].lpEnabled = false;
            } else {
                // confgure all pins that are on in this mode (including irq enable)
                // but only if wans't already enabled
                if (_gpios[i].lpEnabled==false) {
                    _gpios[i].lpEnabled = true;
                    init_hal(&_gpios[i]);
                }
            }
        }
    }
    // release mutex
    os_mutex_release(&_gpiomutex);

}

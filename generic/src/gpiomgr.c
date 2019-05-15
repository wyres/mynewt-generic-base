/**
 Wyres packages
 * gpiomgr : this wraps the basic hal gpio calls with a concept of LPMODE, which defines the power modes of the MCU
 * It will then interface to a power manager, and auto deconfig/reconfig each gpio as the mode changes.
 */

#include <string.h>

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "syscfg/syscfg.h"

#include "wyres-generic/wutils.h"

#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/lowpowermgr.h"

#define MAX_GPIOS (MYNEWT_VAL(MAX_GPIOS))

typedef struct gpio {
    int8_t pin;
    GPIO_TYPE type;
    uint8_t value;
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
        hal_gpio_init_out(pin, p->value);
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
        hal_gpio_init_in(pin, pull);
        p->value = hal_gpio_read(pin);
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
        p->irqEn = true;
        p->pull = pull;
        p->trig = trig;
        p->lpmode = offmode;
        p->lpEnabled = true;        // assume pin is alive in current lp mode!
        hal_gpio_irq_init(pin, handler, arg, trig, pull);
        p->value = hal_gpio_read(pin);
    }
    return p;

}
void GPIO_release(int8_t pin) {
    GPIO* p = findGPIO(pin);
    if(p!=NULL) {
        // Only de-init if enabled in current LP mode as otherwise we ALREADY de-inited it to enter this LP mode...
        if (p->lpEnabled) {
            if (p->type==GPIO_IRQ) {
                hal_gpio_irq_release(p->pin);
            }
            hal_gpio_deinit(p->pin);
        }
        releaseGPIO(p);
    } // ignore if no such pin
}


void GPIO_irq_enable(int8_t pin) {
    GPIO* p = findGPIO(pin);
    assert(p!=NULL);
    assert(p->type==GPIO_IRQ);
    p->irqEn = 1;
    if (p->lpEnabled) {
        hal_gpio_irq_enable(p->pin);
    }
}
void GPIO_irq_disable(int8_t pin) {
    GPIO* p = findGPIO(pin);
    assert(p!=NULL);
    assert(p->type==GPIO_IRQ);
    p->irqEn = 0;
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

static void releaseGPIO(GPIO* g) {
    assert(g!=NULL);
    // take MUTEX
    os_mutex_pend(&_gpiomutex, OS_TIMEOUT_NEVER);
    hal_gpio_deinit(g->pin);
    g->pin = -1;
    // release mutex
    os_mutex_release(&_gpiomutex);
}

// Callback from LP manager
static void onLPModeChange(LP_MODE current, LP_MODE next) {
    // TODO 
    // deconfigure all pins that are off in this mode (including irq disable)
    // confgure all pins that are on in this mode (including irq enable)
}

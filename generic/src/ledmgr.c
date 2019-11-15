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


#include <string.h>

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#include "wyres-generic/wutils.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/ledmgr.h"

#define MAX_LEDS    MYNEWT_VAL(MAX_LEDS)

// Led task should be high pri as does very little but wants to do it in real time
#define LED_TASK_PRIO       MYNEWT_VAL(LEDMGR_TASK_PRIO)
#define LED_TASK_STACK_SZ   OS_STACK_ALIGN(64)

// internals
#define ISSET(v, p) ((v & (1<<p))!=0)
#define SETBIT(v, p) (v|=(1<<p))

#define MAX_PATTERN_SECS (2)
#define LED_SLICES_PER_SEC (10)

struct s_req {
    uint32_t pattern;
    uint32_t durSecs;
};
struct s_ledref {
    int8_t gpio;
    bool active;
    struct s_req cur;
    struct s_req next;
    struct os_callout durTimer;       // For the duration timer for this specific led
};

static os_stack_t _led_task_stack[LED_TASK_STACK_SZ];
static struct os_task _led_task_str;

static struct s_ledref _leds[MAX_LEDS];     // TODO use mempools
static uint8_t _ledRefsSz = 0;
static struct os_sem _ledActiveSema;

// predefine private fns
static int checkLED(int8_t gpio);
static void led_dur_ev_cb(struct os_event *ev);
static void startLEDTimer(int ledref);
static void stopLEDTimer(int ledref);
static int findLEDRef(int8_t gpio_pin);
static uint32_t makeBinaryFromString(const char* binary);
static void led_mgr_task(void* arg);
static void ledActive();
static void lp_change(LP_MODE oldmode, LP_MODE newmode);

// Called from sysinit via reference in pkg.yml
void led_mgr_init(void) {
    // Initialise the 'active led requests' semaphone with 0 (ie no requests)
    os_sem_init(&_ledActiveSema,0);
    // Create the led handler task
    os_task_init(&_led_task_str, "led_task", &led_mgr_task, NULL, LED_TASK_PRIO,
                 OS_WAIT_FOREVER, _led_task_stack, LED_TASK_STACK_SZ);
    // listen for lowpower entry to deep sleep - we cancel running leds when this happens
    LPMgr_register(lp_change);
}

// Public API
/*
 * Submit a request to flash a specific 'pattern' (expressed in 1/0s) on the given LED pin, for 'dur' seconds
 * 'pri' indicates if the request should be queued after any currently executing pattern, or interrupt it immediately
 * The return indicates if the request was accepted or not
 */
bool ledRequest(int8_t gpio, const char* pattern, uint32_t dur, LED_PRI pri) {
    // Convert gpio to index (find existing slot or creates one)
    int r = checkLED(gpio);
    if (r<0) {
        // No more slots
        return false;
    }
    if (pri==LED_REQ_INTERUPT) {
        _leds[r].cur.pattern = makeBinaryFromString(pattern);
        _leds[r].cur.durSecs = dur;
        _leds[r].next.pattern = 0;
    } else {
        // Not priority, must 'queue'
        if (_leds[r].active && _leds[r].cur.pattern!=0) {
            if (_leds[r].next.pattern!=0) {
                // queue full, sorry
                return false;
            } else {
                _leds[r].next.pattern = makeBinaryFromString(pattern);
                _leds[r].next.durSecs = dur;
                return true;        // no need to deal with currently executing request
            }
        } else {
            _leds[r].cur.pattern = makeBinaryFromString(pattern);
            _leds[r].cur.durSecs = dur;
        }
    }
    // new current request, start it
    _leds[r].active=true;
    ledActive();
    // Start timer for end
    startLEDTimer(r);
    return true;
}
// Start a pattern immediately (same as interuppting)
bool ledStart(int8_t gpio, const char* pattern, uint32_t dur) {
    return ledRequest(gpio, pattern, dur, LED_REQ_INTERUPT);
}
/*
 * Cancel the currently executing request on the LED pin given. The LED will either pass to the next pattern on the
 * queue or be off
 */
void ledCancel(int8_t gpio) {
    int r = findLEDRef(gpio);
    assert(r>=0);       // Shouldnt be cancelling a non-existant LED!
    // Cancel current timer
    stopLEDTimer(r);
    _leds[r].cur.pattern = _leds[r].next.pattern;
    _leds[r].cur.durSecs = _leds[r].next.durSecs;
    _leds[r].next.pattern = 0;
    if (_leds[r].cur.pattern!=0) {
        _leds[r].active=true;
        ledActive();
        // Start timer for end
        startLEDTimer(r);
    } else {
        _leds[r].active=false;
        GPIO_write(_leds[r].gpio, 0);

    }
}

// privates
// THere is at least 1 led request active, ensure task is awake
static void ledActive() {
    // Ensure task is awake by giving it a token on the sema (only if there aren't any)
    if (os_sem_get_count(&_ledActiveSema)<1) {
        os_sem_release(&_ledActiveSema);
    }
}

// Create led control 
static int checkLED(int8_t gpio) {
    int r = findLEDRef(gpio);
    if (r<0) {
        // Not yet created, make it
        // are we at limit?
        if (_ledRefsSz>=MAX_LEDS) {
            return -1;      // sorry
        }
        // get index to return and inc ready for next time
        r = _ledRefsSz++;

        // fill in the next slot and return its index as reference
        _leds[r].gpio = gpio;
        _leds[r].active = false;
        _leds[r].cur.pattern = 0;
        _leds[r].next.pattern = 0;
        // Setup io : using IO mgr to deal with deep sleep entry/exit. 
        GPIO_define_out("LED", gpio, 0, LP_DOZE);
        // Each entry has its own timer, where the arg in the event for the timer callback is the gpio value...
        os_callout_init(&(_leds[r].durTimer), os_eventq_dflt_get(),
                    &led_dur_ev_cb, (void*)(&_leds[r]));

        //log("created new LED with gpio pin %d", gpio);
    }
    return r;
}

static uint32_t makeBinaryFromString(const char* binary) {
    if (binary==NULL) {
        return 0;
    }
    uint32_t ret = 0;
    for(int i=0;i<(MAX_PATTERN_SECS*LED_SLICES_PER_SEC);i++) {
        // String may be shorter than 20 elements but effectively this pads 0s to right
        if (i<strlen(binary) && binary[i]=='1') {
            SETBIT(ret, i);
        }
    }
    return ret;
}

static void led_mgr_task(void* arg) {
    // Handle a list of requests for specific led blink pattern on a specific led to be started/stopped
    // these requests are dealt with in the task below in order

    int timeslice = 0;

    while (1) {
        timeslice = (timeslice+1) % (MAX_PATTERN_SECS*LED_SLICES_PER_SEC);       // 100ms slice within the 2 seconds
        int nleds = _ledRefsSz;
        bool patternActive = false;
        for (int i=0; i<nleds;i++) {
            // Only write if active pattern. 
            if (_leds[i].active) {
                patternActive = true;       // at least one pattern is running
                // get current pattern for this led and get if high or low
                // set led on or off as required
                if (ISSET(_leds[i].cur.pattern, timeslice)) {
                    GPIO_write(_leds[i].gpio, 1);
                } else {
                    GPIO_write(_leds[i].gpio, 0);
                }
            }
        }
        // wait 100ms to change led states IFF led pattern running, else wait on semaphore for new request
        if (patternActive) {
            /* Wait 100ms (or 1second divided by the number of time slices per sec) - this yields the CPU */
            os_time_delay(OS_TICKS_PER_SEC/LED_SLICES_PER_SEC);
        } else {
            // wait until a new led request arrives (so can sleep)
            os_sem_pend(&_ledActiveSema, OS_TIMEOUT_NEVER);
//            os_time_delay(OS_TICKS_PER_SEC/LED_SLICES_PER_SEC);
        }
    }
}

// callout (timer) event callback
static void led_dur_ev_cb(struct os_event *ev) {
    // just 'cancel' current led request (which is referenced by the ev_arg of the event structure...)
    assert(ev!=NULL);
    assert(ev->ev_arg!=NULL);
    int8_t gpio = ((struct s_ledref *)(ev->ev_arg))->gpio;
    ledCancel(gpio);
}
static void startLEDTimer(int ledref) {
    assert(ledref>=0 && ledref<_ledRefsSz);
    // timer runs if value is >0, else no duration timer for this guy.... caller must cancel it explcitly or interuppt with another request
    if (_leds[ledref].cur.durSecs>0) {
        os_callout_reset(&_leds[ledref].durTimer, _leds[ledref].cur.durSecs * OS_TICKS_PER_SEC);
    } else {
        os_callout_stop(&_leds[ledref].durTimer);
    }
}
static void stopLEDTimer(int ledref) {
    assert(ledref>=0 && ledref<_ledRefsSz);
    os_callout_stop(&_leds[ledref].durTimer);
}

static int findLEDRef(int8_t gpio_pin) {
    for (int i=0; i<_ledRefsSz;i++) {
        if (_leds[i].gpio == gpio_pin) {
            return i;       // good to go
        }
    }
    return -1;
}
// LOw power mode change
static void lp_change(LP_MODE oldmode, LP_MODE newmode) {
    if (newmode>=LP_DEEPSLEEP) {
        log_debug("LM:sleep");
        // stop any running LEDs
        for(int r=0;r<_ledRefsSz;r++) {
            _leds[r].active=false;
            // Ensure no timer running
            stopLEDTimer(r);
        }
    } else {
        log_debug("LM:wake");
    }
}


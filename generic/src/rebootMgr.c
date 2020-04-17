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
 * Manage reboot : saving logs, code passage ids etc into flash, indicating reset reasons etc
 */
#include "os/os.h"

#include "hal/hal_system.h"
#include "hal/hal_watchdog.h"
#include "mcu/stm32_hal.h"
#include "bsp/bsp.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/configmgr.h"
#include "wyres-generic/rebootmgr.h"
#include "wyres-generic/timemgr.h"

// The max watchdog timeout this MCU can support
// 28s for STM32
#define MAX_MCU_WATCHDOG_SECS (28)          

// Led task should be high pri as does very little but wants to do it in real time
#define WATCHDOG_TASK_PRIO       MYNEWT_VAL(WATCHDOG_TASK_PRIO)
#define WATCHDOG_TASK_STACK_SZ   OS_STACK_ALIGN(64)
// Select watchog timeout based on requested type
#define WATCHDOG_TIMEOUT_SECS   (MYNEWT_VAL(WATCHDOG_TIMEOUT_TYPE)==0?\
                                    0:(MYNEWT_VAL(WATCHDOG_TIMEOUT_TYPE)==1?\
                                    (MAX_MCU_WATCHDOG_SECS-1):MYNEWT_VAL(WATCHDOG_TIMEOUT_MINS)*60))

#define REBOOT_LIST_SZ  (8)
#define FN_LIST_SZ  (8*8)   // for 8 entries, each of 2 uint32_ts

static char* _resetReason="AP/M\0";
static uint8_t _appReasonCode = 0;
// reboot reason codes + last byte for index
static uint8_t _rebootReasonList[REBOOT_LIST_SZ+1];
// fn call log + last byte for index
static uint8_t _fnList[FN_LIST_SZ+1];

static void* _assertCallerFn = 0;

static enum RM_reason mapHalResetCause() {
    switch(hal_reset_cause()) {
        case HAL_RESET_POR:
            return RM_HARD_RESET;
        case HAL_RESET_BROWNOUT:
            return RM_HW_BROWNOUT;
        case HAL_RESET_PIN:
            return RM_HW_NRST;
        case HAL_RESET_WATCHDOG:
            return RM_HW_WG;
        case HAL_RESET_SOFT:
            return RM_HW_BLAMES_SW;
        default:
            return RM_HARD_RESET;
    }
}

// run at startup
void reboot_init(void) {
    // get reboot reason from PROM
    // Its a circular list, last byte is current index
    memset(_rebootReasonList, 0, REBOOT_LIST_SZ+1);
    CFMgr_getOrAddElement(CFG_UTIL_KEY_REBOOTREASON, &_rebootReasonList, REBOOT_LIST_SZ+1);
    if (_rebootReasonList[REBOOT_LIST_SZ]>REBOOT_LIST_SZ) {
        log_noout("reboot list index is bad %d", _rebootReasonList[REBOOT_LIST_SZ]);
        _rebootReasonList[REBOOT_LIST_SZ] = 0;
    }
    // If last reboot due to hw reset (ie didn't go via the RMMgr_reboot() call), then ask HAL for more details
    if (_rebootReasonList[_rebootReasonList[REBOOT_LIST_SZ]]==RM_HARD_RESET) {
        _rebootReasonList[_rebootReasonList[REBOOT_LIST_SZ]] = mapHalResetCause();
    }
    // Store the last restart reason in separate var for ease of use
    _appReasonCode = _rebootReasonList[_rebootReasonList[REBOOT_LIST_SZ]];
    // inc index to next position ready for next restart
    _rebootReasonList[REBOOT_LIST_SZ] = ((_rebootReasonList[REBOOT_LIST_SZ]+1) % REBOOT_LIST_SZ);
    // Set the next reboot reason to hard reset (in case it is as we won't go via reboot function!)
    _rebootReasonList[_rebootReasonList[REBOOT_LIST_SZ]] = RM_HARD_RESET;
    // Update PROM
    CFMgr_setElement(CFG_UTIL_KEY_REBOOTREASON, &_rebootReasonList, REBOOT_LIST_SZ+1);

    // Get last assert caller
    CFMgr_getOrAddElement(CFG_UTIL_KEY_ASSERTCALLERFN, &_assertCallerFn, 4);

    // fn logging list (should have been stored at reboot...)
    CFMgr_getOrAddElement(CFG_UTIL_KEY_FN_LIST, &_fnList, FN_LIST_SZ+1);

    // fixup reason string
    _resetReason[0] = '0'+((_appReasonCode / 10)%10);
    _resetReason[1] = '0'+(_appReasonCode % 10);
    // From hal
    _resetReason[3] = '0'+(hal_reset_cause() %10);

    // If rebooted to enter stock mode (ie no watchdog), then do it
    if (_appReasonCode==RM_ENTER_STOCK_MODE) {
        // Normally noone is initialised yet, so we don't need to cleanup
        hal_bsp_halt();
        assert(0);      // not returning
    }
    // If OS not doing watchdog then enable our management of it
#if MYNEWT_VAL(WATCHDOG_INTERVAL) == 0
    RMMgr_watchdog_init(WATCHDOG_TIMEOUT_SECS);
#endif
}

void RMMgr_reboot(uint8_t reason) {
    // Store reason in PROM
    _rebootReasonList[_rebootReasonList[REBOOT_LIST_SZ]] = reason;
    // Update PROM
    CFMgr_setElement(CFG_UTIL_KEY_REBOOTREASON, &_rebootReasonList, REBOOT_LIST_SZ+1);

    // store fn tracker buffer in PROM/FLASH
    CFMgr_setElement(CFG_UTIL_KEY_FN_LIST, &_fnList, FN_LIST_SZ+1);

    hal_system_reset();
}
const char* RMMgr_getResetReason() {
    return _resetReason;
}
uint16_t RMMgr_getResetReasonCode() {
    // Recovered from PROM at boot time
    return _appReasonCode + (hal_reset_cause() << 8);
}
void RMMgr_getResetReasonBuffer(uint8_t* buf, uint8_t sz) {
    for(int i=0; i<sz; i++) {
        buf[i] = _rebootReasonList[(_rebootReasonList[REBOOT_LIST_SZ]-(i+1) + REBOOT_LIST_SZ) % REBOOT_LIST_SZ];
    }
}

bool RMMgr_wasHardReset() {
    return (_appReasonCode == RM_HARD_RESET);
}

void RMMgr_saveAssertCaller(void* fnaddr) {
    _assertCallerFn = fnaddr;
    CFMgr_setElement(CFG_UTIL_KEY_ASSERTCALLERFN, &_assertCallerFn, 4);

}
void* RMMgr_getLastAssertCallerFn() {
    return _assertCallerFn;
}
typedef struct {
        uint32_t ts;
        void* caller;
} FNP_LOG_t;

// Log passage in a fn by recording its address for later analysis
void RMMgr_addLogFn(void* caller) {
    // add to circular list along with timestamp
    // LAST byte of block is index to next free entry in the block
        // Calculate current (empty) slot offset
    uint8_t index = _fnList[FN_LIST_SZ];
    if (index>=FN_LIST_SZ) {
        index = 0;      // reset if we find it to be bad
        _fnList[FN_LIST_SZ] = 0;
    }

    FNP_LOG_t* entry = (FNP_LOG_t *)&_fnList[index];
    entry->ts = TMMgr_getRelTimeMS();
    entry->caller = caller;
    // update next free index
    _fnList[FN_LIST_SZ] = ((index+sizeof(FNP_LOG_t)) % FN_LIST_SZ);
    // This is written to PROM if reboot called
}

// Get specific log fn entry from the log list of fns 
void* RMMgr_getLogFn(uint8_t offset) {
    if (offset>FN_LIST_SZ/sizeof(FNP_LOG_t)) {
        return NULL;
    }
    // Calculate current (empty) slot offset
    uint8_t index = _fnList[FN_LIST_SZ];
    if (index>=FN_LIST_SZ) {
        index = 0;      // reset if we find it to be bad
        _fnList[FN_LIST_SZ] = 0;
    }
    // GO back by 'offset+1' items (looping round if required)
    index = (index-((offset+1)*sizeof(FNP_LOG_t)))%(FN_LIST_SZ);

    FNP_LOG_t* entry = (FNP_LOG_t *)&_fnList[index];
    return entry->caller;
}


// application or hal level watchdog and task management
static struct {
    os_stack_t watchdog_task_stack[WATCHDOG_TASK_STACK_SZ];
    struct os_task watchdog_task_str;
    uint32_t    timeoutMS;
    uint32_t    last_tickleTS;
    struct os_callout timer;
    bool isEnabled;
} _awctx;

static void watchdog_task(void* arg) {
    while(1) {
        RMMgr_watchdog_tickle();
        // sleep for half watchdog timeout
        os_time_delay(os_time_ms_to_ticks32(_awctx.timeoutMS/2));
    }
}

static void app_watchdog_timeoutcb(struct os_event *d) {
    // are we enabled, and if so, did we get tickled recently?
    if (_awctx.isEnabled && (TMMgr_getRelTimeSecs()-_awctx.last_tickleTS) > _awctx.timeoutMS) {
        // no, rbye bye
        assert(0);
    }
    // yes, restart timer
    os_callout_reset(&_awctx.timer, os_time_ms_to_ticks32(_awctx.timeoutMS));
}
void RMMgr_watchdog_init(uint32_t timeoutsecs) {
    _awctx.isEnabled = false;
    _awctx.timeoutMS = timeoutsecs*1000;
    if (timeoutsecs==0) {
        return;     // 0 means disabled 
    }
    // Create task to tickle watchdog from default task
    os_task_init(&_awctx.watchdog_task_str, "watchdog", &watchdog_task, NULL, WATCHDOG_TASK_PRIO,
                 OS_WAIT_FOREVER, _awctx.watchdog_task_stack, WATCHDOG_TASK_STACK_SZ);
    // deal with watchdog here if OS not doing to (interval at 0 means app responsible)
    // If the watchdog is set to less than the MCU max we can use the MCU internal watchdog timer (which is very low level and robust)
    if (timeoutsecs < MAX_MCU_WATCHDOG_SECS) {
        // init watchdog
        hal_watchdog_init(_awctx.timeoutMS);       // 28s is the max...
    } else {
        os_callout_init(&_awctx.timer, os_eventq_dflt_get(), app_watchdog_timeoutcb, NULL);
        os_callout_reset(&_awctx.timer, os_time_ms_to_ticks32(_awctx.timeoutMS));
    }
    // and start it
    RMMgr_watchdog_enable();       
}
void RMMgr_watchdog_enable() {
    if (_awctx.timeoutMS==0) {
        return;         // ignore
    }
    // Are we using MCU or OS timer watchdogs?
    if (_awctx.timeoutMS < (MAX_MCU_WATCHDOG_SECS*1000)) {
        // start it
        hal_watchdog_enable();       
    } else {
        // Enable os timer watchdog
        _awctx.isEnabled = true;
    }
}
void RMMgr_watchdog_disable() {
    if (_awctx.timeoutMS==0) {
        return;         // ignore
    }
    // Are we using MCU or OS timer watchdogs?
    if (_awctx.timeoutMS < (MAX_MCU_WATCHDOG_SECS*1000)) {
        // oops cant do that - ensure we know...
        assert(0);
    } else {
        _awctx.isEnabled = false;
    }
}
/** update watchdog */
void RMMgr_watchdog_tickle() {
    if (_awctx.timeoutMS==0) {
        return;         // ignore
    }
    // Are we using MCU or OS timer watchdogs?
    if (_awctx.timeoutMS < (MAX_MCU_WATCHDOG_SECS*1000)) {
        hal_watchdog_tickle();
    } else {
        _awctx.last_tickleTS = TMMgr_getRelTimeSecs();
    }
}

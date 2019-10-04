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

#include "wyres-generic/wutils.h"
#include "wyres-generic/configmgr.h"
#include "wyres-generic/rebootMgr.h"
#include "wyres-generic/timemgr.h"

// Led task should be high pri as does very little but wants to do it in real time
#define WATCHDOG_TASK_PRIO       MYNEWT_VAL(WATCHDOG_TASK_PRIO)
#define WATCHDOG_TASK_STACK_SZ   OS_STACK_ALIGN(32)

#define REBOOT_LIST_SZ  (8)
#define FN_LIST_SZ  (8*8)   // for 8 entries, each of 2 uint32_ts

static void watchdog_task(void* arg);

static os_stack_t _watchdog_task_stack[WATCHDOG_TASK_STACK_SZ];
static struct os_task _watchdog_task_str;

static char* _resetReason="AP/M\0";
static uint8_t _appReasonCode = 0;
// reboot reason codes + last byte for index
static uint8_t _rebootReasonList[REBOOT_LIST_SZ+1];
// fn call log + last byte for index
static uint8_t _fnList[FN_LIST_SZ+1];

static void* _assertCallerFn = 0;

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
    // and reset so we know if reboot due to reset?

    // fixup reason string
    _resetReason[0] = '0'+((_appReasonCode / 10)%10);
    _resetReason[1] = '0'+(_appReasonCode % 10);
    // From hal
    _resetReason[3] = '0'+(hal_reset_cause() %10);

    // Create task to tickle watchdog from default task
    os_task_init(&_watchdog_task_str, "watchdog", &watchdog_task, NULL, WATCHDOG_TASK_PRIO,
                 OS_WAIT_FOREVER, _watchdog_task_stack, WATCHDOG_TASK_STACK_SZ);

    // init watchdog
//    hal_watchdog_init(5*60*1000);       // 5 minutes
    // and start it
//    hal_watchdog_enable();       
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
    // hal reset causes for STM32L1:
    /*
    if (reg & RCC_CSR_WWDGRSTF) {
        reason = HAL_RESET_WATCHDOG;            // 3
    } else if (reg & RCC_CSR_SFTRSTF) {
        reason = HAL_RESET_SOFT;                // 4
    } else if (reg & RCC_CSR_PINRSTF) {
        reason = HAL_RESET_PIN;                 // 2
    } else if (reg & RCC_CSR_LPWRRSTF) {
        // For L1xx this is low-power reset 
        reason = HAL_RESET_BROWNOUT;            // 5
    } else {
        reason = HAL_RESET_POR;                 // 1
    */
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

// Log passage in a fn by recording its address for later analysis
void log_fn_fn() {
    void* caller = __builtin_extract_return_addr(__builtin_return_address(0));
    // add to PROM based circular list along with timestamp
    *((uint32_t*)&_fnList[_fnList[FN_LIST_SZ]]) = TMMgr_getRelTime();
    *((uint32_t*)&_fnList[_fnList[FN_LIST_SZ]+4]) = (uint32_t)caller;
    _fnList[FN_LIST_SZ] = ((_fnList[FN_LIST_SZ]+8) % FN_LIST_SZ);
}

static void watchdog_task(void* arg) {
    while(1) {
        hal_watchdog_tickle();
        // sleep for 1 minute
        os_time_delay(OS_TICKS_PER_SEC*60);
    }
}

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
 * Low power manager. Used to change power modes
 * Can register callbacks to inform of changes to mode, eg GPIO mgr will do this to disable GPIOS etc
 */
#include "os/os.h"

#include "wyres-generic/wutils.h"

#include "wyres-generic/lowpowermgr.h"

#define MAX_LPCBFNS MYNEWT_VAL(MAX_LPCBFNS)

// Registered callbacks fns
static struct lp_ctx {
    LP_CBFN_t devices[MAX_LPCBFNS];        // TODO should be a mempool
    uint8_t deviceCnt;
    LP_MODE sleepMode;
} _ctx = {
    .deviceCnt=0,
    .sleepMode=LP_SLEEP,
};

// Initialise low power manager
void LPMgr_init(void) {
    // Hook OS enter/exit of WFI???
    // TODO
}

// Register to be told when we change mode
void LPMgr_register(LP_CBFN_t cb) {
    assert(_ctx.deviceCnt < MAX_LPCBFNS);
    _ctx.devices[_ctx.deviceCnt++]=cb;
}
// THe level of sleeping when someone (OS) asks to enter 'low power mode'
void LPMgr_setLPMode(LP_MODE m) {
    _ctx.sleepMode = m;
    // Set up MCU ready for a WKI instruction (for OS that doesn't call LPMgr_sleep())
    // eg    os_hookwfi(LPMgr_entersleep, LPMgr_exitsleep);
}

// Explicit request to sleep
// goto sleep until either an IRQ/timer (if enabled in the mode used) or (max) the wakeupTimeoutMs time has passed
void LPMgr_sleep(uint32_t wakeupTimeoutMs) {
    // tell all registered CBs we sleep (and at what level)
    for(int i=0;i<_ctx.deviceCnt;i++) {
        (*_ctx.devices[i])(LP_RUN, _ctx.sleepMode);
    }
    // SLEEP TODO
    // Should call fns in bsp really as this is board/mcu dependant
    // see    LPM_Enter() called from main.c in old code, code in boards/SK-iM880C/lowpower_board.c
    // WAKEUP
    // Tell everyone who cares
    for(int i=0;i<_ctx.deviceCnt;i++) {
        (*_ctx.devices[i])(_ctx.sleepMode, LP_RUN);
    }
}

// For OS that has hooks round WFI
void LPMgr_entersleep() {
    // tell all registered CBs we sleep (and at what level)
    for(int i=0;i<_ctx.deviceCnt;i++) {
        (*_ctx.devices[i])(LP_RUN, _ctx.sleepMode);
    }
}
void LPMgr_exitsleep() {
    // Tell everyone who cares
    for(int i=0;i<_ctx.deviceCnt;i++) {
        (*_ctx.devices[i])(_ctx.sleepMode, LP_RUN);
    }
}


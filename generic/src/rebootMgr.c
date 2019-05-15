/**
 * Wyres private code
 * Manage reboot : saving logs, code passage ids etc into flash, indicating reset reasons etc
 */
#include "os/os.h"

#include "hal/hal_system.h"
#include "hal/hal_watchdog.h"
#include "mcu/stm32_hal.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/configmgr.h"
#include "wyres-generic/rebootMgr.h"

// Led task should be high pri as does very little but wants to do it in real time
#define WATCHDOG_TASK_PRIO       MYNEWT_VAL(WATCHDOG_TASK_PRIO)
#define WATCHDOG_TASK_STACK_SZ   OS_STACK_ALIGN(32)

static void watchdog_task(void* arg);

static os_stack_t _watchdog_task_stack[WATCHDOG_TASK_STACK_SZ];
static struct os_task _watchdog_task_str;

static char* _resetReason="AP/M";
static uint8_t _appReasonCode = 0;
static uint32_t _assertCallerLoc = 0;
static void* _assertCallerFn = 0;

// run at startup
void reboot_init(void) {
    // get reboot reason from PROM
    _appReasonCode = 0;
    CFMgr_getOrAddElement(CFG_UTIL_KEY_REBOOTREASON, &_appReasonCode, 1);
    // and zero the PROM value so we known next time
    CFMgr_resetElement(CFG_UTIL_KEY_REBOOTREASON);
    // _appReasonCode = PROM_read(X);
    _resetReason[0] = '0'+((_appReasonCode / 10)%10);
    _resetReason[1] = '0'+(_appReasonCode % 10);
    // From hal
    _resetReason[3] = '0'+(hal_reset_cause() %10);

    CFMgr_getOrAddElement(CFG_UTIL_KEY_ASSERTCALLERLOC, &_assertCallerLoc, 4);
    CFMgr_getOrAddElement(CFG_UTIL_KEY_ASSERTCALLERFN, &_assertCallerFn, 4);

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
    CFMgr_setElement(CFG_UTIL_KEY_REBOOTREASON, &reason, 1);

    // store fn tracker buffer in PROM/FLASH

    // store stack trace in prom..
    hal_system_reset();
}
const char* RMMgr_getResetReason() {
    // Recovered from PROM at boot time
    return _resetReason;
}

void RMMgr_saveAssertCaller(void* fnaddr) {
    _assertCallerFn = fnaddr;
    CFMgr_setElement(CFG_UTIL_KEY_ASSERTCALLERFN, &_assertCallerFn, 4);

}
void* RMMgr_getLastAssertCallerFn() {
    return _assertCallerFn;
}

static void watchdog_task(void* arg) {
    while(1) {
        hal_watchdog_tickle();
        // sleep for 1 minute
        os_time_delay(OS_TICKS_PER_SEC*60);
    }
}

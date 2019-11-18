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
#include <hal/hal_bsp.h>

#include "wyres-generic/wutils.h"

#include "wyres-generic/lowpowermgr.h"

#define MAX_LPCBFNS MYNEWT_VAL(MAX_LPCBFNS)

// Registered callbacks fns
static struct lp_ctx {
    struct {
        LP_MODE_t desiredMode;  // Current sleep mode required
        LP_CBFN_t cb;           // callback to be informed of changes
    } lpUsers[MAX_LPCBFNS];
    uint8_t deviceCnt;
    LP_MODE_t sleepMode;
} _ctx = {
    .deviceCnt=0,
    .sleepMode=LP_SLEEP,
};
static LP_MODE_t calcNextSleepMode();
void os_hook_sleep(LP_HOOK_t enter, LP_HOOK_t exit);

// Initialise low power manager
void LPMgr_init(void) {
    // Hook OS enter/exit of WFI???
// See https://medium.com/@ly.lee/low-power-nb-iot-on-stm32-blue-pill-with-apache-mynewt-and-embedded-rust-cef5a3ecdd90
// need to override os_tick_init() and os_tick_idle() to use STM32 specific sleep mode and RTC (rather than tick timer)
/*
#  Linker flags: Rename all os_tick_init() and os_tick_idle() references to 
#  __wrap_os_tick_init() and __wrap_os_tick_idle(), so that we can provide a custom implementation
pkg.lflags:    
    - -Wl,-wrap,os_tick_init     
    - -Wl,-wrap,os_tick_idle
 */
    // Then os_tick_init() should use MCU specific tick method (eg RTC in STM32) and method of going to sleep (including selection of a sleep type)
    // This is a generic low power manager. So it delegates this stuff to BSP/MCU code...
    os_hook_sleep(LPMgr_entersleep, LPMgr_exitsleep);
}

// Register to be told when we change mode
LP_ID_t LPMgr_register(LP_CBFN_t cb) {
    assert(_ctx.deviceCnt < MAX_LPCBFNS);
    uint8_t id = _ctx.deviceCnt;
    _ctx.lpUsers[_ctx.deviceCnt].cb=cb;     // May be NULL if user only changes the desired LP mode...
    _ctx.lpUsers[_ctx.deviceCnt].desiredMode=LP_DEEPSLEEP;      // start by assuming everyone is ok with deep sleep
    _ctx.deviceCnt++;
    return id;
}
// THe level of sleeping when someone (the OS) asks to enter 'low power mode'
void LPMgr_setLPMode(LP_ID_t id, LP_MODE_t m) {
    assert(id>=0 && id < MAX_LPCBFNS);
    // Set this guy's desired mode
    _ctx.lpUsers[id].desiredMode = m;
    _ctx.sleepMode = calcNextSleepMode();
    // Taken into account next time we do LPMgr_entersleep()
}
LP_MODE_t LPMgr_getNextLPMode() {
    _ctx.sleepMode = calcNextSleepMode();
    return _ctx.sleepMode;
}
// Hook functions round the 'idle' method. These allow the code to pause/resume external hw or other tasks.
// Returns indication of level of sleep to use eg light sleep (1) or a deep sleep (2) or even a OFF mode (3)
// These should be the HAL_BSP_POWER_XXX defines from hap_power.h. The value is passed to the MUC specific 'sleep' 
// method hal_bsp_power_state() to setup the actual sleep.
int LPMgr_entersleep() {
    // tell all registered CBs we sleep (and at what level)
    for(int i=0;i<_ctx.deviceCnt;i++) {
        if (_ctx.lpUsers[i].cb!=NULL) {
            (*_ctx.lpUsers[i].cb)(LP_RUN, _ctx.sleepMode);
        }
    }
    if (_ctx.sleepMode == LP_OFF) {
        return HAL_BSP_POWER_OFF;       // Restart on RTC
    } else if (_ctx.sleepMode == LP_DEEPSLEEP) {
        return HAL_BSP_POWER_SLEEP;     // NOTE : we don't use HAL_BSP_POWER_DEEP_SLEEP as this implies RAM loss
    } else if (_ctx.sleepMode == LP_SLEEP) {
        return HAL_BSP_POWER_SLEEP;     // CPU + pêriphs paused
    } else {
        // LP_DOZE normally
        return HAL_BSP_POWER_WFI;       // Only CPU pause
    }
}
int LPMgr_exitsleep() {
    // Tell everyone who cares
    for(int i=0;i<_ctx.deviceCnt;i++) {
        if (_ctx.lpUsers[i].cb!=NULL) {
            (*_ctx.lpUsers[i].cb)(_ctx.sleepMode, LP_RUN);
        }
    }
    return HAL_BSP_POWER_ON;
}

// Internals
static LP_MODE_t calcNextSleepMode() {
    LP_MODE_t ret = LP_OFF;     // the deepest mode, man...
    for(int i=0;i<_ctx.deviceCnt;i++) {
        // check each user, anyone needing 'less' sleep is priority
        if (_ctx.lpUsers[i].desiredMode<ret) {
            ret = _ctx.lpUsers[i].desiredMode;
        }
    }
    return ret;
}

// TODO : all the code below is not yet operational, essentially coz the linker directives are not in place.
// ToBeTested
// in MCU specific code
static void MCU_init_alarm(uint32_t os_ticks_per_sec, uint32_t reload_val, int prio) {
#if 0
    //  Power on the RTC before using.
    rcc_enable_rtc_clock();
    rtc_interrupt_disable(RTC_SEC);
    rtc_interrupt_disable(RTC_ALR);
    rtc_interrupt_disable(RTC_OW);
    //  rtc_auto_awake() will not reset the RTC when you press the RST button.
    //  It will also continue to count while the MCU is held in reset. If
    //  you want it to reset, use rtc_awake_from_off()
    rtc_awake_from_off(clock_source);  //  This will enable RTC.
    rtc_set_prescale_val(prescale);

    //  Set the RTC time only at power on. Don't set it when waking from standby.
    rtc_set_counter_val(0);              //  Start counting millisecond ticks from 0
    rtc_set_alarm_time((uint32_t) -1);   //  Reset alarm to -1 or 0xffffffff so we don't trigger now
    exti_set_trigger(EXTI17, EXTI_TRIGGER_RISING);  //  Enable alarm wakeup via the interrupt
    exti_enable_request(EXTI17);

    NVIC_SetVector(RTC_IRQn,       (uint32_t) rtc_isr);        //  Set the Interrupt Service Routine for RTC
    NVIC_SetVector(RTC_Alarm_IRQn, (uint32_t) rtc_alarm_isr);  //  Set the Interrupt Service Routine for RTC Alarm
    
    nvic_enable_irq(NVIC_RTC_IRQ);        //  Enable RTC tick interrupt processing
    nvic_enable_irq(NVIC_RTC_ALARM_IRQ);  //  Enable RTC alarm wakeup interrupt processing
    __disable_irq();                      //  Disable interrupts while we make changes
    rtc_clear_flag(RTC_SEC);
    rtc_clear_flag(RTC_ALR);
    rtc_clear_flag(RTC_OW);
    rtc_interrupt_enable(RTC_ALR);        //  Allow RTC to generate alarm interrupts
    //  rtc_interrupt_enable(RTC_SEC);    //  Not used: Allow RTC to generate tick interrupts
    __enable_irq();                       //  Enable interrupts
#endif
}
static void MCU_set_alarm(uint32_t millisec) {
#if 0
    //  Set alarm for millisec milliseconds from now.
    volatile uint32_t now = rtc_get_counter_val();

    //  Not documented, but you must disable write protection else the alarm time will not be set and rtc_exit_config_mode() will hang.
    //  TODO: Disable only if write protection is enabled.
    pwr_disable_backup_domain_write_protect();
    rtc_set_alarm_time(now + millisec);
#endif
}

static void MCU_sleep(uint32_t timeMS, int hal_power_state) {
    if (timeMS < 10) { 
        //  no point in sleeping for <10 milliseconds
        // busy wait
        os_time_delay(os_time_ms_to_ticks32(timeMS));
        return;
    }

        //  Stop the system timer.  TODO: Start the timer after sleeping.
    NVIC_DisableIRQ(TIM2_IRQn);

    //  Set the RTC alarm to wake up in `timeMS` milliseconds from now.
    MCU_set_alarm(timeMS);

    // go into relevant power saving mode, to be woken by the alarm
    hal_bsp_power_state(hal_power_state);
}
// TODO MOVE to BSP
int hal_bsp_power_state(int state) {
    switch(state) {
        case HAL_BSP_POWER_OFF: {
            break;
        }
        case HAL_BSP_POWER_DEEP_SLEEP: {
            break;
        }
        case HAL_BSP_POWER_SLEEP: {
            break;
        }
        case HAL_BSP_POWER_WFI: {
            /* 
            _CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);    //  Clear SLEEPDEEP bit of Cortex System Control Register.
            _CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPONEXIT_Msk);  //  Clear SLEEPONEXIT bit of Cortex System Control Register.
            __DSB();
            __WFI();  //  Wait for interrupt from RTC Alarm.
            */
            break;
        }
        case HAL_BSP_POWER_ON:
        default: {
            break;
        }
    }
    return state;
}
// TODO this should be in the OS?
static LP_HOOK_t _hook_exit_cb=NULL;
static LP_HOOK_t _hook_enter_cb=NULL;

// OS feature : hook idle enter/exit phases. Note the hooks are call with irq disabled in OS critical section - so don't hang about
void os_hook_sleep(LP_HOOK_t enter, LP_HOOK_t exit) {
    // Should only have 1 hook of sleeping in the code, so assert if called twice
    assert(_hook_enter_cb==NULL);
    _hook_enter_cb = enter;
    _hook_exit_cb = exit;
}

//  This is what os_tick_idle() should be like ie MCU/BSP independant
void __wrap_os_tick_idle(os_time_t ticks) {
    OS_ASSERT_CRITICAL();
    //  Sleep for the number of ticks.
    uint32_t timeMS = os_time_ticks_to_ms32(ticks);
    
    // The hook tells us sleep type we want, or WFI if no hook
    int hal_power_state = _hook_enter_cb!=NULL?(*_hook_enter_cb)():HAL_BSP_POWER_WFI;
    // Do the sleeping - this should BLOCK and return in timeMS 
    MCU_sleep(timeMS, hal_power_state);

    //  Upon waking, sync the OS time.
// TODO    os_power_sync_time();
    // and tell hook
    if (_hook_exit_cb!=NULL) {
        (*_hook_exit_cb)();
    }
}

//  This intercepts all calls to os_tick_init()
void __wrap_os_tick_init(uint32_t os_ticks_per_sec, int prio)
{
    uint32_t reload_val;
    reload_val = ((uint64_t)SystemCoreClock / os_ticks_per_sec) - 1;
    //  Init the power management.
    MCU_init_alarm(os_ticks_per_sec, reload_val, prio);
}

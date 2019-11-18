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
#ifndef H_LOWPOWER_H
#define H_LOWPOWER_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Low power sleep modes, in order of increasing sleepness (ie lower power coz more stuff is off)
 * RUN : normal operation
 * DOZE : only CPU core is stopped for WFI (STM32L151CC 'Sleep mode')
 * SLEEP : CPU core + as many periphs off as possible (using info provided by the user of the periph) ('Low-power sleep mode')
 * DEEP_SLEEP : only RAM /RTC retained - all periphs are off. ('Stop mode with RTC')
 * OFF : everything off except RTC, wakeup via RTC or reset is via reboot ('Standby mode with RTC')
 */ 
typedef enum  { LP_RUN, LP_DOZE, LP_SLEEP, LP_DEEPSLEEP, LP_OFF } LP_MODE_t;
typedef uint8_t LP_ID_t;
typedef void (*LP_CBFN_t)(LP_MODE_t prevmode, LP_MODE_t newmode);
typedef int (*LP_HOOK_t)();

// Register as a low power actor, to be able to set desired power mode, and to be told when entering or leaving sleep
LP_ID_t  LPMgr_register(LP_CBFN_t cb);
// THe level of sleeping when someone asks to enter low power mode. Each actor sets the appropriate mode for their current operations,
// and the LPM takes the 'worst' case when doing sleeps
// TODO should there be a way to FORCE a lp mode (override other users of LPM?)
void LPMgr_setLPMode(LP_ID_t id, LP_MODE_t m);
// if OS wraps the enter/exit of sleep mode
int LPMgr_entersleep();
int LPMgr_exitsleep();

#ifdef __cplusplus
}
#endif

#endif  /* H_LOWPOWER_H */

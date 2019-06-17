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
 * DOZE : only CPU core is stopped (STM32L151CC 'Sleep mode')
 * SLEEP : CPU core + as many periphs off as possible (using info provided by the user of the periph) ('Low-power sleep mode')
 * DEEP_SLEEP : only RAM /RTC retained - all periphs are off. ('Stop mode with RTC')
 * OFF : all off, restart via RTC or reset ('Standby mode with RTC')
 */ 
typedef enum  { LP_RUN, LP_DOZE, LP_SLEEP, LP_DEEPSLEEP, LP_OFF } LP_MODE;
typedef void (*LP_CBFN_t)(LP_MODE prevmode, LP_MODE newmode);
// Register to be told when entering or leaving sleep
void LPMgr_register(LP_CBFN_t cb);
// THe level of sleeping when someone asks to enter low power mode
void LPMgr_setLPMode(LP_MODE m);
// goto sleep until either an IRQ/timer (if enabled in the mode used) or (max) the wakeupTimeoutMs time has passed
void LPMgr_sleep(uint32_t wakeupTimeoutMs);

#ifdef __cplusplus
}
#endif

#endif  /* H_LOWPOWER_H */

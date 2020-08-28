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
#ifndef H_TIMEMGR_H
#define H_TIMEMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
     uint16_t year;
     uint8_t month;
     uint8_t dayOfMonth;
     uint8_t hour24;
     uint8_t min;
     uint8_t sec;
     uint16_t ms;
} caltime_t;

/* time in ms since boot (warning, loops at 49 days) */
uint32_t TMMgr_getRelTimeMS();
/* time in ms since epoch as timed by the RTC */
uint64_t TMMgr_getRTCTimeMS();

const char *TMMgr_isoTimeString(caltime_t* timep);

/* time in secs since boot */
uint32_t TMMgr_getRelTimeSecs();
/* time in secs since epoch (if epoch relative boot time was set) */
uint32_t TMMgr_getTimeSecs();
/* set boot time in secs since epoch */
void TMMgr_setBootTime(uint32_t tSecsEpoch);
/* loop busily while time passes */
uint32_t TMMgr_busySleep(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif  /* H_TIMEMGR_H */

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
 * Time/Date management
 */
#include "os/os.h"

#include "wyres-generic/wutils.h"

#include "wyres-generic/timemgr.h"

extern uint64_t hal_rtc_getRTCTimeMS();     // In rtc_utils.c in specific hw/mcu/stm/stm32l1xx dir
extern void hal_rtc_getRTCTime(uint16_t* year, uint8_t* month, uint8_t* dayOfMonth, uint8_t* hour24, uint8_t* min, uint8_t* sec, uint16_t* ms);

static uint32_t _bootTimeSecs=0;

// Since boot (warning wraps after 49 days as is in ms)
uint32_t TMMgr_getRelTimeMS() {
    int64_t t = os_get_uptime_usec();
    return (uint32_t)(t/1000);
}

// since boot in secs (no real wrapping issue)
uint32_t TMMgr_getRelTimeSecs() {
    int64_t t = os_get_uptime_usec();
    return (uint32_t)(t/1000000);
}

/** get ms since epoch from RTC */
uint64_t TMMgr_getRTCTimeMS() {
    return hal_rtc_getRTCTimeMS();
}

static char _TIME[40];
static caltime_t _now;
const char* 
TMMgr_isoTimeString(caltime_t* timep) {
    caltime_t* t = timep;
    if (t==NULL) {
        hal_rtc_getRTCTime(&_now.year, &_now.month, &_now.dayOfMonth, &_now.hour24, &_now.min, &_now.sec, &_now.ms);
        t = &_now;
    }
    sprintf(_TIME, "%02d/%02d/%04d-%02d:%02d:%02d.%03d", t->dayOfMonth, t->month, t->year, t->hour24, t->min, t->sec, t->ms);
    return (const char* )&_TIME[0];
}

uint32_t TMMgr_getTimeSecs() {
    return _bootTimeSecs+TMMgr_getRelTimeSecs();
}

int32_t TMMgr_timeDelta(uint32_t t1MS, uint32_t t2MS) {
    // deal with fact that timestamps in ms since boot wrap after 49.7 days
    // TODO
    return t2MS - t1MS;
}

int32_t TMMgr_deltaNow(uint32_t tMS) {
    return TMMgr_timeDelta(TMMgr_getRelTimeMS(), tMS);
}

void TMMgr_setTimeSecs(uint32_t tSecsSinceEpoch) {
    // Adjust offset so that a get will get the right time
    _bootTimeSecs = (tSecsSinceEpoch - TMMgr_getRelTimeSecs());
}

uint32_t TMMgr_busySleep(uint32_t ms) {
    int64_t s = os_get_uptime_usec();
    uint32_t i = 0;
    while ((s+(ms*1000))>os_get_uptime_usec()) {
        i++;        // just to give it something to do
    }
    return ((os_get_uptime_usec() - s)/1000);
}
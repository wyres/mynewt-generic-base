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

uint32_t TMMgr_getTimeSecs() {
    return _bootTimeSecs+TMMgr_getRelTimeSecs();
}

int32_t TMMgr_timeDelta(uint32_t t1MS, uint32_t t2MS) {
    // deal with fact that timestamps in ms since boot wrap after 49.7 days

    return t2MS - t1MS;
}
int32_t TMMgr_deltaNow(uint32_t tMS) {
    return TMMgr_timeDelta(TMMgr_getRelTimeMS(), tMS);
}

void TMMgr_setBootTime(uint32_t tSecsSinceEpoch) {
    _bootTimeSecs = tSecsSinceEpoch;
}

uint32_t TMMgr_busySleep(uint32_t ms) {
    int64_t s = os_get_uptime_usec();
    uint32_t i = 0;
    while ((s+(ms*1000))>os_get_uptime_usec()) {
        i++;        // just to give it something to do
    }
    return ((os_get_uptime_usec() - s)/1000);
}
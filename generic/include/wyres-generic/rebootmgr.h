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
#ifndef H_REBOOTMGR_H
#define H_REBOOTMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

// Why did I reboot last time?
const char* RMMgr_getResetReason();
uint16_t RMMgr_getResetReasonCode();
// get list of last 'sz' entries from the reboot reason list, with latest first.
void RMMgr_getResetReasonBuffer(uint8_t* buf, uint8_t sz);
void RMMgr_reboot(uint8_t reason);
void RMMgr_saveAssertCaller(void* fnaddr);
uint32_t RMMgr_getLastAssertCallerLoc();
void* RMMgr_getLastAssertCallerFn();
bool RMMgr_wasHardReset();
void RMMgr_addLogFn(void* caller);
// Get a logged fn pointer (offset is the 'last but N' value)
void* RMMgr_getLogFn(uint8_t offset);

enum RM_reason { RM_HARD_RESET = 0, RM_ASSERT, RM_WATCHDOG, RM_RADIO_ERR, RM_HW_ERR, RM_DM_ACTION, RM_AT_ACTION, 
    RM_HW_BROWNOUT, RM_HW_WG, RM_HW_NRST, RM_HW_BLAMES_SW, RM_ENTER_STOCK_MODE };

#ifdef __cplusplus
}
#endif

#endif  /* H_REBOOTMGR_H */

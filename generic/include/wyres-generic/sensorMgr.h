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
#ifndef H_SENSORMGR_H
#define H_SENSORMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif


void SRMgr_start();
void SRMgr_stop();
uint32_t SRMgr_getTemp();
uint32_t SRMgr_getPressure();
uint32_t SRMgr_getBattery();
uint32_t SRMgr_getLight();
uint32_t SRMgr_getButtonMask();

#ifdef __cplusplus
}
#endif

#endif  /* H_SENSORMGR_H */

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
#ifndef H_CONFIGMGR_H
#define H_CONFIGMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CFGKEY(__m, __k) (((__m & 0xff) << 8) | (__k & 0xff))
typedef void (*CFG_CBFN_t)(uint16_t key);

bool CFMgr_registerCB(CFG_CBFN_t cb);
bool CFMgr_addElementDef(uint16_t key, uint8_t len, void* initdata);
bool CFMgr_getOrAddElement(uint16_t key, void* data, uint8_t len);
uint8_t CFMgr_getElementLen(uint16_t key);
bool CFMgr_setElement(uint16_t key, void* data, uint8_t len);
bool CFMgr_resetElement(uint16_t key);

// Define module ids here as unique values 1-255. Module 0 is for basic untilites (who can manage their keys between them..)
// NEVER REDEFINE A VALUE UNLESS OK TO CLEAR DEVICE CONFIG AFTER UPGRADE
#define CFG_MODULE_UTIL 0
#define CFG_MODULE_LORA 1
#define CFG_MODULE_APP 2
#define CFG_MODULE_WYRES 3
#define CFG_MODULE_APP_CORE 4
#define CFG_MODULE_APP_MOD 5

// Config keys used in the wyres generic utils - a key of 0 is never used
#define CFG_KEY_ILLEGAL (0)
#define CFG_UTIL_KEY_REBOOTREASON   CFGKEY(CFG_MODULE_UTIL, 1)
#define CFG_UTIL_KEY_FN_LIST        CFGKEY(CFG_MODULE_UTIL, 2)
#define CFG_UTIL_KEY_ASSERTCALLERFN CFGKEY(CFG_MODULE_UTIL, 3)

#ifdef __cplusplus
}
#endif

#endif  /* H_CONFIGMGR_H */

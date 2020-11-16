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

// Use this macro to define unique config key within your 'bloc'
#define CFGKEY(__m, __k) (((__m & 0xff) << 8) | (__k & 0xff))
typedef void (*CFG_CBFN_t)(void* ctx, uint16_t key);

bool CFMgr_registerCB(CFG_CBFN_t cb);
bool CFMgr_addElementDef(uint16_t key, uint8_t len, void* initdata);
/*
 * Get a config key value into *data, creating the entry if it does not exist using the *data value and the given length
 */
bool CFMgr_getOrAddElement(uint16_t key, void* data, uint8_t len);
/*
 * Helper getOrAdd calls for specific int types that check range
 */
bool CFMgr_getOrAddElementCheckRangeUINT32(uint16_t key, uint32_t* data, uint32_t min, uint32_t max);
bool CFMgr_getOrAddElementCheckRangeUINT8(uint16_t key, uint8_t* data, uint8_t min, uint8_t max);
bool CFMgr_getOrAddElementCheckRangeINT32(uint16_t key, int32_t* data, int32_t min, int32_t max);
bool CFMgr_getOrAddElementCheckRangeINT8(uint16_t key, int8_t* data, int8_t min, int8_t max);

/* 
 * get an element value, returning its length, into a buffer of size maxlen
 * returns -1 if key not found
 */
int CFMgr_getElement(uint16_t key, void* data, uint8_t maxlen);
uint8_t CFMgr_getElementLen(uint16_t key);
bool CFMgr_setElement(uint16_t key, void* data, uint8_t len);
bool CFMgr_resetElement(uint16_t key);
void CFMgr_iterateKeys(int keymodule, CFG_CBFN_t cb, void* cbctx);

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

//Config keys for movement manager
#define CFG_UTIL_KEY_ACCELERO_DETECTION_MODE     CFGKEY(CFG_MODULE_UTIL, 4)
#define CFG_UTIL_KEY_ACCELERO_SHOCK_THRESHOLD    CFGKEY(CFG_MODULE_UTIL, 5)
#define CFG_UTIL_KEY_ACCELERO_FREEFALL_THRESHOLD CFGKEY(CFG_MODULE_UTIL, 6)
#define CFG_UTIL_KEY_ACCELERO_SHOCK_DURATION     CFGKEY(CFG_MODULE_UTIL, 7)
#define CFG_UTIL_KEY_ACCELERO_FREEFALL_DURATION  CFGKEY(CFG_MODULE_UTIL, 8)

#define CFG_UTIL_KEY_REBOOT_TIME                 CFGKEY(CFG_MODULE_UTIL, 9)

#ifdef __cplusplus
}
#endif

#endif  /* H_CONFIGMGR_H */

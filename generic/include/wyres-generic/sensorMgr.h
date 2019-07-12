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

// Generic callback to signal something changed (what depends on registration!)
typedef void (*SR_CBFN_t)(void);
typedef enum { SR_BUTTON_RELEASED=0, SR_BUTTON_PRESSED=1} SR_BUTTON_STATE_t;
typedef enum { SR_BUTTON_SHORT, SR_BUTTON_MED, SR_BUTTON_LONG, SR_BUTTON_VLONG } SR_BUTTON_PRESS_TYPE_t;

void SRMgr_start();
void SRMgr_stop();

uint32_t SRMgr_getLastButtonTime();
// Register callback to be notified when button changes state (also means button is active during deep sleep)
bool SRMgr_registerButtonCB(SR_CBFN_t cb);
// Remove registration - if noone is registered then button state only checked at UL time..
void SRMgr_unregisterButtonCB(SR_CBFN_t cb);

uint32_t SRMgr_getLastNoiseTime();
uint8_t SRMgr_getNoiseFreqkHz();
uint8_t SRMgr_getNoiseLeveldB();
// Register callback to be notified when noise is detected (also means micro is active during deep sleep)
bool SRMgr_registerNoiseCB(SR_CBFN_t cb);
// Remove registration - if noone is registered then micro input only checked at UL time..
void SRMgr_unregisterNoiseCB(SR_CBFN_t cb);

// When did temp, pressure, battery, light last change 'significantly'?
uint32_t SRMgr_getLastEnvChangeTime();
int16_t SRMgr_getTempdC();
uint32_t SRMgr_getPressurePa();
uint16_t SRMgr_getBatterymV();
uint8_t SRMgr_getLight();
uint16_t SRMgr_getADC1mV();
uint16_t SRMgr_getADC2mV();
uint8_t SRMgr_getButton();
bool SRMgr_hasButtonChanged();
uint8_t SRMgr_getLastButtonPressType();
uint32_t SRMgr_getLastButtonPressTS();
uint32_t SRMgr_getLastButtonReleaseTS();
bool SRMgr_hasTempChanged();
bool SRMgr_hasPressureChanged();
bool SRMgr_hasBattChanged();
bool SRMgr_hasLightChanged();
bool SRMgr_hasADC1Changed();
bool SRMgr_hasADC2Changed();
bool SRMgr_updateEnvs();

#ifdef __cplusplus
}
#endif

#endif  /* H_SENSORMGR_H */

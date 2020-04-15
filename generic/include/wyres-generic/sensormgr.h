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
typedef enum { SR_BUTTON_RELEASED=0, SR_BUTTON_PRESSED=1} SR_BUTTON_STATE_t;
typedef enum { SR_BUTTON_NO_PRESS=0, SR_BUTTON_SHORT, SR_BUTTON_MED, SR_BUTTON_LONG, SR_BUTTON_VLONG } SR_BUTTON_PRESS_TYPE_t;
typedef void (*SR_BUTTON_CBFN_t)(void* ctx, SR_BUTTON_STATE_t currentState, SR_BUTTON_PRESS_TYPE_t currentPressType);

typedef void (*SR_NOISE_CBFN_t)(void* ctx, int noiseFreq, int noiseDBm);

/** 
 * startup sensors ready for reading. Returns true if all ok, false if hw issue
 */
bool SRMgr_start();
void SRMgr_stop();

uint32_t SRMgr_getLastButtonTimeSecs();
// define an IO as a button
bool SRMgr_defineButton(int8_t io);
// Register callback to be notified when button changes state (also means button is active during deep sleep)
bool SRMgr_registerButtonCB(int8_t io, SR_BUTTON_CBFN_t cb, void* ctx);
// Remove registration - if noone is registered then button state only checked at UL time..
void SRMgr_unregisterButtonCB(SR_BUTTON_CBFN_t cb);

uint32_t SRMgr_getLastNoiseTimeSecs();
uint8_t SRMgr_getNoiseFreqkHz();
uint8_t SRMgr_getNoiseLeveldB();
// Register callback to be notified when noise is detected (also means micro is active during deep sleep)
bool SRMgr_registerNoiseCB(SR_NOISE_CBFN_t cb, void* ctx);
// Remove registration - if noone is registered then micro input only checked at UL time..
void SRMgr_unregisterNoiseCB(SR_NOISE_CBFN_t cb);

int16_t SRMgr_getTempcC();
int32_t SRMgr_getPressurePa();
uint16_t SRMgr_getBatterymV();
uint8_t SRMgr_getLight();
uint16_t SRMgr_getADC1mV();
uint16_t SRMgr_getADC2mV();
uint8_t SRMgr_getButton(int8_t io);
bool SRMgr_hasButtonChanged(int8_t io);
uint8_t SRMgr_getLastButtonPressType(int8_t io);
/* timestamps in ms since boot */
uint32_t SRMgr_getLastButtonPressTS(int8_t io);
uint32_t SRMgr_getLastButtonReleaseTS(int8_t io);
bool SRMgr_hasTempChanged();
bool SRMgr_hasPressureChanged();
bool SRMgr_hasBattChanged();
bool SRMgr_hasLightChanged();
bool SRMgr_hasADC1Changed();
bool SRMgr_hasADC2Changed();
// signal can update the 'last' value as used the new value
void SRMgr_updateTemp();
void SRMgr_updatePressure();
void SRMgr_updateBatt();
void SRMgr_updateLight();
void SRMgr_updateADC1();
void SRMgr_updateADC2();
void SRMgr_updateButton(int8_t io);
#ifdef __cplusplus
}
#endif

#endif  /* H_SENSORMGR_H */

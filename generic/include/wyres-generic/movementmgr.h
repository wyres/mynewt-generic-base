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
#ifndef H_MOVEMENTMGR_H
#define H_MOVEMENTMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { UPRIGHT, INVERTED, FLAT_BACK, FLAT_FACE, UNKNOWN} MM_ORIENT;
typedef void (*MM_CBFN_t)(void);

// Refresh values
void MMMgr_check();
// Register a callback for when movement detected
bool MMMgr_registerMovementCB(MM_CBFN_t cb);
// Last time a "movement" was detected
uint32_t MMMgr_getLastMovedTime();
// Has it moved since this relative time (in seconds since epoch)
bool MMMgr_hasMovedSince(uint32_t reltimeSecs);
// Last time free fall detected
uint32_t MMMgr_getLastFallTime();
bool MMMgr_hasFallenSince(uint32_t reltimeSecs);
// Last time got a shock (>2G)
uint32_t MMMgr_getLastShockTime();
bool MMMgr_hasShockedSince(uint32_t reltimeSecs);
// Register a callback for when orientation change detected
bool MMMgr_registerOrientationCB(MM_CBFN_t cb);
// Last time orientation changed to new value
uint32_t MMMgr_getLastOrientTime();
MM_ORIENT MMMgr_getOrientation();
// in units of G/10
int8_t MMMgr_getXdG();
int8_t MMMgr_getYdG();
int8_t MMMgr_getZdG();

#ifdef __cplusplus
}
#endif

#endif  /* H_MOVEMENTMGR_H */

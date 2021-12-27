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
 * Movement manager. 
 * Uses the accelero to provide info about movement, orientation etc
 * Possibility to add callback to be informed when movement is detected
 */
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_i2c.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/lowpowermgr.h"

#include "wyres-generic/movementmgr.h"
#include "wyres-generic/timemgr.h"
#include "wyres-generic/acc_basic.h"
#include "wyres-generic/configmgr.h"


#define MAX_MMCBFNS MYNEWT_VAL(MAX_MMCBFNS)

static void callMovedCBs();
static void checkMoved();
static void checkOrientationChange();
static MM_ORIENT calcOrient(int8_t x, int8_t y, int8_t z);

static struct {
    // Registered callbacks fns
    MM_CBFN_t movecbs[MAX_MMCBFNS];     
    MM_CBFN_t orientcbs[MAX_MMCBFNS];   
    // current data from last read
    int8_t x;
    int8_t y;
    int8_t z;
    bool active;                        // is the accelero hw activated?
    ACC_DetectionMode_t detectionMode;
    uint32_t lastMoveTimeS;
    uint32_t lastFallTimeS;
    uint32_t lastShockTimeS;
    uint32_t lastOrientTimeS;
    bool movedSinceLastCheck;
    MM_ORIENT orientation;
    LP_ID_t lpUserId;
} _ctx;

void movement_init(void) 
{
    //Accelero config
    uint8_t threshold = 0;
    uint8_t duration = 0;
    // clear context
    memset(&_ctx, 0, sizeof(_ctx));
    _ctx.orientation = UNKNOWN;
    //Retrieve config for accelero from EEPROM
    CFMgr_getOrAddElement(CFG_UTIL_KEY_ACCELERO_DETECTION_MODE, &_ctx.detectionMode, sizeof(ACC_DetectionMode_t));
    switch(_ctx.detectionMode)
    {
        case ACC_FreeFallDetection:
        {
            threshold = 25;
            duration  = 2;
            CFMgr_getOrAddElement(CFG_UTIL_KEY_ACCELERO_FREEFALL_THRESHOLD, &threshold, sizeof(uint8_t));
            CFMgr_getOrAddElement(CFG_UTIL_KEY_ACCELERO_FREEFALL_DURATION, &duration, sizeof(uint8_t));
            break;
        }
        case ACC_ShockDetection:
        {
            threshold = 100;
            duration  = 6;
            CFMgr_getOrAddElement(CFG_UTIL_KEY_ACCELERO_SHOCK_THRESHOLD, &threshold, sizeof(uint8_t));
            CFMgr_getOrAddElement(CFG_UTIL_KEY_ACCELERO_SHOCK_DURATION, &duration, sizeof(uint8_t));
            break;
        }
        case ACC_DetectionOff:
        default:
        {
            break;
        }
    }
    // check accelero sensor exists and configure it
    if (ACC_init() != ACC_SUCCESS) 
    {
        // Can't log in sysinit called fns
        log_noout("accelero hw init fails");
        wassert_hw_fault();
    }
    if (ACC_setDetectionMode(_ctx.detectionMode, threshold, duration) != ACC_SUCCESS)
    {
        log_noout("accelero detection configuration fails");
        wassert_hw_fault();
    }

    // register with LP mgr to de able to say what LP mode we are ok with
    _ctx.lpUserId = LPMgr_register(NULL);   // no need to tell me
    // We are always ok with deepsleep during idle periods, as no long running actions on the device
    LPMgr_setLPMode(_ctx.lpUserId, LP_DEEPSLEEP);
}

bool MMMgr_registerMovementCB(MM_CBFN_t cb) 
{
    for(int i=0;i<MAX_MMCBFNS;i++) 
    {
        if (_ctx.movecbs[i]==NULL) 
        {
            _ctx.movecbs[i] = cb;
            return true;
        }
    }
    return false;
}
bool MMMgr_registerOrientationCB(MM_CBFN_t cb) {
    for(int i=0;i<MAX_MMCBFNS;i++) 
    {
        if (_ctx.orientcbs[i]==NULL) 
        {
            _ctx.orientcbs[i] = cb;
            return true;
        }
    }
    return false;
}

bool MMMgr_start() {
    // Ignore if already activated
    if (!_ctx.active) {
        _ctx.active = (ACC_activate() == ACC_SUCCESS);
    }
    MMMgr_check();      // update data from device now
    return _ctx.active;
}

bool MMMgr_stop() {
    MMMgr_check();      // update data from device before sleeping
    // Ignore if already not active
    if (_ctx.active) {
        _ctx.active=false;
        // This just attempts to put the accelero into its 'lowest' power mode : it should remain accessible, and monitor at least movement events
        return (ACC_sleep() == ACC_SUCCESS);
    }
    return true;
}
// poll accelero for x,y,z,moved,fall,shock
// Call MMMgr_start() before to get best data 
// returns true if hw ok, false if access fails
bool MMMgr_check() 
{
    bool ret = true;
    bool hasDetected = false;
    // NOTE : we check the hw independantly of the active/sleep status of the device, as 'sleep' should not prevent responses
    if (ACC_HasDetectedMoved(&hasDetected) == ACC_SUCCESS) 
    {
        if (hasDetected)
        {
            _ctx.movedSinceLastCheck = true;
            _ctx.lastMoveTimeS = TMMgr_getRelTimeSecs();
            log_debug("mm:MOVED");
        } else {
            log_debug("mm:NOT MOVED");
        }
    }
    else
    {
        log_warn("mm: move read failed");
        ret = false;
    }
    hasDetected = false;
    if (ACC_HasDetectedFreeFallOrShock(&hasDetected) == ACC_SUCCESS) 
    {
        if (hasDetected)
        {
            if (_ctx.detectionMode==ACC_FreeFallDetection) {
                _ctx.lastFallTimeS = TMMgr_getRelTimeSecs();
                log_debug("mm:FALLEN");
            } else {
                _ctx.lastShockTimeS = TMMgr_getRelTimeSecs();
                log_debug("mm:SHOCK");
            }
        }
    }
    else
    {
        log_warn("mm: fall read failed");
        ret = false;
    }
    int8_t x=_ctx.x;
    int8_t y=_ctx.y;
    int8_t z=_ctx.z;
    
    if (ACC_readXYZ(&x, &y, &z) == ACC_SUCCESS)
    {
        if (x!=_ctx.x || y!=_ctx.y || z!=_ctx.z) {
            // change of orientation - record new values and update change timestamp
            _ctx.x = x;
            _ctx.y = y;
            _ctx.z = z;
            _ctx.lastOrientTimeS = TMMgr_getRelTimeSecs();
        }
//        log_debug("x:%d, y:%d, z:%d",_ctx.x, _ctx.y, _ctx.z);
    }
    else
    {
        log_warn("mm: failed read xyz");
        ret = false;
    }
    // check if event means call callbacks 
    checkMoved();
    checkOrientationChange();
    return ret;
}

/** access accelero info. Note that accessor fns do not read from device : call MMMgr_check() to update data.
 * */
uint32_t MMMgr_getLastMovedTime() 
{
    return _ctx.lastMoveTimeS;
}
bool MMMgr_hasMovedSince(uint32_t reltimeS) 
{
    return (_ctx.lastMoveTimeS>reltimeS);
}
uint32_t MMMgr_getLastFallTime() 
{
    return _ctx.lastFallTimeS;
}
bool MMMgr_hasFallenSince(uint32_t reltimeS)
{
    return (_ctx.lastFallTimeS>reltimeS);
}

uint32_t MMMgr_getLastShockTime() 
{
    return _ctx.lastShockTimeS;
}
bool MMMgr_hasShockedSince(uint32_t reltimeS) 
{
    return (_ctx.lastShockTimeS>reltimeS);
}
uint32_t MMMgr_getLastOrientTime() 
{
    return _ctx.lastOrientTimeS;
}
MM_ORIENT MMMgr_getOrientation() 
{
    return _ctx.orientation;
}
// in units of 1/16g 
void MMMgr_getXYZ(int8_t* xp, int8_t* yp, int8_t* zp) {
    // Always return our last known values anyway
    *xp = _ctx.x;
    *yp = _ctx.y;
    *zp = _ctx.z;
    log_debug("mm:x:%d, y:%d, z:%d",_ctx.x, _ctx.y, _ctx.z);

}


// internals
static void callMovedCBs() 
{
    for(int i=0;i<MAX_MMCBFNS;i++) 
    {
        if (_ctx.movecbs[i]!=NULL) 
        {
            (*_ctx.movecbs[i])();
        }
    }
}
static void checkMoved() 
{
    if (_ctx.movedSinceLastCheck) 
    {
        callMovedCBs();
        _ctx.movedSinceLastCheck = false;
    }
}
static MM_ORIENT calcOrient(int8_t x, int8_t y, int8_t z) {
       // evaluate x/y/z to determine this
    if (abs(y)>abs(x) && abs(y)>abs(z)) {
        if (y>0) {
            return UPRIGHT;
        } else {
            return INVERTED;
        }
    }
    if (abs(z)>abs(x) && abs(z)>abs(y)) {
        if (z>0) {
            return FLAT_FACE;
        } else {
            return FLAT_BACK;
        }
    }
    return UNKNOWN;
}

static void checkOrientationChange() 
{
    MM_ORIENT o = calcOrient(_ctx.x, _ctx.y, _ctx.z);
    if (o != _ctx.orientation) 
    {
        _ctx.lastOrientTimeS = TMMgr_getRelTimeSecs();
        _ctx.orientation = o;
        for(int i=0;i<MAX_MMCBFNS;i++) 
        {
            if (_ctx.orientcbs[i]!=NULL) 
            {
                (*_ctx.orientcbs[i])();
            }
        }
    }
}

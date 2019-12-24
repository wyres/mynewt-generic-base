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

static struct {
    // Registered callbacks fns
    MM_CBFN_t movecbs[MAX_MMCBFNS];        // TODO should be a mempool
    MM_CBFN_t orientcbs[MAX_MMCBFNS];        // TODO should be a mempool
// current data from last read
    int8_t x;
    int8_t y;
    int8_t z;
    uint32_t lastMoveTime;
    uint32_t lastFallTime;
    uint32_t lastShockTime;
    uint32_t lastOrientTime;
    bool movedSinceLastCheck;
    MM_ORIENT orientation;
    LP_ID_t lpUserId;
} _ctx;

// low power state change callback : DO NOT LOG OR TAKE TOO MUCH STACK
void lp_change(LP_MODE_t prev, LP_MODE_t new) 
{
    if (new>=LP_DEEPSLEEP)
    {
        if (ACC_sleep() != ACC_SUCCESS)
        {
            log_noout("Accelero failed to go to sleep");
        }
        // deinit I2C in BSP?
        // TODO
    } 
    else 
    {
        // init I2C in BSP?
        if (ACC_activate() != ACC_SUCCESS)
        {
            log_noout("Accelero failed to activate");
        }
    }
}

void movement_init(void) 
{
    //Accelero config
    ACC_DetectionMode_t detectionMode = ACC_DetectionOff;
    uint8_t threshold = 0;
    uint8_t duration = 0;
    // clear context
    memset(&_ctx, 0, sizeof(_ctx));
    _ctx.orientation = UNKNOWN;
    //Retrieve config for accelero from EEPROM
    CFMgr_getOrAddElement(CFG_UTIL_KEY_ACCELERO_DETECTION_MODE, &detectionMode, sizeof(ACC_DetectionMode_t));
    switch(detectionMode)
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
        assert(0);
    }
    if (ACC_setDetectionMode(detectionMode, threshold, duration) != ACC_SUCCESS)
    {
        log_noout("accelero detection configuration fails");
        assert(0);
    }

    // start timer for checks? or register with a "callmeWhenAwakeANyway" service?

    // register with LP mgr to get called on changes, to deinit/init the I2C
    _ctx.lpUserId = LPMgr_register(lp_change);
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
// poll accelero for x,y,z,moved,fall,shock
// TODO possbily should 'start'/'stop' to let accelero have time to decide which way is up before reading values?
void MMMgr_check() 
{
    bool hasDetected = false;
    if (ACC_activate() == ACC_SUCCESS) 
    {
        log_debug("mm:check");
        if (ACC_HasDetectedMoved(&hasDetected) == ACC_SUCCESS) 
        {
            if (hasDetected)
            {
                _ctx.movedSinceLastCheck = true;
                _ctx.lastMoveTime = TMMgr_getRelTime();
                log_debug("mm:MOVED");
            }
        }
        else
        {
            log_warn("Accelero move detection failed");
        }
        hasDetected = false;
        if (ACC_HasDetectedFreeFallOrShock(&hasDetected) == ACC_SUCCESS) 
        {
            if (hasDetected)
            {
                _ctx.lastFallTime = TMMgr_getRelTime();
                log_debug("mm:FALLEN");
            }
        }
        else
        {
            log_warn("Accelero move detection failed");
        }

        if (ACC_readXYZ(&_ctx.x, &_ctx.y, &_ctx.z) == ACC_SUCCESS)
        {
            log_debug("x:%d, y:%d, z:%d",_ctx.x, _ctx.y, _ctx.z);
        }
        else
        {
            log_warn("Accelero failed to read raw data");
        }  
        if (ACC_sleep() != ACC_SUCCESS)
        {
            log_warn("Accelero failed to go to sleep");
        }
    } 
    else 
    {
        log_warn("mm:accelero failed to activate");
    }
    checkMoved();
    checkOrientationChange();
}

uint32_t MMMgr_getLastMovedTime() 
{
    return _ctx.lastMoveTime;
}
bool MMMgr_hasMovedSince(uint32_t reltime) 
{
    return (_ctx.lastMoveTime>reltime);
}
uint32_t MMMgr_getLastFallTime() 
{
    return _ctx.lastFallTime;
}
bool MMMgr_hasFallenSince(uint32_t reltime)
{
    return (_ctx.lastFallTime>reltime);
}

uint32_t MMMgr_getLastShockTime() 
{
    return _ctx.lastShockTime;
}
bool MMMgr_hasShockedSince(uint32_t reltime) 
{
    return (_ctx.lastShockTime>reltime);
}
uint32_t MMMgr_getLastOrientTime() 
{
    return _ctx.lastOrientTime;
}
MM_ORIENT MMMgr_getOrientation() 
{
    // evaluate x/y/z to determine this
    if (abs(_ctx.y)>abs(_ctx.x) && abs(_ctx.y)>abs(_ctx.z)) {
        if (_ctx.y>0) {
            return UPRIGHT;
        } else {
            return INVERTED;
        }
    }
    if (abs(_ctx.z)>abs(_ctx.x) && abs(_ctx.z)>abs(_ctx.y)) {
        if (_ctx.z>0) {
            return FLAT_FACE;
        } else {
            return FLAT_BACK;
        }
    }
    return UNKNOWN;
}
// in units of G/10 (decaG)
int8_t MMMgr_getXdG()
{
    return _ctx.x;
}
int8_t MMMgr_getYdG() 
{
    return _ctx.y;
}
int8_t MMMgr_getZdG() 
{
    return _ctx.z;
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
static void checkOrientationChange() 
{
    if (MMMgr_getOrientation() != _ctx.orientation) 
    {
        _ctx.lastOrientTime = TMMgr_getRelTime();
        _ctx.orientation = MMMgr_getOrientation();
        for(int i=0;i<MAX_MMCBFNS;i++) 
        {
            if (_ctx.orientcbs[i]!=NULL) 
            {
                (*_ctx.orientcbs[i])();
            }
        }
    }
}

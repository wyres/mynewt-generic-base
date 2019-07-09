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

#include "wyres-generic/movementmgr.h"

// Timeout for I2C accesses in 'ticks'
#define I2C_ACCESS_TIMEOUT (100)

#define MAX_MMCBFNS MYNEWT_VAL(MAX_MMCBFNS)


// I2C config
// Would be nice if it was a bus node defined in bsp for us... and we could just open it...
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
} _ctx;

void movement_init(void) {
    // clear context
    memset(&_ctx, 0, sizeof(_ctx));
    // check accelero exists
    int rc = hal_i2c_master_probe(ACCELERO_I2C_CHAN, ACCELERO_I2C_ADDR, I2C_ACCESS_TIMEOUT);
    assert(rc==0);
    // setup accelero for our usage

    // start timer for checks? or register with a "callmeWhenAwakeANyway" service?
}

bool MMMgr_registerMovementCB(MM_CBFN_t cb) {
    for(int i=0;i<MAX_MMCBFNS;i++) {
        if (_ctx.movecbs[i]==NULL) {
            _ctx.movecbs[i] = cb;
            return true;
        }
    }
    return false;
}
bool MMMgr_registerOrientationCB(MM_CBFN_t cb) {
    for(int i=0;i<MAX_MMCBFNS;i++) {
        if (_ctx.orientcbs[i]==NULL) {
            _ctx.orientcbs[i] = cb;
            return true;
        }
    }
    return false;
}
// poll accelero for x,y,z,moved,fall,shock
void MMMgr_check() {
    // TODO
    uint8_t buff[10];
    // This could be ioctls sent to a wskt like device.. or a mynewt sensor... or something
    struct hal_i2c_master_data mdata = {
        .address = ACCELERO_I2C_ADDR,
        .buffer = buff,
        .len = 10,
    };
    int rc = hal_i2c_master_read(ACCELERO_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1);
    if (rc==0) {
    } else {
        log_warn("i2c access to accelero fails:%d", rc);
    }
    _ctx.movedSinceLastCheck = true;
}
uint32_t MMMgr_getLastMovedTime() {
    return _ctx.lastMoveTime;
}
bool MMMgr_hasMovedSince(uint32_t reltime) {
    return (_ctx.lastMoveTime>reltime);
}
uint32_t MMMgr_getLastFallTime() {
    return _ctx.lastFallTime;
}
bool MMMgr_hasFallenSince(uint32_t reltime) {
    return (_ctx.lastFallTime>reltime);
}

uint32_t MMMgr_getLastShockTime() {
    return _ctx.lastShockTime;
}
bool MMMgr_hasShockedSince(uint32_t reltime) {
    return (_ctx.lastShockTime>reltime);
}
uint32_t MMMgr_getLastOrientTime() {
    return _ctx.lastOrientTime;
}
MM_ORIENT MMMgr_getOrientation() {
    // evaluate x/y/z to determine this
    if (_ctx.x>0 && _ctx.x>_ctx.y && _ctx.x>_ctx.z) {
        return UPRIGHT;
    }
    if (_ctx.x<0 && _ctx.x<_ctx.y && _ctx.x<_ctx.z) {
        return INVERTED;
    }
    if (_ctx.y>0 && _ctx.y>_ctx.x && _ctx.y>_ctx.z) {
        return FLAT_FACE;
    }
    if (_ctx.y<0 && _ctx.y<_ctx.x && _ctx.y<_ctx.z) {
        return FLAT_BACK;
    }
    return UNKNOWN;
}
// in units of G/10 (decaG)
int8_t MMMgr_getXdG() {
    return _ctx.x;
}
int8_t MMMgr_getYdG() {
    return _ctx.y;
}
int8_t MMMgr_getZdG() {
    return _ctx.z;
}

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

#define FNUM FN_MOVEMENTMGR

// Timeout for I2C accesses in 'ticks'
#define I2C_ACCESS_TIMEOUT (100)

#define MAX_MMCBFNS MYNEWT_VAL(MAX_MMCBFNS)

// Registered callbacks fns
static MM_CBFN_t _cbs[MAX_MMCBFNS];        // TODO should be a mempool

// I2C config
// Would be nice if it was a bus node defined in bsp for us... and we could just open it...

// current data from last read
static int8_t _x=0;
static int8_t _y=0;
static int8_t _z=0;
static uint32_t _lastMoveTime = 0;
static uint32_t _lastFallTime = 0;
static uint32_t _lastShockTime = 0;
static bool _movedSinceLastCheck=false;

void movement_init(void) {
    // clear CB array
    memset(_cbs, 0, sizeof(_cbs));
    // check accelero exists
    int rc = hal_i2c_master_probe(ACCELERO_I2C_CHAN, ACCELERO_I2C_ADDR, I2C_ACCESS_TIMEOUT);
    assert(rc==0);
    // setup accelero for our usage

    // start timer for checks? or register with a "callmeWhenAwakeANyway" service?
}

bool MMMgr_register(MM_CBFN_t cb) {
    for(int i=0;i<MAX_MMCBFNS;i++) {
        if (_cbs[i]==NULL) {
            _cbs[i] = cb;
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
    _movedSinceLastCheck = true;
}
uint8_t MMMgr_getMovedMask() {
    return 0;       // TODO
}
uint32_t MMMgr_getLastMovedTime() {
    return _lastMoveTime;
}
bool MMMgr_hasMovedSince(uint32_t reltime) {
    return (_lastMoveTime>reltime);
}
uint8_t MMMgr_getFallMask() {
    return 0;       // TODO
}
uint32_t MMMgr_getLastFallTime() {
    return _lastFallTime;
}
bool MMMgr_hasFallenSince(uint32_t reltime) {
    return (_lastFallTime>reltime);
}
uint8_t MMMgr_getShockMask() {
    return 0;       // TODO
}

uint32_t MMMgr_getLastShockTime() {
    return _lastShockTime;
}
bool MMMgr_hasShockedSince(uint32_t reltime) {
    return (_lastShockTime>reltime);
}
MM_ORIENT MMMgr_getOrientation() {
    // evaluate x/y/z to determine this
    if (_x>0 && _x>_y && _x>_z) {
        return UPRIGHT;
    }
    if (_x<0 && _x<_y && _x<_z) {
        return INVERTED;
    }
    if (_y>0 && _y>_x && _y>_z) {
        return FLAT_FACE;
    }
    if (_y<0 && _y<_x && _y<_z) {
        return FLAT_BACK;
    }
    return UNKNOWN;
}
// in units of G/10
int8_t MMMgr_getX() {
    return _x;
}
int8_t MMMgr_getY() {
    return _y;
}
int8_t MMMgr_getZ() {
    return _z;
}

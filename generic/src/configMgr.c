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
 * Config manager
 * Allows definition of config elements which are opaque and up to 255 bytes long
 * These are stored/retrieved from either FLASH or PROM
 */
#include "os/os.h"
#include "bsp/bsp.h"
#include "wyres-generic/wutils.h"

#include "wyres-generic/configmgr.h"

#define MAX_KEYS 200 //MYNEWT_VAL(CFG_MAX_KEYS)
#define INDEX_SIZE  (5)
#define NVM_HDR_SIZE (0x10)
#define MAX_CFG_CBS 10

struct cfg {
    uint8_t nbKeys;
    uint16_t indexStart;
    uint16_t storeStart;
    uint16_t storeOffset;
    uint8_t nCBs;
    CFG_CBFN_t cbList[MAX_CFG_CBS];
    // use table in PROM directly to avoid using up the RAM
//    struct {
//        uint16_t key;
//        uint8_t len;
//        uint16_t off;
//    } indexTable[MAX_KEYS];
} _cfg;     // all inited to 0 by definition (bss)

static void cfgLockR();
static void cfgUnlockR();
static void cfgLockW();
static void cfgUnlockW();

static int createKey(uint16_t k, uint8_t l, uint8_t* d);
static int findKeyIdx(uint16_t k);
static uint16_t getIdxKey(int idx);
static uint8_t getIdxLen(int idx);
static uint16_t getIdxOff(int idx);
static void informListeners(uint16_t key);
#ifndef RELEASE_BUILD
void dumpCfg();
#endif /* RELEASE_BUILD */
// See eeprom_board.c in src/board/iM880C for basic access methods eg EEPROM_SaveConfiguration() (but do it much better)

// Register a callback fn for whenever the cfg changes
bool CFMgr_registerCB(CFG_CBFN_t cb) {
    if (_cfg.nCBs>=MAX_CFG_CBS) {
        return false;
    }
    if (cb==NULL) {
        return false;
    }
    _cfg.cbList[_cfg.nCBs++] = cb;
    return true;
}

// Add a new element definition key. If the key is already known AND has the same len, this is a noop. 
// If the key exists but has a different length, false is returned.
// If the key is unknown, it is added to the dictionary and the value is set to that of initdata.
bool CFMgr_addElementDef(uint16_t key, uint8_t len, void* initdata) {
    bool ret = false;
    cfgLockR();
    int idx = findKeyIdx(key);
    if (idx>=0) {
        if (getIdxLen(idx) == len) {
            ret = true;
        } else {
            ret = false;       // exists already but with different len!
        }
    } else {
        cfgLockW();
        idx = createKey(key, len, (uint8_t*)initdata);
        cfgUnlockW();
        ret = (idx>=0);
        if (idx<0) {
            log_noout("CFGAE:FAIL CK %4x at idx %d", key, idx);
        } else {
            log_noout("CFGAE: CK %4x at idx %d", key, idx);
        }
    }
    cfgUnlockR();
    return ret;
}

bool CFMgr_getOrAddElement(uint16_t key, void* data, uint8_t len) {
    bool ret = false;
    cfgLockR();
    int idx = findKeyIdx(key);
    if (idx<0) {
        cfgLockW();
        idx = createKey(key, len, (uint8_t*)data);
        cfgUnlockW();
        ret = (idx>=0);
        if (idx<0) {
            log_noout("CFGGE:FAIL CK %4x at idx %d", key, idx);
        } else {
            log_noout("CFGGE:CK %4x at idx %d", key, idx);
        }
    } else {
        // read data
        // Check the given buffer length is correct for the key
        uint8_t klen = getIdxLen(idx);
        if (klen>len) {
            // incorrect code?
            log_noout("CFGGE:WARN CK %4x at idx %d KL %d DL %d", key, idx, klen, len);
            // continue in case just caller limiting buffer size
            klen = len;
        }
        ret = hal_bsp_nvmRead(getIdxOff(idx), klen, (uint8_t*)data);
    }
    cfgUnlockR();
    return ret;
}

// Helper getOrAdd methods for specific int types that check the range
bool CFMgr_getOrAddElementCheckRangeUINT32(uint16_t key, uint32_t* data, uint32_t min, uint32_t max) {
    // get the value
    uint32_t cv = 0;
    int len = CFMgr_getElement(key, &cv, sizeof(uint32_t));
    if (len==-1) {
        // create with given default (which we will assume is valid...)
        CFMgr_getOrAddElement(key, data, sizeof(uint32_t));
        return true;
    }
    if (len!=sizeof(uint32_t)) {
        return false;       // this should not happen
    }
    // Check configured value is ok
    if (cv>=min && cv<=max) {
        // ok value, return it
        *data = cv;
        return true;
    }
    // Overwrite the bad value with default to ensure ok from now
    CFMgr_setElement(key, data, sizeof(uint32_t));
    return false;       // it was bad...
}

bool CFMgr_getOrAddElementCheckRangeUINT8(uint16_t key, uint8_t* data, uint8_t min, uint8_t max) {
    // get the value
    uint8_t cv = 0;
    int len = CFMgr_getElement(key, &cv, sizeof(uint8_t));
    if (len==-1) {
        // create with given default (which we will assume is valid...)
        CFMgr_getOrAddElement(key, data, sizeof(uint8_t));
        return true;
    }
    if (len!=sizeof(uint8_t)) {
        return false;       // this should not happen
    }
    // Check configured value is ok
    if (cv>=min && cv<=max) {
        // ok value, return it
        *data = cv;
        return true;
    }
    // Overwrite the bad value with default?
    CFMgr_setElement(key, data, sizeof(uint8_t));
    return false;       // it was bad...
}
bool CFMgr_getOrAddElementCheckRangeINT32(uint16_t key, int32_t* data, int32_t min, int32_t max) {
    // get the value
    int32_t cv = 0;
    int len = CFMgr_getElement(key, &cv, sizeof(int32_t));
    if (len==-1) {
        // create with given default (which we will assume is valid...)
        CFMgr_getOrAddElement(key, data, sizeof(int32_t));
        return true;
    }
    if (len!=sizeof(int32_t)) {
        return false;       // this should not happen
    }
    // Check configured value is ok
    if (cv>=min && cv<=max) {
        // ok value, return it
        *data = cv;
        return true;
    }
    // Overwrite the bad value with default?
    CFMgr_setElement(key, data, sizeof(int32_t));
    return false;       // it was bad...
}
bool CFMgr_getOrAddElementCheckRangeINT8(uint16_t key, int8_t* data, int8_t min, int8_t max) {
        // get the value
    int8_t cv = 0;
    int len = CFMgr_getElement(key, &cv, sizeof(int8_t));
    if (len==-1) {
        // create with given default (which we will assume is valid...)
        CFMgr_getOrAddElement(key, data, sizeof(int8_t));
        return true;
    }
    if (len!=sizeof(int8_t)) {
        return false;       // this should not happen
    }
    // Check configured value is ok
    if (cv>=min && cv<=max) {
        // ok value, return it
        *data = cv;
        return true;
    }
    // Overwrite the bad value with default?
    CFMgr_setElement(key, data, sizeof(int8_t));
    return false;       // it was bad...
}

// get a config value into the data buffer, of size maxlen.
// Returns the actual length of the element returned, or -1 if the key does not exist
int CFMgr_getElement(uint16_t key, void* data, uint8_t maxlen) {
    int len = 0;
    cfgLockR();
    int idx = findKeyIdx(key);
    if (idx<0) {
        len = -1;      // no such key
    } else {
        // read data
        len = getIdxLen(idx);
        // limit to buffer given!
        if (len>maxlen) {
            len = maxlen;
        }
        if (hal_bsp_nvmRead(getIdxOff(idx), len, (uint8_t*)data)==false) {
            len = -1;      // fail
        }
    }
    cfgUnlockR();
    return len;
}
uint8_t CFMgr_getElementLen(uint16_t key) {
    uint8_t ret = 0;
    cfgLockR();
    int idx = findKeyIdx(key);
    if (idx>=0) {
        ret = getIdxLen(idx);
    }
    cfgUnlockR();
    return ret;
}
// Set an element value. Creates key if unknown if it can
bool CFMgr_setElement(uint16_t key, void* data, uint8_t len) {
    bool ret = false;
    cfgLockR();
    int idx = findKeyIdx(key);
    if (idx<0) {
        cfgLockW();
        idx = createKey(key, len, (uint8_t*)data);
        cfgUnlockW();
        ret = (idx>=0);
        if (idx<0) {
            log_noout("CFGSE:FAIL CK %4x at idx %d", key, idx);
        } else {
            log_noout("CFGSE:CK %4x at idx %d", key, idx);
        }
    } else {
        uint8_t klen = getIdxLen(idx);
        if (len==klen) {
            // Write data
            cfgLockW();
            ret = hal_bsp_nvmWrite(getIdxOff(idx), getIdxLen(idx), (uint8_t*)data);
            cfgUnlockW();
        } else {
            log_noout("CFGSE:FAIL SK %4x at idx %d bad len %d should be %d", key, idx, len, klen);
            ret = false;
        }
    }
    cfgUnlockR();
    // Tell cfg listeners if ok
    if (ret) {
        informListeners(key);
    }
    return ret;
}

bool CFMgr_resetElement(uint16_t key) {
    bool ret = true;
    cfgLockR();
    int idx = findKeyIdx(key);
    if (idx<0) {
        ret = false;
    } else {
        // Write 0 data
        uint8_t vlen = getIdxLen(idx);
        uint16_t voff = getIdxOff(idx);
        cfgLockW();
        for(int i=0;i<vlen;i++) {
            ret &= hal_bsp_nvmWrite8(voff + i, 0);      // any failure sets result to failure
        }
        cfgUnlockW();
    }
    cfgUnlockR();

    // Tell cfg listeners
    informListeners(key);
    return ret;
}

// iterate over all keys, calling cb for each.
// in the CB the other access methods can be called
void CFMgr_iterateKeys(int keymodule, CFG_CBFN_t cb) {
    int key = 0;
    for(int i=0;i<_cfg.nbKeys; i++) {
        // get each key, but ensure NVM in state to allow CB to call other methods
        cfgLockR();
        key = getIdxKey(i);
        cfgUnlockR();
        // if no keymodule filter, or the key has the correct keymodule as its MSB, give it to CB
        if (keymodule==-1 || (key>>8)==keymodule) {
            (*(cb))(key);
        }
    }


}

// Internals

static void informListeners(uint16_t key) {
    // tell anyone that cares
    for(int i=0;i<_cfg.nCBs;i++) {
        (*(_cfg.cbList[i]))(key);
    }
}
/** PROM layout
0000 [nbKeys(Pri)] [nbKeys(Sec)] [IdxStart_LSB] [IdxStart_MSB] [StoreStart_LSB] [StoreStart_MSB] [RFU=0]x10
0010-(0x10+MAX_KEYS*5) [[Key_LSB] [K_MSB] [Len] [StoreOff_LSB][StoreOff_MSB]] x nbKeys (max 200)
 */

/** startup:
 * read nbKeys(Pri), nbKeys(Sec). If sec>pri, nbKeys = pri.
 * read IdxStart (2 bytes). If 0 or > PROM_SIZE, IdxStart=0x0010, nbKeys=0
 * readStoreStart (2 bytes). If 0 or > PROM_SIZE, StoreStart=MAXKEYS*5+0x10, IdxStart=0x0010, nbKeys=0
 * StoreOffset = StoreStart
 * read in Idx, updating StoreOffset at each entry read to be its StoreOff+len.
 */
void CFMgr_init(void) {
    cfgLockR();
    uint8_t nbK_pri = hal_bsp_nvmRead8(0);
    uint8_t nbK_sec = hal_bsp_nvmRead8(1);
    if (nbK_sec!=nbK_pri) {
        // oops. can't log yet
        log_noout("PROM cfg store corruption (%d, %d)", nbK_pri, nbK_sec);
        // log our fn address, and continue. Might be ok...
        log_fn_fn();
    }
    _cfg.nbKeys = nbK_pri;
    _cfg.indexStart = hal_bsp_nvmRead16(2);
    _cfg.storeStart = hal_bsp_nvmRead16(4);
    if (_cfg.indexStart<NVM_HDR_SIZE|| _cfg.indexStart>hal_bsp_nvmSize() ||
             _cfg.storeStart<NVM_HDR_SIZE || _cfg.storeStart>hal_bsp_nvmSize() || 
             _cfg.storeStart < (_cfg.indexStart+MAX_KEYS*INDEX_SIZE)) {
        log_noout("CFG BAD, resetting");
        _cfg.nbKeys=0;
        _cfg.indexStart=NVM_HDR_SIZE;
        _cfg.storeStart=_cfg.indexStart + (MAX_KEYS+1)*INDEX_SIZE;
        _cfg.storeOffset = _cfg.storeStart; 
        cfgLockW();
        hal_bsp_nvmWrite8(0,0);
        hal_bsp_nvmWrite8(1,0);
        hal_bsp_nvmWrite16(2, _cfg.indexStart);
        hal_bsp_nvmWrite16(4, _cfg.storeStart);
        cfgUnlockW();
        // just log passage : no assert (as this writes to PROM!)
        log_fn_fn();
    }
/*
    // Calculate where next free space in store it while reading the key index into ram
    for(int i=0;i<_cfg.nbKeys; i++) {
        _cfg.indexTable[i].key = nvmRead16(_cfg.indexStart+(i*INDEX_SIZE));
        _cfg.indexTable[i].len = nvmRead8(_cfg.indexStart+(i*INDEX_SIZE)+2);
        _cfg.indexTable[i].off = nvmRead16(_cfg.indexStart+(i*INDEX_SIZE)+3);
        _cfg.storeOffset = _cfg.indexTable[i].off + _cfg.indexTable[i].len;
    }
*/
    if (_cfg.nbKeys==0) {
        // no keys its at the store start
        _cfg.storeOffset = _cfg.storeStart;
    } else {
        // Its after the last one
        _cfg.storeOffset = getIdxOff(_cfg.nbKeys-1) + getIdxLen(_cfg.nbKeys-1);
    }

    cfgUnlockR();

    // ready to roll
    // debug
    log_noout("CFG nbK %d", _cfg.nbKeys);

//    dumpCfg();
}

/** Adding new key
 * If nbKeys>=200, fail
 * Write index entry to PROM_START+IdxStart+(nbKeys*5) [K, L, StoreOffset]
 * StoreOffset+=len
 * nbKeys++
 * write nbKeys(sec)
 * write nbKeys(pri)
 * Returns index in the table of the new key
 * !! MUST HAVE cfgLockW/cfgUnlockW round this call
 */
static int createKey(uint16_t k, uint8_t l, uint8_t* d) {
    assert(l!=0);
    if (_cfg.nbKeys>=MAX_KEYS) {
        return -1;       // no joy
    }
    if (_cfg.storeOffset+l > hal_bsp_nvmSize()) {
        return -1;         // full up
    }
    int ret = _cfg.nbKeys;
    // Wrtie to PROM new index entry
    // check results of PROM accesses and fail nicely
    if (!hal_bsp_nvmWrite16(_cfg.indexStart+ret*INDEX_SIZE, k)) {
        log_noout("CFG fail to write at %4x key %4x",_cfg.indexStart+ret*INDEX_SIZE, k);
        return -1;       // no joy
    }
    if (!hal_bsp_nvmWrite8(_cfg.indexStart+ret*INDEX_SIZE+2, l)) {
        log_noout("CFG fail to write at %4x len %2x",_cfg.indexStart+ret*INDEX_SIZE, l);
        return -1;       // no joy
    }
    if (!hal_bsp_nvmWrite16(_cfg.indexStart+ret*INDEX_SIZE+3, _cfg.storeOffset)) {
        log_noout("CFG fail to write at %4x off %4x",_cfg.indexStart+ret*INDEX_SIZE, _cfg.storeOffset);
        return -1;       // no joy
    }
    // update memory copy
//    _cfg.indexTable[ret].key = k;
//    _cfg.indexTable[ret].len = l;
//    _cfg.indexTable[ret].off = _cfg.storeOffset;
    // Write data into store
    if (!hal_bsp_nvmWrite(_cfg.storeOffset, l, d)) {
        log_noout("CFG fail to write data at %4x len %2x",_cfg.storeOffset, l);
        return -1;       // no joy
    }

    // Move next free space in store along    
    _cfg.storeOffset+=l;
    _cfg.nbKeys++;
    // Update number of key in index in PROM
    if (!hal_bsp_nvmWrite8(1, _cfg.nbKeys)) {
        log_noout("CFG fail to write nbKeysSec %2x",_cfg.nbKeys);
        // rewind
        _cfg.storeOffset-=l;
        _cfg.nbKeys--;
        return -1;       // no joy
    }
    if (!hal_bsp_nvmWrite8(0, _cfg.nbKeys)) {
        log_noout("CFG fail to write nbKeysPri %2x",_cfg.nbKeys);
        // rewind
        _cfg.storeOffset-=l;
        _cfg.nbKeys--;
        return -1;       // no joy
    }

    return ret;
}

// Find the index in the key table for the given key, or -1 if not found
// * !! No need to UNLOCK to make this call as only READ
static int findKeyIdx(uint16_t k) {
    assert(k!=CFG_KEY_ILLEGAL);
    for(int i=0;i<_cfg.nbKeys; i++) {
//        if (_cfg.indexTable[i].key == k) {
        if (getIdxKey(i) == k) {    
            return i;
        }
    }
    return -1;
}
// * !! No need to UNLOCK to make this call as only READ
static uint16_t getIdxKey(int idx) {
    if (idx<0 || idx>=_cfg.nbKeys) {
        return CFG_KEY_ILLEGAL;
    }
    return hal_bsp_nvmRead16(_cfg.indexStart+(idx*INDEX_SIZE));

}
// * !! No need to UNLOCK to make this call as only READ
static uint8_t getIdxLen(int idx) {
    if (idx<0 || idx>=_cfg.nbKeys) {
        return 0;
    }
    return hal_bsp_nvmRead8(_cfg.indexStart+(idx*INDEX_SIZE)+2);
    
}
// * !! No need to UNLOCK to make this call as only READ
static uint16_t getIdxOff(int idx) {
    if (idx<0 || idx>=_cfg.nbKeys) {
        return 0;
    }
    return hal_bsp_nvmRead16(_cfg.indexStart+(idx*INDEX_SIZE)+3);    
}

// Protect access to PROM
// Lock for reading only
static void cfgLockR() {
    // NOOP currently : add mutex if required
}
static void cfgUnlockR() {
    // NOOP currently : add mutex if required
}
// Lock for Writing (and reading). Include PROM protection unlock/lock
static void cfgLockW() {
    // No mutex yet
    // Unlock PROM so can wrtie to it
    hal_bsp_nvmUnlock();
}
static void cfgUnlockW() {
    // Lock PROM so can't accidently write to it
    hal_bsp_nvmLock();
}

#ifndef RELEASE_BUILD
// DUMP PROM to blocking UART
void dumpCfg() {
    cfgLockR();
    uint8_t nbK_pri = hal_bsp_nvmRead8(0);
    uint8_t nbK_sec = hal_bsp_nvmRead8(1);
    log_noout("nbKPri %d, nbKSec %d", nbK_pri, nbK_sec);
    uint16_t indexStart = hal_bsp_nvmRead16(2);
    uint16_t storeStart = hal_bsp_nvmRead16(4);
    if (indexStart<NVM_HDR_SIZE|| indexStart>hal_bsp_nvmSize() ||
             storeStart<NVM_HDR_SIZE || storeStart>hal_bsp_nvmSize() || 
             storeStart < (indexStart+MAX_KEYS*INDEX_SIZE)) {
        log_noout("badness with indexStart %4x or storeStart %4x", indexStart, storeStart);
    } else {
        log_noout("ok with indexStart %4x and storeStart %4x", indexStart, storeStart);
    }
    // Its after the last one
    uint16_t storeOffset = getIdxOff(nbK_pri-1) + getIdxLen(nbK_pri-1);
    log_noout("offset to end of store %4x", storeOffset);
    for(int i=0; i<nbK_pri;i++) {
        log_noout("idx %d -> key %4x, len %d, offset %4x", i, getIdxKey(i), getIdxLen(i), getIdxOff(i));
    }
    cfgUnlockR();
}
#endif /* RELEASE_BUILD */ 
#ifdef UNITTEST
bool unittest_cfg() {
    bool ret = true;        // assume all will go ok
    // test data
    uint8_t data[8]= {0};
    ret &= unittest("get", CFMgr_getOrAddElement(CFGKEY(0, 0), data, 8));
    ret &= unittest("check get def", data[0]==0x00);
    data[0] = 0x01;
    ret &= unittest("set", CFMgr_setElement(CFGKEY(0, 0), data, 8));
    ret &= unittest("get new", CFMgr_getOrAddElement(CFGKEY(0, 0), data, 8));
    ret &= unittest("check new", data[0]==0x01);
    return ret;
}
#endif /* UNITTEST */
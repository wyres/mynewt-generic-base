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
} _cfg = {
    .nbKeys=0,
    .indexStart=0,
    .storeStart=0,
    .storeOffset=0,
    .nCBs=0,
};

static int createKey(uint16_t k, uint8_t l, uint8_t* d);
static int findKeyIdx(uint16_t k);
static uint16_t getIdxKey(int idx);
static uint8_t getIdxLen(int idx);
static uint16_t getIdxOff(int idx);
static void informListeners(uint16_t key);
void dumpCfg();

// See eeprom_board.c in src/board/iM880C for basic access methods eg EEPROM_SaveConfiguration() (but do it much better)

// Register a callback fn for whenever the cfg changes
bool CFMgr_registerCB(CFG_CBFN_t cb) {
    if (_cfg.nCBs>=MAX_CFG_CBS) {
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
    nvmUnlock();
    int idx = findKeyIdx(key);
    if (idx>=0) {
        if (getIdxLen(idx) == len) {
            ret = true;
        } else {
            ret = false;       // exists already but with different len!
        }
    } else {
        idx = createKey(key, len, (uint8_t*)initdata);
        ret = (idx>=0);
        if (idx<0) {
            log_noout("CFGAE:FAIL CK %4x at idx %d", key, idx);
        } else {
            log_noout("CFGAE: CK %4x at idx %d", key, idx);
        }
    }
    nvmLock();
    return ret;
}

bool CFMgr_getOrAddElement(uint16_t key, void* data, uint8_t len) {
    bool ret = false;
    nvmUnlock();
    int idx = findKeyIdx(key);
    if (idx<0) {
        idx = createKey(key, len, (uint8_t*)data);
        ret = (idx>=0);
        if (idx<0) {
            log_noout("CFGGE:FAIL CK %4x at idx %d", key, idx);
        } else {
            log_noout("CFGGE:CK %4x at idx %d", key, idx);
        }
    } else {
        // read data
        ret = nvmRead(getIdxOff(idx), getIdxLen(idx), (uint8_t*)data);
    }
    nvmLock();
    return ret;
}
uint8_t CFMgr_getElementLen(uint16_t key) {
    uint8_t ret = 0;
    nvmUnlock();
    int idx = findKeyIdx(key);
    if (idx>=0) {
        ret = getIdxLen(idx);
    }
    nvmLock();
    return ret;
}
// Set an element value. Creates key if unknown if it can
bool CFMgr_setElement(uint16_t key, void* data, uint8_t len) {
    bool ret = false;
    nvmUnlock();
    int idx = findKeyIdx(key);
    if (idx<0) {
        idx = createKey(key, len, (uint8_t*)data);
        ret = (idx>=0);
        if (idx<0) {
            log_noout("CFGSE:FAIL CK %4x at idx %d", key, idx);
        } else {
            log_noout("CFGSE:CK %4x at idx %d", key, idx);
        }
    } else {
        assert(len==getIdxLen(idx));
        // Write data
        ret = nvmWrite(getIdxOff(idx), getIdxLen(idx), (uint8_t*)data);
    }
    nvmLock();
    // Tell cfg listeners
    informListeners(key);
    return ret;
}

bool CFMgr_resetElement(uint16_t key) {
    bool ret = true;
    nvmUnlock();
    int idx = findKeyIdx(key);
    if (idx<0) {
        ret = false;
    } else {
        // Write 0 data
        uint8_t vlen = getIdxLen(idx);
        uint16_t voff = getIdxOff(idx);
        for(int i=0;i<vlen;i++) {
            ret &= nvmWrite8(voff + i, 0);      // any failure sets result to failure
        }
    }
    nvmLock();

    // Tell cfg listeners
    informListeners(key);
    return ret;
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
   // ? neccessary to unlock to READ?
    nvmUnlock();
    uint8_t nbK_pri = nvmRead8(0);
    uint8_t nbK_sec = nvmRead8(1);
    if (nbK_sec!=nbK_pri) {
        // oops. can't log yet
    //  log_warn("PROM cfg store corruption possible...");
    }
    _cfg.nbKeys = nbK_pri;
    _cfg.indexStart = nvmRead16(2);
    _cfg.storeStart = nvmRead16(4);
    if (_cfg.indexStart<NVM_HDR_SIZE|| _cfg.indexStart>nvmSize() ||
             _cfg.storeStart<NVM_HDR_SIZE || _cfg.storeStart>nvmSize() || 
             _cfg.storeStart < (_cfg.indexStart+MAX_KEYS*INDEX_SIZE)) {
        _cfg.nbKeys=0;
        _cfg.indexStart=NVM_HDR_SIZE;
        _cfg.storeStart=_cfg.indexStart + (MAX_KEYS+1)*INDEX_SIZE;
        _cfg.storeOffset = _cfg.storeStart; 
        nvmWrite8(0,0);
        nvmWrite8(1,0);
        nvmWrite16(2, _cfg.indexStart);
        nvmWrite16(4, _cfg.storeStart);
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

    nvmLock();

    // ready to roll
    // debug
    dumpCfg();
}

/** Adding new key
 * If nbKeys>=200, fail
 * Write index entry to PROM_START+IdxStart+(nbKeys*5) [K, L, StoreOffset]
 * StoreOffset+=len
 * nbKeys++
 * write nbKeys(sec)
 * write nbKeys(pri)
 * Returns index in the table of the new key
 * !! MUST HAVE UNLOCK/LOCK round this call
 */
static int createKey(uint16_t k, uint8_t l, uint8_t* d) {
    assert(l!=0);
    if (_cfg.nbKeys>=MAX_KEYS) {
        return -1;       // no joy
    }
    if (_cfg.storeOffset+l > nvmSize()) {
        return -1;         // full up
    }
    int ret = _cfg.nbKeys;
    // Wrtie to PROM new index entry
    // TODO check results of PROM accesses and fail nicely
    if (!nvmWrite16(_cfg.indexStart+ret*INDEX_SIZE, k)) {
        log_noout("CFG fail to write at %4x key %4x",_cfg.indexStart+ret*INDEX_SIZE, k);
        return -1;       // no joy
    }
    if (!nvmWrite8(_cfg.indexStart+ret*INDEX_SIZE+2, l)) {
        log_noout("CFG fail to write at %4x len %2x",_cfg.indexStart+ret*INDEX_SIZE, l);
        return -1;       // no joy
    }
    if (!nvmWrite16(_cfg.indexStart+ret*INDEX_SIZE+3, _cfg.storeOffset)) {
        log_noout("CFG fail to write at %4x off %4x",_cfg.indexStart+ret*INDEX_SIZE, _cfg.storeOffset);
        return -1;       // no joy
    }
    // update memory copy
//    _cfg.indexTable[ret].key = k;
//    _cfg.indexTable[ret].len = l;
//    _cfg.indexTable[ret].off = _cfg.storeOffset;
    // Write data into store
    if (!nvmWrite(_cfg.storeOffset, l, d)) {
        log_noout("CFG fail to write data at %4x len %2x",_cfg.storeOffset, l);
        return -1;       // no joy
    }

    // Move next free space in store along    
    _cfg.storeOffset+=l;
    _cfg.nbKeys++;
    // Update number of key in index in PROM
    if (!nvmWrite8(1, _cfg.nbKeys)) {
        log_noout("CFG fail to write nbKeysSec %2x",_cfg.nbKeys);
        return -1;       // no joy
    }
    if (!nvmWrite8(0, _cfg.nbKeys)) {
        log_noout("CFG fail to write nbKeysPri %2x",_cfg.nbKeys);
        return -1;       // no joy
    }

    return ret;
}

// Find the index in the key table for the given key, or -1 if not found
// * !! MUST HAVE UNLOCK/LOCK round this call
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
// * !! MUST HAVE UNLOCK/LOCK round this call
static uint16_t getIdxKey(int idx) {
    if (idx<0 || idx>=_cfg.nbKeys) {
        return CFG_KEY_ILLEGAL;
    }
    return nvmRead16(_cfg.indexStart+(idx*INDEX_SIZE));

}
// * !! MUST HAVE UNLOCK/LOCK round this call
static uint8_t getIdxLen(int idx) {
    if (idx<0 || idx>=_cfg.nbKeys) {
        return 0;
    }
    return nvmRead8(_cfg.indexStart+(idx*INDEX_SIZE)+2);
    
}
// * !! MUST HAVE UNLOCK/LOCK round this call
static uint16_t getIdxOff(int idx) {
    if (idx<0 || idx>=_cfg.nbKeys) {
        return 0;
    }
    return nvmRead16(_cfg.indexStart+(idx*INDEX_SIZE)+3);    
}

// DEBUG
// DUMP PROM to blocking UART
void dumpCfg() {
    nvmUnlock();
    uint8_t nbK_pri = nvmRead8(0);
    uint8_t nbK_sec = nvmRead8(1);
    log_noout("nbKPri %d, nbKSec %d", nbK_pri, nbK_sec);
    uint16_t indexStart = nvmRead16(2);
    uint16_t storeStart = nvmRead16(4);
    if (indexStart<NVM_HDR_SIZE|| indexStart>nvmSize() ||
             storeStart<NVM_HDR_SIZE || storeStart>nvmSize() || 
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
    nvmLock();

}
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
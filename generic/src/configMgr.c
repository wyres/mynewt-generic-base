/**
 * Wyres private code
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

struct cfg {
    uint16_t indexStart;
    uint16_t storeStart;
    uint16_t storeOffset;
    uint8_t nbKeys;
    struct {
        uint16_t key;
        uint8_t len;
        uint16_t off;
    } indexTable[MAX_KEYS];
} _cfg;

static int createKey(uint16_t k, uint8_t l);
static int findKeyIdx(uint16_t k);

// See eeprom_board.c in src/board/iM880C for basic access methods eg EEPROM_SaveConfiguration() (but do it much better)

// Add a new element definition key. If the key is already known AND has the same len, this is a noop. 
// If the key exists but has a different length, false is returned.
// If the key is unknown, it is added to the dictionary and the value is set to that of initdata.
bool CFMgr_addElementDef(uint16_t key, uint8_t len, void* initdata) {
    int idx = findKeyIdx(key);
    if (idx>=0) {
        if (_cfg.indexTable[idx].len == len) {
            return true;
        }
        return false;       // exists already but with different len!
    }
    idx = createKey(key, len);
    if (idx<0) {
        return false;       // full up, sorry
    }
    // Write data
    return nvmWrite(_cfg.indexTable[idx].off, len, (uint8_t*)initdata);
}

bool CFMgr_getOrAddElement(uint16_t key, void* data, uint8_t len) {
    int idx = findKeyIdx(key);
    if (idx<0) {
        idx = createKey(key, len);
        if (idx<0) {
            return false;
        }
        // Write the given data as the initial value and return
        return nvmWrite(_cfg.indexTable[idx].off, _cfg.indexTable[idx].len, (uint8_t*)data);
    }
    // read data
    return nvmRead(_cfg.indexTable[idx].off, _cfg.indexTable[idx].len, (uint8_t*)data);
}
uint8_t CFMgr_getElementLen(uint16_t key) {
    int idx = findKeyIdx(key);
    if (idx<0) {
        return 0;
    }
    return _cfg.indexTable[idx].len;
}
// Set an element value. Creates key if unknown if it can
bool CFMgr_setElement(uint16_t key, void* data, uint8_t len) {
    int idx = findKeyIdx(key);
    if (idx<0) {
        idx = createKey(key, len);
        if (idx<0) {
            return false;
        }
    }
    // Write data
    return nvmWrite(_cfg.indexTable[idx].off, len, (uint8_t*)data);
}

bool CFMgr_resetElement(uint16_t key) {
    int idx = findKeyIdx(key);
    if (idx<0) {
        return false;
    }
    // Write 0 data
    nvmUnlock();
    for(int i=0;i<_cfg.indexTable[idx].len;i++) {
        nvmWrite8(_cfg.indexTable[idx].off + i, 0);
    }
    nvmLock();

    return true;
}

// Internals
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
    // Calculate where next free space in store it while reading the key index into ram
    for(int i=0;i<_cfg.nbKeys; i++) {
        _cfg.indexTable[i].key = nvmRead16(_cfg.indexStart+(i*INDEX_SIZE));
        _cfg.indexTable[i].len = nvmRead8(_cfg.indexStart+(i*INDEX_SIZE)+2);
        _cfg.indexTable[i].off = nvmRead16(_cfg.indexStart+(i*INDEX_SIZE)+3);
        _cfg.storeOffset = _cfg.indexTable[i].off + _cfg.indexTable[i].len;
    }
    nvmLock();

    // ready to roll
}

/** Adding new key
 * If nbKeys>=200, fail
 * Write index entry to PROM_START+IdxStart+(nbKeys*5) [K, L, StoreOffset]
 * StoreOffset+=len
 * nbKeys++
 * write nbKeys(sec)
 * write nbKeys(pri)
 * Returns index in the table of the new key
 */
static int createKey(uint16_t k, uint8_t l) {
    assert(l!=0);
    if (_cfg.nbKeys>=MAX_KEYS) {
        return -1;       // no joy
    }
    if (_cfg.storeOffset+l > nvmSize()) {
        return -1;         // full up
    }
    int ret = _cfg.nbKeys;
    nvmUnlock();
    // Wrtie to PROM new index entry
    nvmWrite16(_cfg.indexStart+ret*INDEX_SIZE, k);
    nvmWrite8(_cfg.indexStart+ret*INDEX_SIZE+2, l);
    nvmWrite16(_cfg.indexStart+ret*INDEX_SIZE+3, _cfg.storeOffset);
    // update memory copy
    _cfg.indexTable[ret].key = k;
    _cfg.indexTable[ret].len = l;
    _cfg.indexTable[ret].off = _cfg.storeOffset;

    // Move next free space in store along    
    _cfg.storeOffset+=l;
    _cfg.nbKeys++;
    // Update number of key in index in PROM
    nvmWrite8(1, _cfg.nbKeys);
    nvmWrite8(0, _cfg.nbKeys);
    nvmLock();

    return ret;
}

// Find the index in the key table for the given key, or -1 if not found
static int findKeyIdx(uint16_t k) {
    for(int i=0;i<_cfg.nbKeys; i++) {
        if (_cfg.indexTable[i].key == k) {
            return i;
        }
    }
    return -1;
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
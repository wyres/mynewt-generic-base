/**
 * Wyres private code
 * Config manager
 * Allows definition of config elements which are opaque and up to 255 bytes long
 * These are stored/retrieved from either FLASH or PROM
 */
#include "os/os.h"

#include "wyres-generic/wutils.h"

#include "wyres-generic/configmgr.h"

// See eeprom_board.c in src/board/iM880C for basic access methods eg EEPROM_SaveConfiguration() (but do it much better)

// Add a new element definition key. If the key is already known AND has the same key, this is a noop. 
// If the key exists but has a different length, false is returned.
// If the key is unknown, it is added to the dictionary and the value is set to that of initdata.
bool CFMgr_addElementDef(uint16_t key, uint8_t len, void* initdata) {
    return false;
}
bool CFMgr_getOrAddElement(uint16_t key, void* data, uint8_t len) {
    return false;
}
uint8_t CFMgr_getElementLen(uint16_t key) {
    return -1;
}
// Set an element value. 
bool CFMgr_setElement(uint16_t key, void* data, uint8_t len) {
    return false;
}
bool CFMgr_resetElement(uint16_t key) {
    return false;
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
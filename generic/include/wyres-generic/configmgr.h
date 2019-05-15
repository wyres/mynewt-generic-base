#ifndef H_CONFIGMGR_H
#define H_CONFIGMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CFGKEY(__m, __k) (((__m & 0xff) << 8) | (__k & 0xff))

bool CFMgr_addElementDef(uint16_t key, uint8_t len, void* initdata);
bool CFMgr_getOrAddElement(uint16_t key, void* data, uint8_t len);
uint8_t CFMgr_getElementLen(uint16_t key);
bool CFMgr_setElement(uint16_t key, void* data, uint8_t len);
bool CFMgr_resetElement(uint16_t key);

// Define module ids here as unique values 1-255. Module 0 is for basic untilites (who can manage their keys between them..)
#define CFG_MODULE_UTIL 0
#define CFG_MODULE_LORA 1
#define CFG_MODULE_APP 2

// Config keys used in the wyres utils
#define CFG_UTIL_KEY_REBOOTREASON CFGKEY(CFG_MODULE_UTIL, 1)
#define CFG_UTIL_KEY_ASSERTCALLERLOC CFGKEY(CFG_MODULE_UTIL, 2)
#define CFG_UTIL_KEY_ASSERTCALLERFN CFGKEY(CFG_MODULE_UTIL, 3)

#ifdef __cplusplus
}
#endif

#endif  /* H_CONFIGMGR_H */

#ifndef H_APPCONFIGKEYS_H
#define H_APPCONFIGKEYS_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

// Our module defined in configMgr (APP)
// Each config element must have a unique key
#define CFG_UTIL_KEY_CHECKINTERVAL CFGKEY(CFG_MODULE_APP, 1)

#ifdef __cplusplus
}
#endif

#endif  /* H_APPCONFIGKEYS_H */

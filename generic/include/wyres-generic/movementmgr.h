#ifndef H_MOVEMENTMGR_H
#define H_MOVEMENTMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { UPRIGHT, INVERTED, FLAT_BACK, FLAT_FACE, UNKNOWN} MM_ORIENT;
typedef void (*MM_CBFN_t)(void);

bool MMMgr_register(MM_CBFN_t cb);
uint32_t MMMgr_getLastMovedTime();
bool MMMgr_hasMovedSince(uint32_t reltime);
uint32_t MMMgr_getLastFallTime();
bool MMMgr_hasFallenSince(uint32_t reltime);
uint32_t MMMgr_getLastShockTime();
bool MMMgr_hasShockedSince(uint32_t reltime);
MM_ORIENT MMMgr_getOrientation();
// in units of G/10
int8_t MMMgr_getX();
int8_t MMMgr_getY();
int8_t MMMgr_getZ();

#ifdef __cplusplus
}
#endif

#endif  /* H_MOVEMENTMGR_H */

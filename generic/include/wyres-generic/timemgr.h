#ifndef H_TIMEMGR_H
#define H_TIMEMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t TMMgr_getRelTime();
uint32_t TMMgr_getTime();
void TMMgr_setBootTime(uint32_t t);
uint32_t TMMgr_busySleep(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif  /* H_TIMEMGR_H */

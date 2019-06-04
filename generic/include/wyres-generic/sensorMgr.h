#ifndef H_SENSORMGR_H
#define H_SENSORMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif


void SRMgr_start();
void SRMgr_stop();
uint32_t SRMgr_getTemp();
uint32_t SRMgr_getPressure();
uint32_t SRMgr_getBattery();
uint32_t SRMgr_getLight();
uint32_t SRMgr_getButtonMask();

#ifdef __cplusplus
}
#endif

#endif  /* H_SENSORMGR_H */

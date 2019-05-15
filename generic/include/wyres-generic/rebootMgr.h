#ifndef H_REBOOTMGR_H
#define H_REBOOTMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

const char* RMMgr_getResetReason();
void RMMgr_reboot(uint8_t reason);
void RMMgr_saveAssertCaller(void* fnaddr);
uint32_t RMMgr_getLastAssertCallerLoc();
void* RMMgr_getLastAssertCallerFn();

enum RM_reason { RM_ASSERT, RM_WATCHDOG, RM_RADIO_ERR, RM_HW_ERR };

#ifdef __cplusplus
}
#endif

#endif  /* H_REBOOTMGR_H */

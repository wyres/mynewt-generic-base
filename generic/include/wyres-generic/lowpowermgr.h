#ifndef H_LOWPOWER_H
#define H_LOWPOWER_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

// Low power sleep modes, in order of increasing sleepness (ie lower power coz more stuff is off)
typedef enum  { LP_RUN, LP_DOZE, LP_SLEEP, LP_DEEPSLEEP, LP_OFF } LP_MODE;
typedef void (*LP_CBFN_t)(LP_MODE prevmode, LP_MODE newmode);
// Register to be told when entering or leaving sleep
void LPMgr_register(LP_CBFN_t cb);
// THe level of sleeping when someone asks to enter low power mode
void LPMgr_setLPMode(LP_MODE m);
// goto sleep until either an IRQ/timer (if enabled in the mode used) or (max) the wakeupTimeoutMs time has passed
void LPMgr_sleep(uint32_t wakeupTimeoutMs);

#ifdef __cplusplus
}
#endif

#endif  /* H_LOWPOWER_H */

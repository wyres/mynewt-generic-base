/**
 * Wyres private code
 * Low power manager. Used to change power modes
 * Can register callbacks to inform of changes to mode, eg GPIO mgr will do this to disable GPIOS etc
 */
#include "os/os.h"

#include "wyres-generic/wutils.h"

#include "wyres-generic/lowpowermgr.h"

#define MAX_LPCBFNS MYNEWT_VAL(MAX_LPCBFNS)

// Registered callbacks fns
static struct lp_ctx {
    LP_CBFN_t devices[MAX_LPCBFNS];        // TODO should be a mempool
    uint8_t deviceCnt;
    LP_MODE sleepMode;
    LP_MODE curMode;
} _ctx = {
    .deviceCnt=0,
    .sleepMode=LP_SLEEP,
    .curMode=LP_RUN,
};

// Register to be told when we change mode
void LPMgr_register(LP_CBFN_t cb) {
    // TODO add to list
    assert(_ctx.deviceCnt < MAX_LPCBFNS);
    _ctx.devices[_ctx.deviceCnt++]=cb;
}
// THe level of sleeping when someone (OS) asks to enter 'low power mode'
void LPMgr_setLPMode(LP_MODE m) {
    _ctx.sleepMode = m;
    // Set up MCU ready for a WKI instruction (for OS that doesn't call LPMgr_sleep())
}
// goto sleep until either an IRQ/timer (if enabled in the mode used) or (max) the wakeupTimeoutMs time has passed
void LPMgr_sleep(uint32_t wakeupTimeoutMs) {
    // todo
    // Should call fns in bsp really as this is board/mcu dependant
    // see    LPM_Enter() called from main.c in old code, code in boards/SK-iM880C/lowpower_board.c
}



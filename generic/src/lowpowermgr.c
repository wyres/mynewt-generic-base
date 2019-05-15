/**
 * Wyres private code
 * Low power manager. Used to change power modes
 * Can register callbacks to inform of changes to mode, eg GPIO mgr will do this to disable GPIOS etc
 */
#include "os/os.h"

#include "wyres-generic/wutils.h"

#include "wyres-generic/lowpowermgr.h"

#define FNUM FN_LOWPOWERMGR

#define MAX_LPCBFNS MYNEWT_VAL(MAX_LPCBFNS)

// Registered callbacks fns
static LP_CBFN_t _devices[MAX_LPCBFNS];        // TODO should be a mempool
static LP_MODE _curMode=LP_RUN;

void LPMgr_register(LP_CBFN_t cb) {
    // TODO add to list
    _devices[0]=cb;
}

void LPMgr_run() {
    _curMode = LP_RUN;
}
void LPMgr_deepsleep() {
    _curMode = LP_DEEPSLEEP;
}


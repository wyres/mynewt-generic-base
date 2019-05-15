#ifndef H_LOWPOWER_H
#define H_LOWPOWER_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum  { LP_RUN, LP_DOZE, LP_SLEEP, LP_DEEPSLEEP, LP_OFF } LP_MODE;
typedef void (*LP_CBFN_t)(LP_MODE prevmode, LP_MODE newmode);
void LPMgr_register(LP_CBFN_t cb);

#ifdef __cplusplus
}
#endif

#endif  /* H_LOWPOWER_H */

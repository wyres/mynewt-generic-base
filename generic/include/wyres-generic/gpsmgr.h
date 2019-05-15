#ifndef H_GPSMGR_H
#define H_GPSMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gps_data {
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t prec;      // recision in m. -1 means the fix is invalid
    uint8_t nSats;      // number of satellites used for this fix
    uint32_t rxAt;      // timestamp in ms since boot of when this position was updated
} gps_data_t;
typedef enum { GPS_LOCK, GPS_UNLOCK, GPS_FAIL, GPS_NEWFIX } GPS_EVENT_TYPE_t;
typedef void (*GPS_CB_FN_t)(GPS_EVENT_TYPE_t e);

void gps_mgr_init(const char* dname, int8_t pwrPin, int8_t uartSelect);

bool getGPSData(gps_data_t* d);
void gps_start(GPS_CB_FN_t cb);
void gps_stop();

#ifdef __cplusplus
}
#endif

#endif  /* H_GPSMGR_H */

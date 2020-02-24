/**
 * Copyright 2019 Wyres
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, 
 * software distributed under the License is distributed on 
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
 * either express or implied. See the License for the specific 
 * language governing permissions and limitations under the License.
*/
#ifndef H_GPSMGR_H
#define H_GPSMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { POWER_ONOFF, POWER_ALWAYSON, POWER_ONSTANDBY } GPS_POWERMODE_t;

typedef struct gps_data {
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t prec;      // precision in 0.1m. -1 means the fix is invalid
    uint32_t rxAt;      // timestamp in secs since boot of when this position was updated
    uint8_t nSats;      // number of satellites used for this fix
} gps_data_t;
/** status updates. Also may be used to give final result of the gps, hence ensure values don't change */
typedef enum { GPS_COMM_OK=0, GPS_COMM_FAIL=1, GPS_NO_FIX=2, GPS_SATOK, GPS_SATLOSS, GPS_NEWFIX, GPS_DONE } GPS_EVENT_TYPE_t;
typedef void (*GPS_CB_FN_t)(GPS_EVENT_TYPE_t e);

void gps_mgr_init(const char* dname, uint32_t baudrate, int8_t pwrPin, int8_t uartSelect);

void gps_setPowerMode(GPS_POWERMODE_t m);
/* get just the latest precision */
int32_t gps_getCurrentPrecision();
bool gps_getData(gps_data_t* d);
/* time in secs since boot of last time we got a good gps fix. 0 if never had one. */
uint32_t gps_lastGPSFixTimeSecs();
// Get age of the last fix we got, or -1 if never had a fix
int32_t gps_lastGPSFixAgeMins();

void gps_start(GPS_CB_FN_t cb, uint32_t tsecs);
void gps_stop();        // Note, wait for GPS_DONE event before using uart for anything else

#ifdef __cplusplus
}
#endif

#endif  /* H_GPSMGR_H */

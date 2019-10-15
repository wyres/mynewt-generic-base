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
#ifndef H_WBLEMGR_H
#define H_WBLEMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { WBLE_COMM_OK, WBLE_COMM_FAIL, WBLE_SCAN_RX_IB, WBLE_IB_CONN, WBLE_IB_DISC } WBLE_EVENT_t;
typedef struct ibeacon_data {
//    uint8_t uuid[16];     // Not currently available or useful
    uint16_t major;
    uint16_t minor;
    int8_t rssi;
    uint8_t extra;
} ibeacon_data_t;
typedef void (*WBLE_CB_FN_t)(WBLE_EVENT_t e, ibeacon_data_t* b);

void* wble_mgr_init(const char* dname, uint32_t baudrate, int8_t pwrPin, int8_t uartSelect);

// start connection
void wble_start(void* ctx, WBLE_CB_FN_t cb);
// Stop operation
void wble_stop(void* ctx);
// Ask for ibeacon scanning to start, giving callback for data as received
void wble_scan_start(void* ctx, const uint8_t* uuid, uint16_t majorStart, uint16_t majorEnd);
void wble_scan_stop(void* ctx);
// configure ibeacon operation (when not scanning)
void wble_ibeacon_start(void* ctx, const uint8_t* uuid, uint16_t maj, uint16_t min, uint8_t extra);
void wble_ibeacon_stop(void* ctx);
// get number of ibs we have seen so far
int wble_getNbIB(void* ctx);
// get the list of IBs (best to do this once stopped)
ibeacon_data_t* wble_getIBList(void* ctx, int* sz);
// copy out sorted list of beacons up to sz elements into caller provided list
int wble_getSortedIBList(void* ctx, int sz, ibeacon_data_t* list);
#ifdef __cplusplus
}
#endif

#endif  /* H_WBLEMGR_H */

#ifndef H_WBLEMGR_H
#define H_WBLEMGR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct ibeacon_data {
    uint8_t uuid[16];
    uint16_t major;
    uint16_t minor;
    int8_t rssi;
    uint8_t extra;
} ibeacon_data_t;
typedef void (*WBLE_CB_FN_t)(ibeacon_data_t* b);

void wble_mgr_init(const char* dname, int8_t pwrPin, int8_t uartSelect);

// Get current list of ibeacons. Returns 0 for sucess, -1 for failure
int getBeaconList(uint8_t* sz, ibeacon_data_t** list);
// start connection
void wble_start();
// Stop operation
void wble_stop();
// Ask for ibeacon scanning to start, giving callback for data as received
void wble_scan_start(const char* uuid, WBLE_CB_FN_t cb);
void wble_scan_stop();
// configure ibeacon operation (when not scanning)
void wble_ibeacon_start(const char* uuid, uint16_t maj, uint16_t min, uint8_t extra);
void wble_ibeacon_stop();

#ifdef __cplusplus
}
#endif

#endif  /* H_WBLEMGR_H */

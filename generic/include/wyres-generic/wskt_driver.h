#ifndef H_WSKT_DRIVER_H
#define H_WSKT_DRIVER_H

#include <stdint.h>
#include <mcu/mcu.h>

#include "os/os_eventq.h"
#include "wskt_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_WKST_DNAME_SZ (32)

typedef struct wskt_devicefns {
    int (*open)(wskt_t* s);
    int (*ioctl)(wskt_t* s, wskt_ioctl_t* cmd);
    int (*write)(wskt_t* s, uint8_t* data, uint32_t sz);
    int (*close)(wskt_t*s);
} wskt_devicefns_t;


typedef struct wskt_device {
    wskt_devicefns_t* device_fns;
    void* device_cfg;
    char dname[MAX_WKST_DNAME_SZ];
} wskt_device_t;

#define WSKT_DEVICE_CFG(skt) (((wskt_device_t*)(skt->dev))->device_cfg)
#define WSKT_DEVICE_FNS(skt) (((wskt_device_t*)(skt->dev))->device_fns)

// DEVICE API
// To register devices at init
void wskt_registerDevice(const char* device, wskt_devicefns_t* dfns, void* dcfg);
// get open sockets on my device - caller gives an array of pointers of size bsz to copy them into
uint8_t wskt_getOpenSockets(const char* device, wskt_t** sbuf, uint8_t bsz);

#ifdef __cplusplus
}
#endif

#endif  /* H_WSKT_DRIVER_H */

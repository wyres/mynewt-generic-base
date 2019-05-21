#ifndef H_WSKT_COMMON_H
#define H_WSKT_COMMON_H

#include <stdint.h>
#include <mcu/mcu.h>

#include "os/os.h"

#ifdef __cplusplus
extern "C" {
#endif

// This is how big your buffer should be at a minimum in the event you use to open a socket
#define WSKT_BUF_SZ MYNEWT_VAL(WSKT_BUF_SZ)

/* api for socket-like devices */
#define SKT_NOERR   (0)
#define SKT_NOSPACE   (-1)
#define SKT_NODEV   (-2)
#define SKT_EINVAL   (-3)

typedef struct wskt {
    void* dev;          // wskt_device_t* for the driver
    struct os_event* evt;
    struct os_eventq* eq;
} wskt_t;

typedef enum { IOCTL_PWRON, IOCTL_PWROFF, IOCTL_RESET, IOCTL_SET_BAUD, IOCTL_FILTERASCII } wskt_ioctl_cmd;
typedef struct wskt_ioctl {
    wskt_ioctl_cmd cmd;
    uint32_t param;
} wskt_ioctl_t;

#ifdef __cplusplus
}
#endif

#endif  /* H_WSKT_COMMON_H */

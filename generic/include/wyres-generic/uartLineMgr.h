#ifndef H_UARTLINEMGR_H
#define H_UARTLINEMGR_H

#include <inttypes.h>
#include <mcu/mcu.h>

#include "os/os_eventq.h"
#include "wskt_common.h"

#ifdef __cplusplus
extern "C" {
#endif
#define UART_LINE_SZ (WSKT_BUF_SZ)

// Create a device for a line access to UART
bool uart_line_comm_create(char* dname, uint32_t baudrate);

#ifdef __cplusplus
}
#endif

#endif  /* H_UARTLINEMGR_H */

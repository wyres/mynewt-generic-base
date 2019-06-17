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
#ifndef H_WSKT_USER_H
#define H_WSKT_USER_H

#include <inttypes.h>
#include <mcu/mcu.h>

#include "os/os_eventq.h"
#include "wskt_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// APP API : access devices via socket like ops
// open new socket to a device instance. If NULL rturned then the device is not accessible
// The evt must have its arg pointing to the correct thing for this device eg a buffer to receive into
wskt_t* wskt_open(const char* device, struct os_event* evt, struct os_eventq* eq);
// configure specific actions on the device. Conflictual commands from multiple sockets are not advised... as far as possible they will mediated eg power off...
int wskt_ioctl(wskt_t* skt, wskt_ioctl_t* cmd);
// Send data to the device. This will be interleaved with other open sockets on the same device on a block basis
int wskt_write(wskt_t* skt, uint8_t* data, uint32_t sz);
// indicate done using this device. Your skt variable will be set to NULL after to avoid any unpleasentness
int wskt_close(wskt_t** skt);

#ifdef __cplusplus
}
#endif

#endif  /* H_WSKT_USER_H */

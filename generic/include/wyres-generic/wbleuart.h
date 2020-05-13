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
#ifndef H_WBLEUART_H
#define H_WBLEUART_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { WBLEUART_COMM_OK, WBLEUART_COMM_FAIL, WBLEUART_RX, WBLEUART_COMM_DISC } WBLEUART_EVENT_t;
typedef void (*WBLEUART_CB_FN_t)(WBLEUART_EVENT_t e, void* d);

// One time create the device on specific uart connected to a BLE module. Return is identifier for the connection to be passed for other calls.
void* wbleuart_create(const char* dname, uint32_t baudrate, int8_t pwrPin, int8_t uartSelect);
// Open request for uarting to a remote BLE (via NUS), giving callback to get status and rx data
void wbleuart_line_open(void* c, WBLEUART_CB_FN_t cb);
// Write data to remote
int wbleuart_line_write(void* c, uint8_t* data, uint32_t sz);
// Close down connection
void wbleuart_line_close(void* c);

#ifdef __cplusplus
}
#endif

#endif  /* H_WBLEUART_H */

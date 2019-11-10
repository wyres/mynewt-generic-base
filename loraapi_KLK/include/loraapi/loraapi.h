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
#ifndef H_LORAAPI_H
#define H_LORAAPI_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif


/* New api */
typedef enum { LORAWAN_RES_OK, LORAWAN_RES_JOIN_OK, LORAWAN_RES_NOT_JOIN, LORAWAN_RES_NO_RESP, LORAWAN_RES_DUTYCYCLE, 
                LORAWAN_RES_NO_BW, LORAWAN_RES_OCC, LORAWAN_RES_HWERR, LORAWAN_RES_FWERR, LORAWAN_RES_TIMEOUT, LORAWAN_RES_BADPARAM } LORAWAN_RESULT_t;
typedef enum { LORAWAN_SF12=12, LORAWAN_SF11=11, LORAWAN_SF10=10, LORAWAN_SF9=9, LORAWAN_SF8=8, LORAWAN_SF7=7,	LORAWAN_FSK250=5, LORAWAN_SF_USEADR=13, LORAWAN_SF_DEFAULT=14 } LORAWAN_SF_t;
typedef void* LORAWAN_REQ_ID_t;     // A request id. NULL means the request was failed
typedef void (*LORAWAN_JOIN_CB_t)(void* userctx, LORAWAN_RESULT_t res);
typedef void (*LORAWAN_TX_CB_t)(void* userctx, LORAWAN_RESULT_t res);
typedef void (*LORAWAN_RX_CB_t)(void* userctx, LORAWAN_RESULT_t res, uint8_t port, int rssi, int snr, uint8_t* msg, uint8_t sz);

void lora_api_init(uint8_t* devEUI, uint8_t* appEUI, uint8_t* appKey);   // Called from app

bool lora_api_isJoined();

 // Do the join (if already joined, returns this status)
LORAWAN_RESULT_t lora_api_join(LORAWAN_JOIN_CB_t callback, LORAWAN_SF_t sf, void* userctx);

// register callback to deal with packets received on specific port (or -1 for all ports)
// data buffer given during callback will be valid only during callback (which must not block)
// Returns result code. To cancel this registration at a later point use the lora_api_cancelRxCB() with the same port/cbfn
LORAWAN_RESULT_t lora_api_registerRxCB(int port, LORAWAN_RX_CB_t callback, void* userctx);	// calls the cb whenever pkt rxed (whether on classA, B, or C)
void lora_api_cancelRxCB(int port, LORAWAN_RX_CB_t callback);

// request an UL. This will be sent 'as soon as possible' async to this call
// data buffer should be maintained as-is until the callback happens to release it
// The returned result indicates if the request is accepted or not (eg because join not done, or the radio is already in use 
// (or scheduled to be in use in the time frame of the tx/rx exchange))
LORAWAN_RESULT_t lora_api_send(LORAWAN_SF_t sf, uint8_t port, bool reqAck, bool doRx, 
                uint8_t* data, uint8_t sz, LORAWAN_TX_CB_t callback, void* userctx);

// Schedule direct radio tx access for specific time
LORAWAN_REQ_ID_t lora_api_radio_tx(uint32_t abs_time, LORAWAN_SF_t sf, uint32_t freq, int txpower, uint8_t* data, uint8_t sz, LORAWAN_TX_CB_t callback, void* userctx);
// Schedule direct radio rx access for specific time
LORAWAN_REQ_ID_t lora_api_radio_rx(uint32_t abs_time, LORAWAN_SF_t sf, uint32_t freq, uint32_t timeoutms, uint8_t* data, uint8_t sz, LORAWAN_RX_CB_t callback, void* userctx);

// Cancel a pending radio direct request. True if cancelled without action, false if already in progress and cannot be cancelled
bool lora_api_cancel(LORAWAN_REQ_ID_t id);

#ifdef __cplusplus
}
#endif

#endif  /* H_LORAAPI_H */

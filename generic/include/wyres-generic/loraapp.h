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
#ifndef H_LORAAPP_H
#define H_LORAAPP_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { LORA_TX_OK, LORA_TX_OK_ACKD, LORA_TX_TIMEOUT, LORA_TX_ERR_RETRY, LORA_TX_ERR_NOTJOIN, LORA_TX_ERR_FATAL } LORA_TX_RESULT_t;
typedef void (*LORA_RES_CB_FN_t)(LORA_TX_RESULT_t e);
typedef void (*LORA_RX_CB_FN_t)(uint8_t port, void* data, uint8_t sz);

void lora_app_init(LORA_RES_CB_FN_t txcb, LORA_RX_CB_FN_t rxcb);
void lora_app_setAck(bool useAck);
void lora_app_setAdr(bool useAdr);
void lora_app_setTxPort(uint8_t p);
void lora_app_setRxPort(uint8_t p);
void lora_app_setDR(int8_t d);
void lora_app_setTxPower(int8_t p);
void lora_app_setDevEUI(uint8_t* d);
void lora_app_setAppEUI(uint8_t* d);
void lora_app_setAppKey(uint8_t* d);

bool lora_app_join();
bool lora_app_isJoined();
// tx a buffer. calls the callback fn (in init()) with result : 
LORA_TX_RESULT_t lora_app_tx(uint8_t* data, uint8_t sz, uint32_t timeoutMs);

// Our module defined in configMgr (APP)
// Each config element must have a unique key
#define CFG_UTIL_KEY_DEVEUI CFGKEY(CFG_MODULE_LORA, 1)
#define CFG_UTIL_KEY_APPEUI CFGKEY(CFG_MODULE_LORA, 2)
#define CFG_UTIL_KEY_APPKEY CFGKEY(CFG_MODULE_LORA, 3)
#define CFG_UTIL_KEY_DEVADDR CFGKEY(CFG_MODULE_LORA, 4)
#define CFG_UTIL_KEY_NWKSKEY CFGKEY(CFG_MODULE_LORA, 5)
#define CFG_UTIL_KEY_APPSKEY CFGKEY(CFG_MODULE_LORA, 6)
#define CFG_UTIL_KEY_ADREN CFGKEY(CFG_MODULE_LORA, 7)
#define CFG_UTIL_KEY_ACKEN CFGKEY(CFG_MODULE_LORA, 8)
#define CFG_UTIL_KEY_DR CFGKEY(CFG_MODULE_LORA, 9)
#define CFG_UTIL_KEY_TXPOWER CFGKEY(CFG_MODULE_LORA, 10)
#define CFG_UTIL_KEY_TXPORT CFGKEY(CFG_MODULE_LORA, 11)
#define CFG_UTIL_KEY_RXPORT CFGKEY(CFG_MODULE_LORA, 12)

#ifdef __cplusplus
}
#endif

#endif  /* H_LORAAPP_H */

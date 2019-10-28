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

#include <string.h>

#include "os/os.h"

#include "wyres-generic/wutils.h"
#include "lora/loraapi.h"
#include "LoRaMac.h"

#define LORAAPI_TASK_PRIO       MYNEWT_VAL(LORAAPI_TASK_PRIO)
#define LORAAPI_TASK_STACK_SZ   OS_STACK_ALIGN(256)

// How many people can wait for something at same time? also indicates how many outstanding requests can exist together
#define MAX_JOINCBS   (1)       // app can have multiple branchs that join but its not recommended
#define MAX_TXCBS   (4)         // Can q up to 4 tx requests
#define MAX_RXCBS   (2)         // can register 2 rx callbacks on different ports
#define MAX_TXRADIOCBS   (1)    // as not yet implemented
#define MAX_RXRADIOCBS   (1)    // or maybe coz don't need this fn

#define MAX_LWEVTS  (4)     // only 4 outstanding lorawan->task events at any time
typedef enum { LWEVT_TYPE_UNUSED, LWEVT_TYPE_MCPS_CONFIRM, LWEVT_TYPE_MCPS_INDICATION, LWEVT_TYPE_MLME_CONFIRM, LWEVT_TYPE_MLME_INDICATION } LWEVT_TYPE_t;
typedef struct {
    LWEVT_TYPE_t type;
    union {
        McpsConfirm_t mcpsc;
        McpsIndication_t mcpsi;
        MlmeConfirm_t mlmec;
        MlmeIndication_t mlmei;
    } msg;
} LWEVT_t;


static struct loraapi_ctx {
    struct {
        LORAWAN_JOIN_CB_t cbfn;
        void* userctx;
    } joinLoraWANReqs[MAX_JOINCBS];
    struct {
        LORAWAN_TX_CONTRACT_t contract;
        LORAWAN_SF_t sf;
        int8_t power;
        uint8_t port;
        uint8_t* data;
        uint8_t sz;
        LORAWAN_TX_CB_t cbfn;
        void* userctx;
    } txLoraWANReqs[MAX_TXCBS];
    struct {
        uint8_t port;
        LORAWAN_RX_CB_t cbfn;
        void* userctx;
    } rxLoraWANReqs[MAX_RXCBS];
    struct {
        uint32_t atTxMS;
        int8_t power;
        uint32_t freq;
        LORAWAN_SF_t sf;
        uint8_t* data;
        uint8_t sz;
        LORAWAN_TX_CB_t cbfn;
        void* userctx;        
    } txRadioReqs[MAX_TXRADIOCBS];
    struct {
        uint32_t atRxMS;
        uint32_t tRxMS;
        uint32_t freq;
        LORAWAN_SF_t sf;
        uint8_t* data;
        uint8_t sz;
        LORAWAN_RX_CB_t cbfn;
        void* userctx;        
    } rxRadioReqs[MAX_RXRADIOCBS];
    uint8_t deveui[8];
    uint8_t appeui[8];
    uint8_t appkey[16];
    os_stack_t loraapi_task_stack[LORAAPI_TASK_STACK_SZ];
    struct os_task loraapi_task_str;
    struct os_eventq lwevt_q;
    // lwevt pool
    struct {
        struct os_event e;
        LWEVT_t lwevt;
    } lwevts[MAX_LWEVTS];
    struct os_mutex lwevts_mutex;
    uint32_t noEventCnt;        // could each time we have event pool starvation
} _loraCtx = {
//    .deveui = {0x38,0xE8,0xEB,0xE0, 0x00, 0x00, 0x0d, 0x78},
    .deveui = {0x38,0xb8,0xeb,0xe0, 0x00, 0x00, 0xFF, 0xFF},
//    .appeui = {0x01,0x23,0x45,0x67, 0x89, 0xab, 0xcd, 0xef},
//    .appeui = {0x38,0xE8,0xEB,0xEA, 0xBC, 0xDE, 0xF0, 0x42},
    .appeui = {0x38,0xE8,0xEB,0xE0, 0x00, 0x00, 0x00, 0x00},
    .appkey = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF},
    .noEventCnt=0,
};

static void loraapi_task(void* data);

/** API implementation */

bool lora_api_isJoined() {
    return false;
}

 // Do the join (if already joined, returns this status to the callback)
LORAWAN_RESULT_t lora_api_join(LORAWAN_JOIN_CB_t callback, LORAWAN_SF_t sf, void* userctx) {
    if (lora_api_isJoined()) {
        // fake a join accept to do event to do callback to say joined???
        // TODO
        return LORAWAN_RES_OK;
    }
    // Start join process
    // Check for a free slot
    for(int i=0;i<MAX_RXCBS;i++) {
        if (_loraCtx.joinLoraWANReqs[i].cbfn==NULL) {
            _loraCtx.joinLoraWANReqs[i].cbfn = callback;
            _loraCtx.joinLoraWANReqs[i].userctx = userctx;
            // kick off join request
            // TODO
            return LORAWAN_RES_OK;
        }
    }
    // soz
    return LORAWAN_RES_NOT_JOIN;
}

// register callback to deal with packets received on specific port (or -1 for all ports)
// data buffer given during callback will be valid only during callback (which must not block)
// Returns request id, which can be used to cancel this registration at a later point
LORAWAN_REQ_ID_t lora_api_registerRxCB(int port, LORAWAN_RX_CB_t callback, void* userctx) {	// calls the cb whenever pkt rxed (whether on classA, B, or C)
    // Check for a free slot
    for(int i=0;i<MAX_RXCBS;i++) {
        if (_loraCtx.rxLoraWANReqs[i].cbfn==NULL) {
            _loraCtx.rxLoraWANReqs[i].cbfn = callback;
            _loraCtx.rxLoraWANReqs[i].userctx = userctx;
            _loraCtx.rxLoraWANReqs[i].port = port;
            return &_loraCtx.rxLoraWANReqs[i];
        }
    }
    return NULL;
}

// schedule a tx for time determined by <contract>
// contract is ctype (eg anytime, within next X, at absolute/relative time), and time
// data is COPIED during call, caller can reuse buffer
// Returns id for the tx (will be used in callback) or NULL if not scheduled because tx queue is full
LORAWAN_REQ_ID_t lora_api_send(LORAWAN_SF_t sf, uint8_t port, LORAWAN_TX_CONTRACT_t contract, 
                uint8_t* data, uint8_t sz, LORAWAN_TX_CB_t callback, void* userctx) {
    // Check for a free slot
    for(int i=0;i<MAX_TXCBS;i++) {
        if (_loraCtx.txLoraWANReqs[i].cbfn==NULL) {
            _loraCtx.txLoraWANReqs[i].cbfn = callback;
            _loraCtx.txLoraWANReqs[i].userctx = userctx;
            _loraCtx.txLoraWANReqs[i].port = port;
            _loraCtx.txLoraWANReqs[i].sf = sf;
            _loraCtx.txLoraWANReqs[i].power = 14;       // TODO
            _loraCtx.txLoraWANReqs[i].data = data;
            _loraCtx.txLoraWANReqs[i].sz = sz;
            _loraCtx.txLoraWANReqs[i].contract = contract;
            // tell task to schedule
            // TODO
            return &_loraCtx.txLoraWANReqs[i];
        }
    }
    return NULL;
}

// Schedule direct radio tx access for specific time
LORAWAN_REQ_ID_t lora_api_radio_tx(uint32_t abs_time, LORAWAN_SF_t sf, uint32_t freq, int txpower, uint8_t* data, uint8_t sz, LORAWAN_TX_CB_t callback, void* userctx) {
    // Check for a free slot
    for(int i=0;i<MAX_TXRADIOCBS;i++) {
        if (_loraCtx.txRadioReqs[i].cbfn==NULL) {
            _loraCtx.txRadioReqs[i].cbfn = callback;
            _loraCtx.txRadioReqs[i].userctx = userctx;
            _loraCtx.txRadioReqs[i].data = data;
            _loraCtx.txRadioReqs[i].sz = sz;
            _loraCtx.txRadioReqs[i].sf = sf;
            _loraCtx.txRadioReqs[i].freq = freq;
            _loraCtx.txRadioReqs[i].power = txpower;
            _loraCtx.txRadioReqs[i].atTxMS = abs_time;
            return &_loraCtx.rxRadioReqs[i];
        }
    }
    return NULL;
}
// Schedule direct radio rx access for specific time
LORAWAN_REQ_ID_t lora_api_radio_rx(uint32_t abs_time, LORAWAN_SF_t sf, uint32_t freq, uint32_t timeoutms, uint8_t* data, uint8_t sz, LORAWAN_RX_CB_t callback, void* userctx) {
    // Check for a free slot
    for(int i=0;i<MAX_RXCBS;i++) {
        if (_loraCtx.rxRadioReqs[i].cbfn==NULL) {
            _loraCtx.rxRadioReqs[i].cbfn = callback;
            _loraCtx.rxRadioReqs[i].userctx = userctx;
            _loraCtx.txRadioReqs[i].data = data;
            _loraCtx.txRadioReqs[i].sz = sz;
            _loraCtx.rxRadioReqs[i].sf = sf;
            _loraCtx.rxRadioReqs[i].freq = freq;
            _loraCtx.rxRadioReqs[i].atRxMS = abs_time;
            _loraCtx.rxRadioReqs[i].tRxMS = timeoutms;
            return &_loraCtx.rxRadioReqs[i];
        }
    }
    return NULL;
}

// Cancel a pending request. True if cancelled without action, false if already in progress and cannot be cancelled
bool lora_api_cancel(LORAWAN_REQ_ID_t id) {
    return true;
}


// Internals

/* 
static bool configSocket(lorawan_sock_t skt) {
    bool ret = true;
    lorawan_status_t status;
    lorawan_attribute_t mib;
    mib.Type = LORAWAN_ATTR_ADR;
    mib.Param.AdrEnable = _loraCfg.useAdr;
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("failed to set ADR");
        ret = false;
    }
    mib.Type = LORAWAN_ATTR_MCPS_TYPE;
    if (_loraCfg.useAck) {
        mib.Param.McpsType = LORAWAN_MCPS_CONFIRMED;
    } else {
        mib.Param.McpsType = LORAWAN_MCPS_UNCONFIRMED;
    }
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("failed to set ACK");
        ret = false;
    }
    mib.Type = LORAWAN_ATTR_CHANNELS_DATARATE;
    mib.Param.ChannelsDefaultDatarate = _loraCfg.loraDR;
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("failed to set data rate");
        ret = false;
    }
    mib.Type = LORAWAN_ATTR_CHANNELS_TX_POWER;
    mib.Param.ChannelsDefaultTxPower = ((14-_loraCfg.txPower)/2);        // lorawan stack power does 0-7, no one knows why
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("failed to set tx power");
        ret = false;
    }
    return ret;
}
*/
static struct os_event* allocEvent() {
    // Find an unused one in list
    struct os_event* ret = NULL;
    // mutex access to this list for concurrent calls to allocEvent
    os_mutex_pend(&_loraCtx.lwevts_mutex, OS_TIMEOUT_NEVER);
    for(int i=0;i<MAX_LWEVTS;i++) {
        if (_loraCtx.lwevts[i].lwevt.type==LWEVT_TYPE_UNUSED) {
            // flag in use
            _loraCtx.lwevts[i].lwevt.type = LWEVT_TYPE_UNUSED;
            ret = &_loraCtx.lwevts[i].e;
            break;
        }
    }
    os_mutex_release(&_loraCtx.lwevts_mutex);
    return ret;
}

static void freeEvent(struct os_event* e) {
    os_mutex_pend(&_loraCtx.lwevts_mutex, OS_TIMEOUT_NEVER);
    ((LWEVT_t*)(e->ev_arg))->type = LWEVT_TYPE_UNUSED;
    os_mutex_release(&_loraCtx.lwevts_mutex);
}

static void loraapi_task(void* data) {
    while(1) {
        // Execute events which come from lorawan callbacks etc
        os_eventq_run( &_loraCtx.lwevt_q );
    }
    assert(0);
}

static void execEvent(struct os_event* e) {
    LWEVT_t* evt = (LWEVT_t*)(e->ev_arg);
    switch(evt->type) {
        case LWEVT_TYPE_MCPS_CONFIRM: {
            McpsConfirm_t* McpsConfirm = &evt->msg.mcpsc;
            // Confirmation of sending of app level message
            log_debug("MCPSconfirm: %d\r\n", McpsConfirm->AckReceived);
            // TODO find guy who ordered tx and callback with result

            break;
        }
        case LWEVT_TYPE_MCPS_INDICATION: {
            McpsIndication_t* McpsIndication = &evt->msg.mcpsi;

            // DL message. Call anyone registered on this port
            log_debug("MCPSind (%d/%d)\r\n", McpsIndication->Status, McpsIndication->Port);

            if( McpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK){
                log_debug("status not OK\r\n");
            } else {
#if 0
    int i;
    lorawan_printf("$ LoRaWAN Rx Data: [devAddr:%08lx] [ack:%d] [Fcnt:%lu] [Fpend:%u] [mCast:%u] [port:%u] [rssi:%d] [snr:%u] [slot:%d]\r\n",
            McpsIndication->DevAddr,
            McpsIndication->AckReceived,
            McpsIndication->DownLinkCounter,
            McpsIndication->FramePending,
            McpsIndication->Multicast,
            McpsIndication->Port,
            McpsIndication->Rssi,
            McpsIndication->Snr,
            McpsIndication->RxSlot
    );

    //print data if there are
    if(McpsIndication->BufferSize != 0){
        lorawan_printf("$ Payload Data [size:%u]:\r\n", McpsIndication->BufferSize);
        for(i=0;i<McpsIndication->BufferSize;i++)
            lorawan_printf("%02x", McpsIndication->Buffer[i]);
        lorawan_printf("\r\n");
    }
#endif
                if( McpsIndication->Port != 0 ){
                    /* Search for a valid callbacks to forward payload */

                }
                else{ 
                    /* Means that neither Fport, nor Payload, are in the frame -> ?? */

                }
            }
            break;
        }
        case LWEVT_TYPE_MLME_CONFIRM: {
            MlmeConfirm_t* MlmeConfirm = &evt->msg.mlmec;
            // Could be a JOIN accept?
            switch(MlmeConfirm->MlmeRequest) {
                case MLME_JOIN: {
                    if (MlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
                        log_debug("Join Accepted\r\n");
                        // send event to signal this
                        
                        // TODO find join cb and call it to tell app we're joined
                    } else {
                        // badness
                        log_debug("JOIN failed %d",MlmeConfirm->Status);
                    }
                    break;
                }
                default: {
                    log_debug("MLMEconf %d/%d\r\n", MlmeConfirm->MlmeRequest,
                                        MlmeConfirm->Status);
                }
            }
            break;
        }
        case LWEVT_TYPE_MLME_INDICATION: {
            MlmeIndication_t* MlmeIndication = &evt->msg.mlmei;
            log_debug("MLMEind\r\n");
            // Don't care about mac command DLs
            break;
        }
        default:{
            log_debug("unknown event:%d", evt->type);
            break;
        }
    }
    freeEvent(e);
}


/**** Lorawan stack api callbacks. These are just copied into events and posted for execution by the task. */
/* Primitive definitions used by the LoRaWAN */
static void _mcps_confirm ( McpsConfirm_t *McpsConfirm ){
    struct os_event* e = allocEvent();
    if (e!=NULL) {
        LWEVT_t* evt = (LWEVT_t*)e->ev_arg;
        // copy what stack gave us as only valid during this function
        memcpy(&evt->msg.mcpsc, McpsConfirm, sizeof(McpsConfirm_t));
        evt->type = LWEVT_TYPE_MCPS_CONFIRM;
        os_eventq_put(&_loraCtx.lwevt_q, e);
    } else {
        // oopsie
        _loraCtx.noEventCnt++;
    }
}

// Downlink
static void _mcps_indication ( McpsIndication_t *McpsIndication ){
    struct os_event* e = allocEvent();
    if (e!=NULL) {
        LWEVT_t* evt = (LWEVT_t*)e->ev_arg;
        // copy what stack gave us as only valid during this function
        memcpy(&evt->msg.mcpsi, McpsIndication, sizeof(McpsIndication_t));
        evt->type = LWEVT_TYPE_MCPS_INDICATION;
        os_eventq_put(&_loraCtx.lwevt_q, e);
    } else {
        // oopsie
        _loraCtx.noEventCnt++;
    }
}

// Confirmation of WAN MAC level request
static void _mlme_confirm( MlmeConfirm_t *MlmeConfirm ) {
    struct os_event* e = allocEvent();
    if (e!=NULL) {
        LWEVT_t* evt = (LWEVT_t*)e->ev_arg;
        // copy what stack gave us as only valid during this function
        memcpy(&evt->msg.mlmec, MlmeConfirm, sizeof(MlmeConfirm_t));
        evt->type = LWEVT_TYPE_MLME_CONFIRM;
        os_eventq_put(&_loraCtx.lwevt_q, e);
    } else {
        // oopsie
        _loraCtx.noEventCnt++;
    }

}

static void _mlme_indication( MlmeIndication_t *MlmeIndication ){
    struct os_event* e = allocEvent();
    if (e!=NULL) {
        LWEVT_t* evt = (LWEVT_t*)e->ev_arg;
        // copy what stack gave us as only valid during this function
        memcpy(&evt->msg.mlmei, MlmeIndication, sizeof(MlmeIndication_t));
        evt->type = LWEVT_TYPE_MLME_INDICATION;
        os_eventq_put(&_loraCtx.lwevt_q, e);
    } else {
        // oopsie
        _loraCtx.noEventCnt++;
    }
}

static LoRaMacPrimitives_t _lorawan_primitives = {
        _mcps_confirm,
        _mcps_indication,
        _mlme_confirm,
        _mlme_indication
};

static uint8_t _get_battery_level( void ) {
    // TODO: link this with the HAL
    return 255;
}
static LoRaMacCallback_t _lorawan_callbacks = { _get_battery_level };

static LoRaMacRegion_t lorawan_get_first_active_region( void ){
    LoRaMacRegion_t region;
    for(region=0; region != LORAMAC_REGION_US915_HYBRID; region++){
        if( RegionIsActive( region ) == true )
            return region;
    }
    return 0;
}

/*** initialisation */
// initialise lorawan stack with our config
void lora_api_init(uint8_t* devEUI, uint8_t* appEUI, uint8_t* appKey) {
    for(int i=0;i<8;i++) {
        _loraCtx.deveui[i] = devEUI[i];
    }
    for(int i=0;i<8;i++) {
        _loraCtx.appeui[i] = appEUI[i];
    }
    for(int i=0;i<16;i++) {
        _loraCtx.appkey[i] = appKey[i];
    }
    // init events (mutex, q, each event in the pool)
    os_mutex_init(&_loraCtx.lwevts_mutex);
    for(int i=0;i<MAX_LWEVTS;i++) {
        _loraCtx.lwevts[i].e.ev_cb = &execEvent;
        _loraCtx.lwevts[i].e.ev_arg = &(_loraCtx.lwevts[i].lwevt);
        _loraCtx.lwevts[i].lwevt.type = LWEVT_TYPE_UNUSED;
    }
    os_eventq_init(&_loraCtx.lwevt_q);
    os_task_init(&_loraCtx.loraapi_task_str, "lw_eventq",
                 loraapi_task, NULL,
                 LORAAPI_TASK_PRIO, OS_WAIT_FOREVER,
                 _loraCtx.loraapi_task_stack,
                 LORAAPI_TASK_STACK_SZ);

    // Initialise stackforce lorawan stack
    LoRaMacStatus_t status = LoRaMacInitialization(&_lorawan_primitives, &_lorawan_callbacks,
                                   lorawan_get_first_active_region());
}

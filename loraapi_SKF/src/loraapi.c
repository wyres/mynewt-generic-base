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

/* loraapi implmementation using direct access to the stackforce apis. */
#include <string.h>

#include "os/os.h"

#include "wyres-generic/wutils.h"
#include "loraapi/loraapi.h"
#include "LoRaMac.h"

#define LORAAPI_TASK_PRIO       MYNEWT_VAL(LORAAPI_TASK_PRIO)
#define LORAAPI_TASK_STACK_SZ   OS_STACK_ALIGN(256)

// How many people can wait for something at same time? also indicates how many outstanding requests can exist together
#define MAX_TXCBS   (4)         // Can q up to 4 tx requests
#define MAX_RXCBS   (2)         // can register 2 rx callbacks on different ports
#define MAX_TXRADIOCBS   (1)    // as not yet implemented
#define MAX_RXRADIOCBS   (1)    // or maybe coz don't need this fn
#define MAX_LORA_DATA_SZ (250)

#define MAX_LWEVTS  (4)     // only 4 outstanding lorawan->task events at any time

typedef enum { LWEVT_TYPE_UNUSED, 
    LWEVT_TYPE_MCPS_CONFIRM, LWEVT_TYPE_MCPS_INDICATION, LWEVT_TYPE_MLME_CONFIRM, LWEVT_TYPE_MLME_INDICATION,
    LWEVT_TYPE_DOJOIN, LWEVT_TYPE_DOTXLW, LWEVT_TYPE_DOTXRADIO, LWEVT_TYPE_DORXRADIO
 } LWEVT_TYPE_t;
typedef struct {
    LWEVT_TYPE_t type;
    union {
        McpsConfirm_t mcpsc;
        McpsIndication_t mcpsi;
        MlmeConfirm_t mlmec;
        MlmeIndication_t mlmei;
    } msg;
} LWEVT_t;

typedef struct {
        LORAWAN_JOIN_CB_t cbfn;
        void* userctx;
    } JoinReq_t;

typedef struct  {
        LORAWAN_TX_CONTRACT_t contract;
        struct os_callout timer;
        uint32_t createTSMS;
        LORAWAN_SF_t sf;
        int8_t power;
        uint8_t port;
        uint8_t data[MAX_LORA_DATA_SZ];         // this is a bit sucky
        uint8_t sz;
        LORAWAN_TX_CB_t cbfn;
        void* userctx;
        bool txInProgress;
    } TxLoraWanReq_t;

typedef struct  {
        int port;       // as can be -1 to say all (app) ports
        LORAWAN_RX_CB_t cbfn;
        void* userctx;
    } RxLoraWanReq_t;

typedef struct  {
        struct os_callout timer;
        uint32_t atTxMS;
        int8_t power;
        uint32_t freq;
        LORAWAN_SF_t sf;
        uint8_t* data;
        uint8_t sz;
        LORAWAN_TX_CB_t cbfn;
        void* userctx;        
    } TxRadioReq_t;

typedef struct  {
        struct os_callout timer;
        uint32_t atRxMS;
        uint32_t tRxMS;
        uint32_t freq;
        LORAWAN_SF_t sf;
        uint8_t* data;
        uint8_t sz;
        LORAWAN_RX_CB_t cbfn;
        void* userctx;        
    } RxRadioReq_t;

static struct loraapi_ctx {
    bool isJoin;
    LORAWAN_SF_t defaultSF;
    int defaultLWPower;
    JoinReq_t joinLoraWANReq;
    TxLoraWanReq_t txLoraWANReqs[MAX_TXCBS];
    RxLoraWanReq_t rxLoraWANReqs[MAX_RXCBS];
    TxRadioReq_t txRadioReqs[MAX_TXRADIOCBS];
    RxRadioReq_t rxRadioReqs[MAX_RXRADIOCBS];
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
} _loraCtx;     // Note : initialised to all 0 in init method

static struct os_event* allocEvent();
static void freeEvent(struct os_event* e);
static void loraapi_task(void* data);
static void execStackEvent(struct os_event* e);
static void execTxLora(struct os_event* e);
static void execTxRadio(struct os_event* e);
static void execRxRadio(struct os_event* e);
static uint32_t getNowMS();
static uint8_t maxSz4SF(int sf);

/** API implementation */

bool lora_api_isJoined() {
    return _loraCtx.isJoin;
}

 // Do the join (if already joined, returns this status to the callback)
LORAWAN_RESULT_t lora_api_join(LORAWAN_JOIN_CB_t callback, LORAWAN_SF_t sf, void* userctx) {
    assert(callback!=NULL);
    if (lora_api_isJoined()) {
        return LORAWAN_RES_JOIN_OK;
    }
    // Start join process
    // Check not already in progress
    if (_loraCtx.joinLoraWANReq.cbfn==NULL) {
        _loraCtx.defaultSF = sf;
        _loraCtx.joinLoraWANReq.cbfn = callback;
        _loraCtx.joinLoraWANReq.userctx = userctx;
        // kick off join request
        struct os_event* e = allocEvent();
        if (e!=NULL) {
            LWEVT_t* evt = (LWEVT_t*)e->ev_arg;
            evt->type = LWEVT_TYPE_DOJOIN;
            os_eventq_put(&_loraCtx.lwevt_q, e);
            return LORAWAN_RES_OK;
        } else {
            // oopsie
            _loraCtx.noEventCnt++;
            return LORAWAN_RES_DUTYCYCLE;
        }
    } else {
        return LORAWAN_RES_OCC;
    }
    
    // soz
    return LORAWAN_RES_NOT_JOIN;
}

// register callback to deal with packets received on specific port (or -1 for all ports)
// data buffer given during callback will be valid only during callback (which must not block)
// Returns request id, which can be used to cancel this registration at a later point
LORAWAN_REQ_ID_t lora_api_registerRxCB(int port, LORAWAN_RX_CB_t callback, void* userctx) {	// calls the cb whenever pkt rxed (whether on classA, B, or C)
    assert(callback!=NULL);
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
static bool scheduleTxLoraWanReq(TxLoraWanReq_t* req) {
    // calculate in how long we want to schedule it (or in 100ms if 'immediately')
    uint32_t remainingdelayms = req->contract.txTimePeriodSecs!=0 ? (req->createTSMS+(req->contract.txTimePeriodSecs*1000)) - getNowMS() : 100;
    // TODO calculate required time for rx windows using the SF etc...
    if (remainingdelayms<((req->contract.doRx|req->contract.reqAck)?16000:2000)) {
        return false;       // no can do in remaining time...
    }
    os_callout_reset(&req->timer, os_time_ms_to_ticks32(remainingdelayms));
    return true;
}
// schedule a tx for time determined by <contract>
// contract is ctype (eg anytime, within next X, at absolute/relative time), and time
// data is COPIED during call, caller can reuse buffer (this is not so good for memory usage....)
// Returns id for the tx (will be used in callback) or NULL if not scheduled because tx queue is full
LORAWAN_REQ_ID_t lora_api_send(LORAWAN_SF_t sf, uint8_t port, LORAWAN_TX_CONTRACT_t* contract, 
                uint8_t* data, uint8_t sz, LORAWAN_TX_CB_t callback, void* userctx) {
    assert(callback!=NULL);
    assert(data!=NULL);
    if (sf==LORAWAN_SF_DEFAULT) {
        sf = _loraCtx.defaultSF;
    }
    if (sz>maxSz4SF(sf)) {
        sz=maxSz4SF(sf);
    }
    // Check for a free slot
    for(int i=0;i<MAX_TXCBS;i++) {
        if (_loraCtx.txLoraWANReqs[i].cbfn==NULL) {
            _loraCtx.txLoraWANReqs[i].cbfn = callback;
            _loraCtx.txLoraWANReqs[i].userctx = userctx;
            _loraCtx.txLoraWANReqs[i].port = port;
            _loraCtx.txLoraWANReqs[i].sf = sf;
            _loraCtx.txLoraWANReqs[i].power = _loraCtx.defaultLWPower;  
            // Data buffer is static max size for now - better to be mbufs or a malloc??? TODO
            memcpy(_loraCtx.txLoraWANReqs[i].data, data, sz);
            _loraCtx.txLoraWANReqs[i].sz = sz;
            _loraCtx.txLoraWANReqs[i].contract.doRx = contract->doRx;
            _loraCtx.txLoraWANReqs[i].contract.priority = contract->priority;
            _loraCtx.txLoraWANReqs[i].contract.reqAck = contract->reqAck;
            _loraCtx.txLoraWANReqs[i].contract.txTimePeriodSecs = contract->txTimePeriodSecs;
            
            // tell task to schedule tx request
            if (scheduleTxLoraWanReq(&_loraCtx.txLoraWANReqs[i])) {
                return &_loraCtx.txLoraWANReqs[i];
            } 
            // oops
            _loraCtx.txLoraWANReqs[i].cbfn = NULL;      // free slot
            return NULL;
        }
    }
    return NULL;
}

// Schedule direct radio tx access for specific time
LORAWAN_REQ_ID_t lora_api_radio_tx(uint32_t abs_time, LORAWAN_SF_t sf, uint32_t freq, int txpower, uint8_t* data, uint8_t sz, LORAWAN_TX_CB_t callback, void* userctx) {
    assert(callback!=NULL);
    assert(data!=NULL);
    // Check for a free slot
    for(int i=0;i<MAX_TXRADIOCBS;i++) {
        TxRadioReq_t* req = &_loraCtx.txRadioReqs[i];
        if (req->cbfn==NULL) {
            req->cbfn = callback;
            req->userctx = userctx;
            req->data = data;
            req->sz = sz;
            req->sf = sf;
            req->freq = freq;
            req->power = txpower;
            req->atTxMS = abs_time;

            // TODO : check time is after now, and no other radio request using it....
            os_callout_reset(&req->timer, os_time_ms_to_ticks32(abs_time - getNowMS()));

            return req;
        }
    }
    return NULL;
}
// Schedule direct radio rx access for specific time
LORAWAN_REQ_ID_t lora_api_radio_rx(uint32_t abs_time, LORAWAN_SF_t sf, uint32_t freq, uint32_t timeoutms, uint8_t* data, uint8_t sz, LORAWAN_RX_CB_t callback, void* userctx) {
    assert(callback!=NULL);
    assert(data!=NULL);
    // Check for a free slot
    for(int i=0;i<MAX_RXCBS;i++) {
        RxRadioReq_t* req = &_loraCtx.rxRadioReqs[i];
        if (req->cbfn==NULL) {
            req->cbfn = callback;
            req->userctx = userctx;
            req->data = data;
            req->sz = sz;
            req->sf = sf;
            req->freq = freq;
            req->atRxMS = abs_time;
            req->tRxMS = timeoutms;
            // TODO : check time is after now, and no other radio request using it....
            os_callout_reset(&req->timer, os_time_ms_to_ticks32(abs_time - getNowMS()));

            return req;
        }
    }
    return NULL;
}

// Cancel a pending request. True if cancelled without action, false if already in progress and cannot be cancelled
bool lora_api_cancel(LORAWAN_REQ_ID_t id) {
    // TODO - find req in one of the lists, cancel it
    return true;
}


// Internals

// get OS ms since boot
static uint32_t getNowMS() {
    return os_time_ticks_to_ms32(os_time_get());
}
// TODO use stack regional params functions to find max pkt size per SF per region
static uint8_t maxSz4SF(int sf) {
    return 52;
}

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

static int mapSF2DR(int sf) {
    switch(sf) {
        case LORAWAN_SF12: 
            return 0;
        case LORAWAN_SF11: 
            return 1;
        case LORAWAN_SF10: 
            return 2;
        case LORAWAN_SF9: 
            return 3;
        case LORAWAN_SF8: 
            return 4;
        case LORAWAN_SF7: 
            return 5;
        case LORAWAN_FSK250: 
            return 6;
        default:
            return 5;
    }
}
/*
static bool lora_check_send(uint8_t sz, LORAWAN_SF_t sf) {
    LoRaMacTxInfo_t txInfo;
    LoRaMacStatus_t txposs = LoRaMacQueryTxPossible( sz, &txInfo );
    if (txposs == LORAMAC_STATUS_OK) {
        return true;
    }
    return false;
}
*/
static bool lora_send(uint8_t* data, uint8_t sz, uint8_t port, int sf, bool confirmed) {
    McpsReq_t mcps_req;
    if (confirmed) {
        mcps_req.Type = MCPS_CONFIRMED;
        mcps_req.Req.Confirmed.fPort = port;
        mcps_req.Req.Confirmed.fBuffer = data;
        mcps_req.Req.Confirmed.fBufferSize = sz;
        mcps_req.Req.Confirmed.Datarate = mapSF2DR(sf);
        mcps_req.Req.Confirmed.NbTrials = 1;            // only 1 try even if confrmed, app level can take care of retries
    } else {
        mcps_req.Type = MCPS_UNCONFIRMED;
        mcps_req.Req.Unconfirmed.fPort = port;
        mcps_req.Req.Unconfirmed.fBuffer = data;
        mcps_req.Req.Unconfirmed.fBufferSize = sz;
        mcps_req.Req.Unconfirmed.Datarate = mapSF2DR(sf);
    }
    LoRaMacStatus_t status = LoRaMacMcpsRequest( &mcps_req );
    if(status == LORAMAC_STATUS_OK ) {
        // Send started
        return true;
    }
    log_debug("lora send fails %d", status);
    return false;
}

// Ask stack to send JOIN request
static bool lora_join(uint8_t* devEUI, uint8_t* appEUI, uint8_t* appKey, int sf) {
    MlmeReq_t mlme_req;
    mlme_req.Type = MLME_JOIN;
    mlme_req.Req.Join.DevEui = devEUI;
    mlme_req.Req.Join.AppEui = appEUI;
    mlme_req.Req.Join.AppKey = appKey;
    mlme_req.Req.Join.Datarate = mapSF2DR(sf);
    LoRaMacStatus_t status = LoRaMacMlmeRequest(&mlme_req);
    if(status == LORAMAC_STATUS_OK ) {
        // Send started
        return true;
    }
    log_debug("lora join fails %d", status);
    return false;
}
static void execStackEvent(struct os_event* e) {
    LWEVT_t* evt = (LWEVT_t*)(e->ev_arg);
    switch(evt->type) {
        case LWEVT_TYPE_MCPS_CONFIRM: {
            McpsConfirm_t* McpsConfirm = &evt->msg.mcpsc;
            // Confirmation of sending of app level message
            log_debug("MCPSconfirm: tx status %d, %d\r\n", McpsConfirm->Status,McpsConfirm->AckReceived);
            // find guy who ordered this tx and callback with result. As the stack doesn't take a context, we have to set flags (ick)
            for(int i=0;i<MAX_TXCBS;i++) {
                if (_loraCtx.txLoraWANReqs[i].txInProgress) {
                    // record bits we need before doing callback, as this allows caller to immediately re-schedule a send and use same slot
                    LORAWAN_TX_CB_t cb = _loraCtx.txLoraWANReqs[i].cbfn;
                    void* userctx = _loraCtx.txLoraWANReqs[i].userctx;
                    _loraCtx.txLoraWANReqs[i].txInProgress = false;
                    _loraCtx.txLoraWANReqs[i].cbfn = NULL;
                    // sucess?
                    if (McpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
                        (*cb)(userctx, (LORAWAN_REQ_ID_t*)&_loraCtx.txLoraWANReqs[i],
                            LORAWAN_RES_OK);
                    } else {
                        (*cb)(userctx, (LORAWAN_REQ_ID_t*)&_loraCtx.txLoraWANReqs[i],
                            LORAWAN_RES_HWERR);
                    }
                }
            }
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
                    for(int i=0;i<MAX_RXCBS;i++) {
                        if (_loraCtx.rxLoraWANReqs[i].cbfn!=NULL && 
                            (_loraCtx.rxLoraWANReqs[i].port==-1 || _loraCtx.rxLoraWANReqs[i].port==McpsIndication->Port)) {
                            (*_loraCtx.rxLoraWANReqs[i].cbfn)(_loraCtx.rxLoraWANReqs[i].userctx, (LORAWAN_REQ_ID_t)&_loraCtx.rxLoraWANReqs[i],
                                LORAWAN_RES_OK, McpsIndication->Port, McpsIndication->Rssi, McpsIndication->Snr, 
                                McpsIndication->Buffer, McpsIndication->BufferSize);
                        }
                    }
                }
                else{ 
                    /* Means that neither Fport, nor Payload, are in the frame -> ?? */
                    // ignore
                }
            }
            break;
        }
        case LWEVT_TYPE_MLME_CONFIRM: {
            MlmeConfirm_t* MlmeConfirm = &evt->msg.mlmec;
            // Could be a JOIN accept?
            switch(MlmeConfirm->MlmeRequest) {
                case MLME_JOIN: {
                    LORAWAN_RESULT_t res = LORAWAN_RES_NO_RESP;
                    if (MlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
                        log_debug("Join Accepted\r\n");
                        _loraCtx.isJoin = true;
                        res = LORAWAN_RES_JOIN_OK;
                    } else {
                        // badness
                        log_debug("JOIN failed %d",MlmeConfirm->Status);
                        _loraCtx.isJoin = false;
                    }
                    // get join cb and call it to tell app we're joined
                    if (_loraCtx.joinLoraWANReq.cbfn!=NULL) {
                        (_loraCtx.joinLoraWANReq.cbfn)(_loraCtx.joinLoraWANReq.userctx, res);
                        _loraCtx.joinLoraWANReq.cbfn=NULL;
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
            MlmeIndication_t* mlmeInd = &evt->msg.mlmei;
            log_debug("MLMEind %D\r\n", mlmeInd->MlmeIndication);
            // Don't care about mac command DLs
            break;
        }

        // Not strictly a stack upcall event..
        case LWEVT_TYPE_DOJOIN: {
            // try to send a JOIN request
            if (lora_join(_loraCtx.deveui, _loraCtx.appeui, _loraCtx.appkey, _loraCtx.defaultSF)) {
                // in progress
            } else {
                // oopsie
                // tell awaiting joiner
                if (_loraCtx.joinLoraWANReq.cbfn!=NULL) {
                    (_loraCtx.joinLoraWANReq.cbfn)(_loraCtx.joinLoraWANReq.userctx, LORAWAN_RES_DUTYCYCLE);
                    _loraCtx.joinLoraWANReq.cbfn=NULL;
                }
            }
            break;
        }
        default:{
            log_debug("unknown event:%d", evt->type);
            break;
        }
    }
    freeEvent(e);
}

static void execTxLora(struct os_event* e) {
    TxLoraWanReq_t* req = (TxLoraWanReq_t*)(e->ev_arg);
    // Are we joined?
    if (lora_api_isJoined()) {
        // do send now
        if (lora_send(req->data, req->sz, req->port, req->sf, req->contract.reqAck)) {
            // yes, started tx. Will call sender back once tx done
            req->txInProgress = true;       // this is sadly the only way to know which tx is in progress when we get the stack's callback for the result
            // free timer callout?
        } else {
            // try to reschedule
            if (!scheduleTxLoraWanReq(req)) {
                // nok, couldnt schedule (normally coz out of time)
                // tell sender we failed
                (*req->cbfn)(req->userctx, (LORAWAN_REQ_ID_t*)req, LORAWAN_RES_OCC);
                // free timer callout?
            } // else we're waiting for next try and using the timer
        }
    } else {
        // tell sender failed as no join
        (*req->cbfn)(req->userctx, (LORAWAN_REQ_ID_t*)req, LORAWAN_RES_NOT_JOIN);
    }
}
static void execTxRadio(struct os_event* e) {
    TxRadioReq_t* req = (TxRadioReq_t*)(e->ev_arg);
    // do send now
    // TODO
    // if fails tell sender the result
    (*req->cbfn)(req->userctx, (LORAWAN_REQ_ID_t*)req, LORAWAN_RES_OCC);
    // else we'll tell when radio send done
}
static void execRxRadio(struct os_event* e) {
    RxRadioReq_t* req = (RxRadioReq_t*)(e->ev_arg);
    // do rx now
    // if fails tell caller the result
    (*req->cbfn)(req->userctx, (LORAWAN_REQ_ID_t*)req, LORAWAN_RES_OCC, 0, 0, 0, NULL, 0);
    // else we'll tell when radio done (rx or timeout)   
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
// initialise lorawan stack with our config. Called by application before using stack.
void lora_api_init(uint8_t* devEUI, uint8_t* appEUI, uint8_t* appKey) {
    // Ensure all 0s, makes sure cbfns etc all as unused etc
    memset(&_loraCtx, 0, sizeof(_loraCtx));

    for(int i=0;i<8;i++) {
        _loraCtx.deveui[i] = devEUI[i];
    }
    for(int i=0;i<8;i++) {
        _loraCtx.appeui[i] = appEUI[i];
    }
    for(int i=0;i<16;i++) {
        _loraCtx.appkey[i] = appKey[i];
    }
    _loraCtx.defaultSF = LORAWAN_SF10;
    _loraCtx.defaultLWPower = 14;            // TODO - max for the region or ADRised
    // init events (mutex, q, each event in the pool)
    os_mutex_init(&_loraCtx.lwevts_mutex);
    for(int i=0;i<MAX_LWEVTS;i++) {
        _loraCtx.lwevts[i].e.ev_cb = &execStackEvent;
        _loraCtx.lwevts[i].e.ev_arg = &(_loraCtx.lwevts[i].lwevt);
        _loraCtx.lwevts[i].lwevt.type = LWEVT_TYPE_UNUSED;
    }
    for(int i=0;i<MAX_TXCBS;i++) {
        TxLoraWanReq_t* req = &_loraCtx.txLoraWANReqs[i];
        os_callout_init(&req->timer, &_loraCtx.lwevt_q, execTxLora, req);    
    }
    for(int i=0;i<MAX_TXRADIOCBS;i++) {
        TxRadioReq_t* req = &_loraCtx.txRadioReqs[i];
        os_callout_init(&req->timer, &_loraCtx.lwevt_q, execTxRadio, req);    
    }
    for(int i=0;i<MAX_RXRADIOCBS;i++) {
        RxRadioReq_t* req = &_loraCtx.rxRadioReqs[i];
        os_callout_init(&req->timer, &_loraCtx.lwevt_q, execRxRadio, req);    
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
    assert(status==LORAMAC_STATUS_OK);
}

/*

    mibReq.Type = MIB_NETWORK_JOINED;
    mibReq.Param.IsNetworkJoined = false;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = tx_pow;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_CHANNELS_NB_REP;
    mibReq.Param.ChannelNbRep = nb_rep;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

#if MYNEWT_VAL(SX1272) || MYNEWT_VAL(SX1276)
    mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
    mibReq.Param.SystemMaxRxError = (MYNEWT_VAL(SX127X_RADIO_MIN_RX_DURATION))/2;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );
#endif
 */
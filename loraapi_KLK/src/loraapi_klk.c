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

/* loraapi implmementation using the kerlink wrapper api round the stackforce implementation. */
/* Current functionalities TBI:
 - KLK api does not allow JOIN without UL -> JOIN cost always has a UL as well
 - KLK api does not allow UL without RX -> request UL with doRx=false is not honoured
 - KLK api does not allow DL RX on wildcard port -> cannot register RX with wildcard port simply
 - KLK api does not allow handling of txpower/SF per-tx -> must reconfig socket every time?
 - radio access apis not yet implemented
 */
#include <string.h>

#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_spi.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/configmgr.h"
#include "wyres-generic/timemgr.h"
#include "wyres-generic/lowpowermgr.h"
#include "loraapi/loraapi.h"
// Kerlink lorawan api
#include "lorawan_api/lorawan_api.h"

#define LORAAPI_TASK_PRIO       MYNEWT_VAL(LORAAPI_TASK_PRIO)
#define LORAAPI_TASK_STACK_SZ   OS_STACK_ALIGN(256)
#define MAX_RXCBS   (2)         // can register 2 rx callbacks on different ports if required
#define MAX_TXRADIOCBS   (1)    // as not yet implemented
#define MAX_RXRADIOCBS   (1)    // or maybe coz don't need this fn

#define MAX_LWEVTS  (2)     // number of outstanding API->task events at any time

// Events sent on a task q to be executed asynchronously
typedef enum { LWEVT_TYPE_UNUSED, 
    LWEVT_TYPE_DOJOIN, LWEVT_TYPE_DOTXLW, LWEVT_TYPE_DOTXRADIO, LWEVT_TYPE_DORXRADIO
 } LWEVT_TYPE_t;
typedef struct {
    LWEVT_TYPE_t type;
    void* req;          // points to request data, depends on event type
} LWEVT_t;

typedef struct {
        LORAWAN_JOIN_CB_t cbfn;
        void* userctx;
    } JoinReq_t;

typedef struct  {
        LORAWAN_SF_t sf;
        int8_t power;
        uint8_t port;
        bool reqAck;
        bool doRx;
        uint8_t* data;         
        uint8_t sz;
        LORAWAN_TX_CB_t cbfn;
        void* userctx;
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
    LP_ID_t lpUserId;
    LORAWAN_SF_t defaultSF;
    int defaultLWPower;
    bool useADR;
    uint32_t txTimeoutMS;
    JoinReq_t joinLoraWANReq;
    TxLoraWanReq_t txLoraWANReq;
    RxLoraWanReq_t rxLoraWANReqs[MAX_RXCBS];        // can have multiple rx listeners
    TxRadioReq_t txRadioReqs[MAX_TXRADIOCBS];
    RxRadioReq_t rxRadioReqs[MAX_RXRADIOCBS];
    uint8_t deveui[8];
    uint8_t appeui[8];
    uint8_t appkey[16];
    os_stack_t loraapi_task_stack[LORAAPI_TASK_STACK_SZ];
    struct os_task loraapi_task_str;
    lorawan_sock_t sock_tx;
    lorawan_sock_t sock_rx;
    struct os_eventq* lwevt_q;
    struct os_sem lora_tx_sem;
    // lwevt pool
    struct {
        struct os_event e;
        LWEVT_t lwevt;
    } lwevts[MAX_LWEVTS];
    struct os_mutex lwevts_mutex;
    uint32_t noEventCnt;        // could each time we have event pool starvation
} _loraCtx;     // Note : initialised to all 0 in init method and then to specific defaults

static void loraapi_task(void* data);
static struct os_event* allocEvent();
static void freeEvent(struct os_event* e);
static uint8_t maxSz4SF(int sf);

/** API implementation */

bool lora_api_isJoined() {
    // TODO - do mib request to see if join or not
    if (_loraCtx.sock_tx>0) {
        lorawan_attribute_t mib;
        mib.Type = LORAWAN_ATTR_NETWORK_JOINED;
        if (lorawan_getsockopt(_loraCtx.sock_tx, &mib)== LORAWAN_STATUS_OK) {
            return mib.Param.IsNetworkJoined;
        }
        log_warn("LW:NOK JOIN chk: get fails");
        return false;
    } else {
        log_warn("LW:NOK JOIN chk: no socket");
        return false;
    }
}

 // Do the join (if already joined, returns this status to the callback)
LORAWAN_RESULT_t lora_api_join(LORAWAN_JOIN_CB_t callback, LORAWAN_SF_t sf, void* userctx) {
    assert(callback!=NULL);
    if (_loraCtx.sock_tx<=0) {
        return LORAWAN_RES_FWERR;       // can't join before init
    }

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
            evt->req = NULL;        // not required
            log_info("LW: JOINing as [%02x%02x%02x%02x%02x%02x%02x%02x]",
                _loraCtx.deveui[0],_loraCtx.deveui[1],_loraCtx.deveui[2],_loraCtx.deveui[3],_loraCtx.deveui[4],_loraCtx.deveui[5],_loraCtx.deveui[6],_loraCtx.deveui[7]);

            os_eventq_put(_loraCtx.lwevt_q, e);
            return LORAWAN_RES_OK;
        } else {
            // oopsie
            log_warn("LW:J noevts");
            _loraCtx.joinLoraWANReq.cbfn = NULL;
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
LORAWAN_RESULT_t lora_api_registerRxCB(int port, LORAWAN_RX_CB_t callback, void* userctx) {	// calls the cb whenever pkt rxed (whether on classA, B, or C)
    assert(callback!=NULL);
    assert(port!=0);        // not allowed to register for mac comands
    if (_loraCtx.sock_rx<=0) {
        return LORAWAN_RES_FWERR;       // can't bind before init
    }
    // Check for a free slot
    for(int i=0;i<MAX_RXCBS;i++) {
        if (_loraCtx.rxLoraWANReqs[i].cbfn==NULL) {
            _loraCtx.rxLoraWANReqs[i].cbfn = callback;
            _loraCtx.rxLoraWANReqs[i].userctx = userctx;
            _loraCtx.rxLoraWANReqs[i].port = port;
            // KLK api requires bind to a specific port, no wildcards so must loop and do on all possible ones
            if (port>0) {
                assert(lorawan_bind(_loraCtx.sock_rx, 0, port)==0);
                log_debug("LW: registered callback RX on port=%d", port);
            } else {
                for(int p=1; p<224; p++) {
                    assert(lorawan_bind(_loraCtx.sock_rx, 0, p)==0);
                }
                log_debug("LW: registered callback RX on port=-1");
            }
            return LORAWAN_RES_OK;
        }
    }
    return LORAWAN_RES_OCC;
}
// Cancel an rx cb
void lora_api_cancelRxCB(int port, LORAWAN_RX_CB_t callback) {
    // Check for a matching slot
    for(int i=0;i<MAX_RXCBS;i++) {
        if (_loraCtx.rxLoraWANReqs[i].cbfn==callback && 
            _loraCtx.rxLoraWANReqs[i].port==port) {
            _loraCtx.rxLoraWANReqs[i].cbfn=NULL;
            _loraCtx.rxLoraWANReqs[i].port=0;
            return;
        }
    }
}

// Request an LW UL TX. Data buffer must be kept idem until callback is called
LORAWAN_RESULT_t lora_api_send(LORAWAN_SF_t sf, uint8_t port, bool reqAck, bool doRx, 
                uint8_t* data, uint8_t sz, LORAWAN_TX_CB_t callback, void* userctx) {
    assert(callback!=NULL);
    assert(data!=NULL);
    if (_loraCtx.sock_tx<=0) {
        return LORAWAN_RES_FWERR;       // can't tx before init
    }
    if (sf==LORAWAN_SF_DEFAULT) {
        sf = _loraCtx.defaultSF;
    }
    if (sz>maxSz4SF(sf)) {
        sz=maxSz4SF(sf);
    }
    // Check not already ongoing
    if (_loraCtx.txLoraWANReq.cbfn==NULL) {
        _loraCtx.txLoraWANReq.cbfn = callback;
        _loraCtx.txLoraWANReq.userctx = userctx;
        _loraCtx.txLoraWANReq.port = port;
        _loraCtx.txLoraWANReq.sf = sf;
        _loraCtx.txLoraWANReq.power = _loraCtx.defaultLWPower;  
        _loraCtx.txLoraWANReq.data=data;
        _loraCtx.txLoraWANReq.sz = sz;
        _loraCtx.txLoraWANReq.doRx = doRx;
        _loraCtx.txLoraWANReq.reqAck = reqAck;
        // kick off request
        struct os_event* e = allocEvent();
        if (e!=NULL) {
            LWEVT_t* evt = (LWEVT_t*)e->ev_arg;
            evt->type = LWEVT_TYPE_DOTXLW;
            evt->req = &_loraCtx.txLoraWANReq;
            os_eventq_put(_loraCtx.lwevt_q, e);
            return LORAWAN_RES_OK;
        } else {
            // oopsie
            log_warn("LW:TX noevts");
            _loraCtx.txLoraWANReq.cbfn = NULL;
            _loraCtx.noEventCnt++;
            return LORAWAN_RES_DUTYCYCLE;
        }
    }
    return LORAWAN_RES_OCC;
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
            os_callout_reset(&req->timer, os_time_ms_to_ticks32(abs_time - TMMgr_getRelTimeMS()));

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
            os_callout_reset(&req->timer, os_time_ms_to_ticks32(abs_time - TMMgr_getRelTimeMS()));

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

// Get current lora region
int lora_api_getCurrentRegion() {
    return lorawan_get_current_region();
}

// Set a new region (before JOIN). If the region has not been compiled into this firmware, an error is returned.
LORAWAN_RESULT_t lora_api_setCurrentRegion(int r) {
    if (r==lora_api_getCurrentRegion()) {
        return LORAWAN_RES_OK;
    }
    // Can't change region just now....
    return LORAWAN_RES_BADPARAM;
}


// Internals

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


/*
static bool spi0_read_buffer(uint8_t addr, uint8_t* buffer, uint8_t sz) {
    uint8_t spiId = 0;
    uint8_t spiNss = SPI_0_MASTER_PIN_NSS;
    int ret=0;
    // Config SPI
    struct hal_spi_settings spi0_setting = {
        .data_order = HAL_SPI_MSB_FIRST,
        .data_mode = HAL_SPI_MODE0,
        .baudrate = 2000,
        .word_size = HAL_SPI_WORD_SIZE_8BIT,
    };

    if ((ret = hal_spi_disable(spiId))!=0) {
        log_debug("spi disable fails %d",ret);
    }
    if ((ret=hal_spi_config(spiId, &spi0_setting))!=0) {
        log_debug("spi config fails %d",ret);
    }
    if ((ret = hal_spi_enable(spiId))!=0) {
        log_debug("spi enable fails %d",ret);
    }
    // Read chip id from reg 0x42
    //NSS = 0;
    hal_gpio_init_out(spiNss, 0 );
    hal_gpio_write(spiNss, 0);

    // Write the reg address
    hal_spi_tx_val(spiId, addr & 0x7F);

    // read the data
    for(int i = 0; i < sz; i++ ) {
        buffer[i] = hal_spi_tx_val(spiId, 0);
    }

    //NSS = 1;
    hal_gpio_write(spiNss, 1);
    return true;
}
static bool checkRadio() {
    uint8_t id=0;
    if (!spi0_read_buffer(0x42, &id, 1)) {
        return false;
    }
    if (id == 0x22) {
        return true;
    }
    log_debug("radio id read returns unexpected id %d", id);
    return false;
}
*/

static int8_t mapPowerDb2PowerLevel(int8_t db) {
    // 868MHz : 0=highest, 1=14, 2=11, 3=8, 4=5, 5=2...
    // MApping from dbM to power level is regional dependant...
    int8_t txPowerLevel = ((14-db)/2);
    // Ensure not negative (eg for >14dm permitted regions)
    if (txPowerLevel<0) {
        txPowerLevel = 0;
    }
    return txPowerLevel;
}
// TODO use stack regional params functions to find max pkt size per SF per region
static uint8_t maxSz4SF(int sf) {
    switch(sf) {
        case LORAWAN_SF12: 
            return 52;
        case LORAWAN_SF11: 
            return 52;
        case LORAWAN_SF10: 
            return 52;
        case LORAWAN_SF9: 
            return 98;
        case LORAWAN_SF8: 
            return 98;
        case LORAWAN_SF7: 
            return 250;
        case LORAWAN_FSK250: 
            return 250;
        default:
            return 52;
    }
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
            return 2;       // If your SF is weird, you get SF10
    }
}
static bool configTxSocket(lorawan_sock_t skt, bool useAck, LORAWAN_SF_t sf, int txPower) {
    bool ret = true;
    lorawan_status_t status;
    lorawan_attribute_t mib;
    mib.Type = LORAWAN_ATTR_MCPS_TYPE;
    if (useAck) {
        mib.Param.McpsType = LORAWAN_MCPS_CONFIRMED;
    } else {
        mib.Param.McpsType = LORAWAN_MCPS_UNCONFIRMED;
    }
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("LW:NOK set ACK");
        ret = false;
    }
    int8_t loraDR = mapSF2DR(sf);
    mib.Type = LORAWAN_ATTR_CHANNELS_DATARATE;
    mib.Param.ChannelsDefaultDatarate = loraDR;
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("LW:NOK set data rate");
        ret = false;
    }
    mib.Type = LORAWAN_ATTR_CHANNELS_TX_POWER;
    mib.Param.ChannelsDefaultTxPower = mapPowerDb2PowerLevel(txPower);        // lorawan stack power does 0-7, coz its in the original spec
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("LW:NOK set tx power");
        ret = false;
    }
    return ret;
}

// Do lower layer UL. Caller should check if JOINed or not... and if req is free (ie no other tx in progress)
static bool do_lora_send(TxLoraWanReq_t* req) {
    /* put the data into the queue, the message will be sent
        * when the LoRaWAN stack is ready. 
        **/
    // Set DR, txpower, ack, rx window for this tx
    if (configTxSocket(_loraCtx.sock_tx, req->reqAck, req->sf, req->power)) {
        log_debug("LW:tx cfg ok");
    }
    // and schedule the sending
    int ret = lorawan_send(_loraCtx.sock_tx, req->port, req->data, req->sz);
    switch(ret) {
        case LORAWAN_STATUS_OK: {
            log_debug("LW:tx %d bytes ok on port:%d sf:%d txpower:%d ackReq:%d doRx %d]",
                req->sz, req->port, 
                req->sf, req->power, req->reqAck,
                req->doRx);
            // release the task to wait for result 
            os_sem_release(&_loraCtx.lora_tx_sem);
            // yes, started tx. Will call sender back once tx done
            return true;
        }
        case LORAWAN_STATUS_PORT_BUSY: {
            log_debug("LW:tx  busy");
            // tell sender we failed
            (*req->cbfn)(req->userctx, LORAWAN_RES_OCC);
            // And free his slot
            req->cbfn = NULL;
            return false;
        }
        default: {
            log_warn("LW:tx fatal error (%d).",
                ret);
            // tell sender failed 
            (*req->cbfn)(req->userctx, LORAWAN_RES_HWERR);
            // And free his slot
            req->cbfn = NULL;
            return false;       // best you reset mate
        }
    }
}

static void stack_joinTXUL_cb(void* uctx, LORAWAN_RESULT_t res) {
    LORAWAN_RESULT_t jres = LORAWAN_RES_NOT_JOIN;
    log_debug("LW: JCB %d", res);
    if (res==LORAWAN_RES_OK) {
        // mens join must have worked
        jres = LORAWAN_RES_JOIN_OK;
        _loraCtx.isJoin = true;
    } else {
        // join probably failed... replicate result upwards
        jres = res;
    }
    if (_loraCtx.joinLoraWANReq.cbfn!=NULL) {
        (_loraCtx.joinLoraWANReq.cbfn)(_loraCtx.joinLoraWANReq.userctx, jres);
        // clear cb so app can try again. Because we don't want them to retry immediately, we don't let them retry in the callback!
        _loraCtx.joinLoraWANReq.cbfn=NULL;
    }

}

// KLK api means must do UL to force JOIN. Kind of annoying but might as well make it useful
// This cannot include directly the appcore message format in app_msg.h to avoid dependancies, but we'll make something similair
//	0 : b0-3: ULrespid, b4-5: protocol version, b6: 1=listening for DL, 0=not listening, b7: force even parity for this byte
//	1 : length of following TLV block = 6
// 2/3 : T=0, L=4
// 4-7 : uptime in ms
static struct {
    uint8_t h1;
    uint8_t h2;
    uint8_t t;
    uint8_t l;
    uint32_t uptime;
} _joinUL = {
    .h1=0x50,       // respId=0, version=1, not listening for DL, even parity
    .h2=6,          // 6 bytes to follow
    .t=1,           // tag 1 = uptime (in appcore.h)
    .l=4,           // length of value is 4 bytes
};

// Try to JOIN using the config already set in init(). Caller should check beforehand if we are already joined....
static bool do_lora_join() {
    // no explicit join phase currently with KLK wrapper. 
    // send a simple UL with our own CB to force it to do so
    TxLoraWanReq_t* req = &_loraCtx.txLoraWANReq;
    req->cbfn = stack_joinTXUL_cb;
    // put current time in UL message?
    _joinUL.uptime = TMMgr_getRelTimeSecs();
    req->data = (uint8_t*)(&_joinUL);
    req->sz = sizeof(_joinUL);        // we send just our uptime
    req->port = 1;
    req->sf = _loraCtx.defaultSF;
    req->power = _loraCtx.defaultLWPower;  
    req->doRx = true;
    req->reqAck = false;
    if (do_lora_send(req)) {
        log_debug("LW:JTX OK");
        return true;
    } else {
        log_debug("LW:JTX NOK");
        return false;
    }
}

// Interpret event requests
static void execReqEvent(struct os_event* e) {
    LWEVT_t* evt = (LWEVT_t*)(e->ev_arg);
    switch(evt->type) {
        case LWEVT_TYPE_DOJOIN: {
            // Are we joined?
            if (lora_api_isJoined()) {
                log_debug("LW:J req but already joined");
                if (_loraCtx.joinLoraWANReq.cbfn!=NULL) {
                    (_loraCtx.joinLoraWANReq.cbfn)(_loraCtx.joinLoraWANReq.userctx, LORAWAN_RES_JOIN_OK);
                    _loraCtx.joinLoraWANReq.cbfn=NULL;
                }
            } else {
                // try to send a JOIN request (devEUI, appKey, use ADR etc already setup in init)
                if (do_lora_join()) {
                    // in progress
                } else {
                    // oopsie
                    // tell awaiting joiner
                    if (_loraCtx.joinLoraWANReq.cbfn!=NULL) {
                        (_loraCtx.joinLoraWANReq.cbfn)(_loraCtx.joinLoraWANReq.userctx, LORAWAN_RES_DUTYCYCLE);
                        _loraCtx.joinLoraWANReq.cbfn=NULL;
                    }
                }
            }
            break;
        }
        case LWEVT_TYPE_DOTXLW: {
            TxLoraWanReq_t* req = &_loraCtx.txLoraWANReq;
            // Are we joined?
            if (lora_api_isJoined()) {
                do_lora_send(req);
            } else {
                log_debug("LW:TXUL not joined");
                // tell sender failed as not join
                (*req->cbfn)(req->userctx, LORAWAN_RES_NOT_JOIN);
                // And free his slot
                req->cbfn = NULL;
            }
            break;            
        }
        case LWEVT_TYPE_DORXRADIO: {
            break;            
        }
        case LWEVT_TYPE_DOTXRADIO: {
            break;            
        }
        default:{
            log_debug("LW:? event:%d", evt->type);
            break;
        }
    }
    freeEvent(e);
}

static void execTxRadio(struct os_event* e) {
    // TODO access to stack not yet possible
    log_warn("LW:DRTX TBI");
}
static void execRxRadio(struct os_event* e) {
    // TODO access to stack not yet possible
    log_warn("LW:DRRX TBI");
}

extern void lorawan_init(void);


#if MYNEWT_VAL(SX1272)
//TODO : move this in board_utils.c
extern void SX1272IoDeInit( void );
extern void SX1272AntSwDeInit( void );

void lorawan_deinit (void)
{
    SX1272IoDeInit();
    SX1272AntSwDeInit();
}
#elif MYNEWT_VAL(SX1262)
extern void SX126xIoDeInit( void );
//extern void SX1272AntSwDeInit( void ); No AntSwitch deinit for sx126x to do it as the same way than for 1272 ?

void lorawan_deinit (void)
{
    SX126xIoDeInit();
    //SX1272AntSwDeInit();
}
#else
//TODO Not yet implemented
void lorawan_deinit (void)
{
    //Nothing to do
}
#endif

// Callback from low power manager about change of mode - NO LOGS
static void lp_change(LP_MODE_t prevmode, LP_MODE_t newmode) {
    // Radio is ON in all modes except DEEPSLEEP
    if (prevmode>=LP_DEEPSLEEP && newmode <LP_DEEPSLEEP) {
        // wake up radio - init its periphs
        // TODO find KLK api to do this?
        lorawan_init();         // in board_utils.c
                
    } else if (prevmode<LP_DEEPSLEEP && newmode >= LP_DEEPSLEEP) {
        // shutdown radio - reset its periphs, deinit spi etc
        // TODO
        lorawan_deinit();         // not yet in board_utils.c
    }
}

void lora_api_deinit(void) {

    // TODO : be sure whole context is deinit 
    // and semaphore or mutexes have been released

    // deinit lower layer
    lorawan_deinit();
}



// initialise lorawan stack with our config. Called by application before using stack.
void lora_api_init(uint8_t* devEUI, uint8_t* appEUI, uint8_t* appKey, bool enableADR, LORAWAN_SF_t defaultSF, int8_t defaultTxPower) {
    // Ensure all 0s, makes sure cbfns etc all as unused etc
    memset(&_loraCtx, 0, sizeof(_loraCtx));

    _loraCtx.isJoin = false;
    _loraCtx.txTimeoutMS = 16000;       // longest time it should take in SF12 to send max packet, and wait for max DL on RX2...
    // execute our api requests on the default event q
    _loraCtx.lwevt_q = os_eventq_dflt_get();
    // need a semaphore...
    os_sem_init(&_loraCtx.lora_tx_sem,0);

    for(int i=0;i<8;i++) {
        _loraCtx.deveui[i] = devEUI[i];
    }
    for(int i=0;i<8;i++) {
        _loraCtx.appeui[i] = appEUI[i];
    }
    for(int i=0;i<16;i++) {
        _loraCtx.appkey[i] = appKey[i];
    }
    _loraCtx.useADR = enableADR;
    _loraCtx.defaultSF = defaultSF;
    _loraCtx.defaultLWPower = defaultTxPower;
    // init events (mutex, q, each event in the pool)
    os_mutex_init(&_loraCtx.lwevts_mutex);
    for(int i=0;i<MAX_LWEVTS;i++) {
        _loraCtx.lwevts[i].e.ev_cb = &execReqEvent;
        _loraCtx.lwevts[i].e.ev_arg = &(_loraCtx.lwevts[i].lwevt);
        _loraCtx.lwevts[i].lwevt.type = LWEVT_TYPE_UNUSED;
    }
    for(int i=0;i<MAX_TXRADIOCBS;i++) {
        TxRadioReq_t* req = &_loraCtx.txRadioReqs[i];
        os_callout_init(&req->timer, _loraCtx.lwevt_q, execTxRadio, req);    
    }
    for(int i=0;i<MAX_RXRADIOCBS;i++) {
        RxRadioReq_t* req = &_loraCtx.rxRadioReqs[i];
        os_callout_init(&req->timer, _loraCtx.lwevt_q, execRxRadio, req);    
    }

/*    // Check SX1272 is alive
    if (checkRadio()) {
        log_debug("radio check says its good");
    } else {
        log_warn("radio check fails for spi0...");
        wassert_hw_fault();
    }
    */

    // Ok, ready to setup KLK Lorawan wrapper. In theory could do this in lorawan_join()? but then gotta deal with re-joins etc...
    uint8_t nb_rep = 1;
    // Note that the txPower is set for each individual tx. 
    // Note also that the KLK code expects it as a 'power level' which is not directly the XdBm everyone uses... (see table 7.1.3)
    int status = lorawan_configure_OTAA(_loraCtx.deveui, _loraCtx.appeui, _loraCtx.appkey, nb_rep, 
            mapPowerDb2PowerLevel(_loraCtx.defaultLWPower), lora_api_getCurrentRegion());
    assert(status == LORAWAN_STATUS_OK);
//    } else {
//        int status = lorawan_configure_ABP(_loraCfg.devAddr, _loraCfg.nwkSkey, _loraCfg.appSkey, nb_rep, ((14-_loraCfg.defaultLWPower)/2), lora_api_getCurrentRegion());
//        assert(status == LORAWAN_STATUS_OK);
//        log_debug("Started LoRaWAN OK in ABP mode [with devAddr:%08lx]", _loraCfg.devAddr);
//    }

    // must be reglo
    lorawan_set_dutycycle(false);

    /* 1st action: obtain a socket from the LoRaWAN API */
    _loraCtx.sock_tx = lorawan_socket(SOCKET_TYPE_TX);
    assert(_loraCtx.sock_tx > 0);
    // Set adr option
    lorawan_attribute_t mib;
    mib.Type = LORAWAN_ATTR_ADR;
    mib.Param.AdrEnable = _loraCtx.useADR;
    if (lorawan_setsockopt(_loraCtx.sock_tx, &mib)!= LORAWAN_STATUS_OK) {
        log_warn("LW:failed to set ADR");
    }

    /* 1st action: obtain a socket from the LoRaWAN API */
    _loraCtx.sock_rx = lorawan_socket(SOCKET_TYPE_RX);
    assert(_loraCtx.sock_rx > 0);

    // Create task to run TX/RX as KLK wrapper uses blocking calls... 
    os_task_init(&_loraCtx.loraapi_task_str, "lw_eventq",
                 loraapi_task, NULL,
                 LORAAPI_TASK_PRIO, OS_WAIT_FOREVER,
                 _loraCtx.loraapi_task_stack,
                 LORAAPI_TASK_STACK_SZ);
    // register with lowpowermgr to know when to deinit/init the radio
    _loraCtx.lpUserId = LPMgr_register(lp_change);

    // ok lorawan api all init ok
    log_info("LW: cfgd [%02x%02x%02x%02x%02x%02x%02x%02x] adr:%d, sf:%d, txpower:%d",
            _loraCtx.deveui[0],_loraCtx.deveui[1],_loraCtx.deveui[2],_loraCtx.deveui[3],_loraCtx.deveui[4],_loraCtx.deveui[5],_loraCtx.deveui[6],_loraCtx.deveui[7],
            _loraCtx.useADR, _loraCtx.defaultSF, _loraCtx.defaultLWPower);

}

static void callTxCB(lorawan_event_t txev) {
    if (_loraCtx.txLoraWANReq.cbfn!=NULL) {
        LORAWAN_TX_CB_t txcbfn = _loraCtx.txLoraWANReq.cbfn;
        void* userctx = _loraCtx.txLoraWANReq.userctx;
        _loraCtx.txLoraWANReq.cbfn = NULL;      // so app can do a tx in the cb
        if (txev & LORAWAN_EVENT_ACK) {
            log_debug("LW:tx ev OK ACK");
            (*txcbfn)(userctx, LORAWAN_RES_OK);
        } else if (txev & LORAWAN_EVENT_SENT) {
            log_debug("LW:tx ev OK NOACK");
            (*txcbfn)(userctx, LORAWAN_RES_OK);
        }
        if (txev & LORAWAN_EVENT_ERROR) {
            // Check if we are joined
            if (lora_api_isJoined()) {
                log_debug("LW:tx ev ERR %d (BUT joined)", txev);
                // Duty cycle or the like, try again later please
                (*txcbfn)(userctx, LORAWAN_RES_OCC);
            } else {
                // Not joined this is why an error
                log_debug("LW:tx ev ERR %d (NOT joined)", txev);
                (*txcbfn)(userctx, LORAWAN_RES_NOT_JOIN);
            }
        }
        if (txev==0) {
            // timeout - not sure how to interpret this - tx ok but no rx?
            log_debug("LW: tx ev TIMEOUT");
            (*txcbfn)(userctx, LORAWAN_RES_TIMEOUT);
        }
    } else {
        // shouldn't happen? Assrt?
        log_warn("LW: tx ev no cbfn");
    }
}
static void callRxCB(uint8_t port, uint8_t* data, uint8_t sz, int rssi, int snr) {
    // Find all registered users for this port and call them
    for(int i=0;i<MAX_RXCBS;i++) {
        if (_loraCtx.rxLoraWANReqs[i].cbfn!=NULL &&
            (_loraCtx.rxLoraWANReqs[i].port==port || _loraCtx.rxLoraWANReqs[i].port==-1)) {
            (*_loraCtx.rxLoraWANReqs[i].cbfn)(_loraCtx.rxLoraWANReqs[i].userctx, LORAWAN_RES_OK, port, rssi, snr, data, sz);
        }
    }
}

static void loraapi_task(void* data) {
    while(1) {
        if (_loraCtx.sock_tx>0) {
            // Wait till we have tx'd something
            os_sem_pend(&_loraCtx.lora_tx_sem, OS_TIMEOUT_NEVER);
            lorawan_event_t txev = lorawan_wait_ev(_loraCtx.sock_tx, (LORAWAN_EVENT_ACK|LORAWAN_EVENT_ERROR|LORAWAN_EVENT_SENT), 
                                _loraCtx.txTimeoutMS);
            // process event
            callTxCB(txev);
        }
        // TX done. try for an RX while we're here... 
        if (_loraCtx.sock_rx>0) {
            uint32_t devAddr;
            uint8_t port;
            uint8_t payload[256];
            // Wait in theory for 18s as gotta wait for RX2 delay (up to 16s) + SF12 (2s)
            uint8_t rxsz = lorawan_recv(_loraCtx.sock_rx, &devAddr, &port, payload, 255, 18000);
            log_debug("LW: rx [%d] bytes", rxsz);
            if (rxsz>0) {
                // TODO need to get rssi, snr please!
                callRxCB(port, payload, rxsz, 0, 0);
            }
        }
        // If no socket open, then just wait (else we wait either for a tx or for an rx)
        // this should never be the case as this layer always opens both tx and rx sockets
        if (!(_loraCtx.sock_tx>0 || _loraCtx.sock_rx>0)) {
            os_time_delay(OS_TICKS_PER_SEC*60);
        }
    }
}

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


#include "os/os.h"
#include "wyres-generic/wutils.h"
#include "wyres-generic/wskt_user.h"
#include "wyres-generic/wblemgr.h"
#include "wyres-generic/lowpowermgr.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/uartselector.h"
#include "wyres-generic/sm_exec.h"

#define WBLE_UART    MYNEWT_VAL(WBLE_UART)
//#define WBLE_TASK_PRIO       MYNEWT_VAL(WBLE_TASK_PRIO)
//#define WBLE_TASK_STACK_SZ   OS_STACK_ALIGN(256)
#define MAX_IBEACONS    MYNEWT_VAL(WBLE_MAXIBS)

static char* BLE_CHECK="AT+WHO\r\n";
//static char* BLE_CONFIG="AT+CONFIG";
static char* BLE_POLL="AT+POLL\r\n";
//static char* BLE_PUSH="AT+PUSH";
static char* BLE_SCAN_START="AT+START\r\n";
static char* BLE_SCAN_STOP="AT+STOP\r\n";

static char* BLE_TYPE_SCANNER="AT+TYPE=0\r\n";
static char* BLE_TYPE_IBEACON="AT+TYPE=2\r\n";
#define TYPE_SCANNER    (1)
#define TYPE_IBEACON    (3)

//static os_stack_t _wble_task_stack[WBLE_TASK_STACK_SZ];
//static struct os_task wble_mgr_task_str;

static struct blectx {
    struct os_event myUARTEvent;
//    struct os_eventq myEQ;
    struct os_mutex dataMutex;
    SM_ID_t mySMId;
    const char* uartDevice;
    uint32_t baudrate;
    int8_t pwrPin;
    int8_t uartSelect;
    wskt_t* cnx;
    uint8_t rxbuf[WSKT_BUF_SZ+1];
    uint32_t lastDataTime;
    WBLE_CB_FN_t cbfn;
    uint8_t uuid[16];
    uint32_t cardType;
    uint16_t majorStart;
    uint16_t majorEnd;
    uint8_t ibListSz;
    ibeacon_data_t ibList[MAX_IBEACONS];
    uint8_t nbRxNew;
    uint8_t nbRxNewR;
    uint8_t nbRxUpdate;
    uint8_t nbRxNoSpace;
    uint8_t nbRxBadMajor;
} _ctx = {
    .lastDataTime = 0,
    .cbfn=NULL,
};
// State machine for BLE control
enum BLEStates { MS_BLE_OFF, MS_BLE_WAITPOWERON, MS_BLE_WAIT_TYPE_SCAN, MS_BLE_WAIT_TYPE_IB, MS_BLE_STARTING, MS_BLE_ON, MS_BLE_SCANNING, 
    MS_BLE_IBEACON, MS_BLE_STOPPINGCOMM, MS_BLE_LAST };
enum BLEEvents { ME_BLE_ON, ME_BLE_OFF, ME_BLE_SCAN, ME_BLE_IBEACON, ME_BLE_STOP, ME_BLE_RET_OK, ME_BLE_RET_ERR, ME_BLE_RET_INT,
     ME_BLE_UPDATE, ME_BLE_UART_OK, ME_BLE_UART_NOK };

// predeclare privates
//static void wble_mgr_task(void* arg);
static void wble_mgr_rxcb(struct os_event* ev);
// manage IB list
static void resetIBList();
static ibeacon_data_t*  getIB(int idx);
// Add scanned IB to list if not already present else update it
static int addIB(ibeacon_data_t* ibp);

static SM_STATE_ID_t State_Off(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Ensure no open cnx and tuern off the power
            if (ctx->cnx!=NULL) {
                wskt_close(&ctx->cnx);      // nulls our cnx id also
            }
    
            if (ctx->pwrPin>=0) {
                log_debug("BLE: OFF pin %d", ctx->pwrPin);
                // TODO add a battery check before and after power on as this could be nice to detect battery end of life
                GPIO_write(ctx->pwrPin, 1);     // yup pull UP for OFF
            } else {
                log_debug("BLE: always on?");
            }
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            return MS_BLE_OFF;
        }
        case ME_BLE_ON: {

            return MS_BLE_WAITPOWERON;
        }

        default: {
            log_debug("BLE:? %d in Off", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// Wait 500ms or for a "READY" for module to get its act together
static SM_STATE_ID_t State_WaitPoweron(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {            
            // initialise comms to the ble via the uart like comms device defined in syscfg
            // This is async as we ask for exclusive access... we set a timeout of 100ms for other users data to be flushed
            ctx->cnx = wskt_open(ctx->uartDevice, &ctx->myUARTEvent, os_eventq_dflt_get()); // &ctx->myEQ);
//            assert(ctx->cnx!=NULL);
            if (ctx->cnx==NULL) {
                log_debug("BLE: Failed open uart!");
                sm_sendEvent(ctx->mySMId, ME_BLE_UART_NOK, NULL);
                return SM_STATE_CURRENT;
            }
            // Power up using power pin if required
            if (ctx->pwrPin<0) {
                log_debug("BLE: always on?");
            } else {
                GPIO_write(ctx->pwrPin, 0);     // yup pull down for ON
                log_debug("BLE: ON pin %d", ctx->pwrPin);
            }
            // And set the timer for the powerup time to finish (also serves as timeout for the flush wait)
            sm_timer_start(ctx->mySMId, 500);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            // check if flush of previous users data done yet?
            wskt_ioctl_t cmd = {
                .cmd = IOCTL_CHECKTX,
                .param = 0,
            };
            // just check if anybody's data waiting
            if (wskt_ioctl(ctx->cnx, &cmd)!=0) {
                log_debug("BLE: flushing old tx");
                cmd.cmd = IOCTL_FLUSHTXRX;
                cmd.param = 0;
                wskt_ioctl(ctx->cnx, &cmd);
            }
            // Set baud rate
            cmd.cmd = IOCTL_SET_BAUD;
            cmd.param = ctx->baudrate;
            wskt_ioctl(ctx->cnx, &cmd);
            // Set eol to be LF
            cmd.cmd = IOCTL_SETEOL;
            cmd.param = 0x0A;
            wskt_ioctl(ctx->cnx, &cmd);
            // only want ascii please
            cmd.cmd = IOCTL_FILTERASCII;
            cmd.param = 1;
            wskt_ioctl(ctx->cnx, &cmd);
            cmd.cmd = IOCTL_SELECTUART;
            cmd.param = ctx->uartSelect;
            wskt_ioctl(ctx->cnx, &cmd);
            // not going to set a type, assume its flashed as configurable scanner, go straight to starting check
            return MS_BLE_STARTING;
        }
        case ME_BLE_UART_NOK: {
            // ooops, we didnt get our exclusive access...
            log_debug("BLE: Failed uart!");
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLE_COMM_FAIL, NULL);
            }

            return MS_BLE_OFF;
        }
        case ME_BLE_UART_OK: {
            // init of uart cnx will be done once powerup init timeout 
            log_debug("BLE: uart ready");
            return SM_STATE_CURRENT;
        }
        case ME_BLE_RET_OK: {
            log_debug("BLE: response waiting powerup");
//            return MS_BLE_WAIT_TYPE;
            // ignore any input, wait for powerup timer as might be from a previous uart user
            return SM_STATE_CURRENT;
        }
        case ME_BLE_OFF: {
            // gave up
            return MS_BLE_OFF;
        }            
        default: {
            log_debug("BLE:? %d in WaitPowerOn", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}

// power on, send WHO to check comm ok
static SM_STATE_ID_t State_Starting(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Send who command to ensure comm ok
            wskt_write(ctx->cnx, (uint8_t*)BLE_CHECK, strlen(BLE_CHECK));
            sm_timer_start(ctx->mySMId, 500);
            log_debug("BLE: check who");
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            // No response to WHO, back to off
            log_warn("BLE: no who");
            // if cb call it
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLE_COMM_FAIL, NULL);
            }
            return MS_BLE_OFF;
        }
        case ME_BLE_RET_OK: {
//            log_debug("BLE: response");
            // Normally the who response is the data value. Store it for later
            ctx->cardType = (uint32_t)data;
            // if cb call it
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLE_COMM_OK, NULL);
            }

            return MS_BLE_ON;
        }
        case ME_BLE_OFF: {
            // gave up - directly off
            return MS_BLE_OFF;
        }            
        default: {
            log_debug("BLE:? %d in Starting", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
static SM_STATE_ID_t State_On(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            return SM_STATE_CURRENT;
        }
        case ME_BLE_ON: {
            // We are on
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLE_COMM_OK, NULL);
            }
            return SM_STATE_CURRENT;
        }
        case ME_BLE_OFF: {
            return MS_BLE_STOPPINGCOMM;
        }            
        case ME_BLE_SCAN: {
            if (ctx->cardType!=TYPE_SCANNER) {
                log_debug("BLE:card says type %d, but we want to scan", ctx->cardType);
                return MS_BLE_WAIT_TYPE_SCAN;
            }
            // Start scanning
            return MS_BLE_SCANNING;
        }
        case ME_BLE_IBEACON: {
            if (ctx->cardType!=TYPE_IBEACON) {
                log_debug("BLE:card says type %d, but we want to ibeacon", ctx->cardType);
                return MS_BLE_WAIT_TYPE_IB;
            }
            // Start beaconning
            return MS_BLE_IBEACON;
        }
        default: {
            log_debug("BLE:? %d in On", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// Put module into scanner mode and wait for response, then go to scanning
static SM_STATE_ID_t State_WaitTypeSetScanner(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            sm_timer_start(ctx->mySMId, 500);
            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_SCANNER, strlen(BLE_TYPE_SCANNER));
            log_debug("BLE: set type scanner");
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            ctx->cardType=TYPE_SCANNER;     // Assume it changed ok
            return MS_BLE_SCANNING;
        }
        case ME_BLE_RET_OK: {
//            log_debug("BLE: type change ok");
            ctx->cardType=TYPE_SCANNER;     // Assume it changed ok
            return MS_BLE_SCANNING;
        }
        case ME_BLE_OFF: {
            // gave up
            return MS_BLE_OFF;
        }            
        default: {
            log_debug("BLE:? %d in WaitTypeSetScanner", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// Put module into scanner mode and wait for response
static SM_STATE_ID_t State_WaitTypeSetIBeacon(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            sm_timer_start(ctx->mySMId, 500);
            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_IBEACON, strlen(BLE_TYPE_IBEACON));
            log_debug("BLE: set type IB");
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            ctx->cardType=TYPE_IBEACON;     // Assume it changed ok
            return MS_BLE_IBEACON;
        }
        case ME_BLE_RET_OK: {
//            log_debug("BLE: type change ok");
            ctx->cardType=TYPE_IBEACON;     // Assume it changed ok
            return MS_BLE_IBEACON;
        }
        case ME_BLE_OFF: {
            // gave up
            return MS_BLE_OFF;
        }            
        default: {
            log_debug("BLE:? %d in WaitTypeSetIBeacon", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
static SM_STATE_ID_t State_Scanning(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            resetIBList();      // throw away old list
            // poll mode
            wskt_write(ctx->cnx, (uint8_t*)BLE_POLL, strlen(BLE_POLL));
            // And start the scanning
            if (ctx->uuid!=NULL) {
                // TODO start srting is ",UUID"
                wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_START, strlen(BLE_SCAN_START));
            } else {
                wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_START, strlen(BLE_SCAN_START));
            }
            // poll every second
            sm_timer_start(ctx->mySMId, 1000);
            log_info("BLE:scanning[%d->%d]", ctx->majorStart, ctx->majorEnd);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            // Stop scanner
            wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_STOP, strlen(BLE_SCAN_STOP));
            log_info("BLE:end scan %d %d %d %d %d", ctx->nbRxNew, ctx->nbRxNewR, ctx->nbRxUpdate, ctx->nbRxNoSpace, ctx->nbRxBadMajor);
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            wskt_write(ctx->cnx, (uint8_t*)BLE_POLL, strlen(BLE_POLL));
            // Every second, poll for ibeacon data
            sm_timer_start(ctx->mySMId, 1000);
            return SM_STATE_CURRENT;
        }
        // go direct to off if requested
        case ME_BLE_OFF: {
            return MS_BLE_STOPPINGCOMM;
        }            
        case ME_BLE_IBEACON: {
            // go direct to ibeaconning
            return MS_BLE_IBEACON;
        }            

        case ME_BLE_STOP: {            
            // Back to idle but on mode (exit does the stop)
            return MS_BLE_ON;
        }
        case ME_BLE_RET_OK: {
            // ignore any non-ib return data
            return SM_STATE_CURRENT;
        }
        case ME_BLE_RET_ERR: {
            // retry scan command
            if (ctx->uuid!=NULL) {
                // TODO start srting is ",UUID"
                wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_START, strlen(BLE_SCAN_START));
            } else {
                wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_START, strlen(BLE_SCAN_START));
            }

            return SM_STATE_CURRENT;
        }
        case ME_BLE_UPDATE: {
            // uart callback parsed lines and updates the list of currently visible ibeacons
            // data is directly the index in list TODO could be out of date.... too bad...
            ibeacon_data_t* ib=getIB((int)data);
            // if cb call it
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLE_SCAN_RX_IB, ib);
            }
            return SM_STATE_CURRENT;
        }
        default: {
            log_debug("BLE:? %d in Scaning", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
static SM_STATE_ID_t State_IBeacon(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            log_info("BLE:ibeaconning....");
            // Config to type ibeacon and will start beaconning
            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_IBEACON, strlen(BLE_TYPE_IBEACON));
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            // back to scanner mode
            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_SCANNER, strlen(BLE_TYPE_SCANNER));
            log_info("BLE:end ibeacon");
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            return SM_STATE_CURRENT;
        }
        case ME_BLE_STOP: {            
            // Back to on+idle mode, this will stop the ibeaconning
            return MS_BLE_ON;
        }
        case ME_BLE_OFF: {
            return MS_BLE_STOPPINGCOMM;
        }            
        case ME_BLE_IBEACON: {
            // we are already mate
            return SM_STATE_CURRENT;
        }            

        default: {
            log_debug("BLE:? %d in ibeaconning", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// delay before closing connection for any last requests
static SM_STATE_ID_t State_StoppingComm(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Ensure asked to stop
            wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_STOP, strlen(BLE_SCAN_STOP));
            sm_timer_start(ctx->mySMId, 500);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            return MS_BLE_OFF;
        }
        case ME_BLE_OFF: {
            // gave up
            return MS_BLE_OFF;
        }            
        case ME_BLE_ON: {
            // comm ok already
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLE_COMM_OK, NULL);
            }
            return MS_BLE_ON;
        }
        default: {
            log_debug("BLE:? %d in StoppingComm", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// State table : note can be in any order as the 'id' field is what maps the state id to the rest
static const SM_STATE_t _bleSM[MS_BLE_LAST] = {
    {.id=MS_BLE_OFF,        .name="BleOff",       .fn=State_Off},
    {.id=MS_BLE_WAITPOWERON,.name="BleWaitPower", .fn=State_WaitPoweron},
    {.id=MS_BLE_WAIT_TYPE_SCAN,  .name="BleWaitTypeScan", .fn=State_WaitTypeSetScanner},
    {.id=MS_BLE_WAIT_TYPE_IB,  .name="BleWaitTypeIB", .fn=State_WaitTypeSetIBeacon},
    {.id=MS_BLE_STARTING,   .name="BleStarting",  .fn=State_Starting},    
    {.id=MS_BLE_ON,         .name="BleOnIdle", .fn=State_On},
    {.id=MS_BLE_SCANNING,   .name="BleScanning", .fn=State_Scanning},    
    {.id=MS_BLE_IBEACON,    .name="BleBeaconning", .fn=State_IBeacon},    
    {.id=MS_BLE_STOPPINGCOMM, .name="BleStopping", .fn=State_StoppingComm},    
};

// Called from sysinit via reference in pkg.yml
void* wble_mgr_init(const char* dname, uint32_t baudrate, int8_t pwrPin, int8_t uartSelect) {
    // Ignore multiple inits as this code can't handle them....
    // TODO if required to support multiple BLEs on multiple UARTs....
    if (_ctx.uartDevice!=NULL) {
        if (strcmp(_ctx.uartDevice, dname)==0) {
            // already inited on same device... not an issue
            log_debug("wble: device %s already inited", dname);
            return &_ctx;
        } else {
            log_debug("wble: FAIL init %s but already on %s", dname, _ctx.uartDevice);
            assert(0);
        }
    }
    _ctx.uartDevice = dname;
    _ctx.baudrate = baudrate;
    _ctx.uartSelect=uartSelect;
    _ctx.pwrPin = pwrPin;
    if (_ctx.pwrPin>=0) {
        // Note 1 is OFF so start with it off
        GPIO_define_out("blepower", _ctx.pwrPin, 1, LP_DEEPSLEEP);
    }
    // create event with arg pointing to our line buffer
    // TODO how to tell driver limit of size of buffer???
    _ctx.myUARTEvent.ev_cb = wble_mgr_rxcb;
    _ctx.myUARTEvent.ev_arg = _ctx.rxbuf;
    // TODO Do we really need a whole task/eventQ just to rx the uart strings? as it only ever sends sm events as a result? (so callbacks run on the SM task...)
    // Nah, just use default task/default eventq
//    os_task_init(&wble_mgr_task_str, "wble_task", wble_mgr_task, NULL, WBLE_TASK_PRIO,
//               OS_WAIT_FOREVER, _wble_task_stack, WBLE_TASK_STACK_SZ);
    // Create eventQ
//    os_eventq_init(&_ctx.myEQ);
    // Create task 
    _ctx.mySMId = sm_init("blemgr", _bleSM, MS_BLE_LAST, MS_BLE_OFF, &_ctx);
    sm_start(_ctx.mySMId);
    return &_ctx;
}

void wble_start(void* c, WBLE_CB_FN_t cb) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    ctx->cbfn = cb;
    sm_sendEvent(ctx->mySMId, ME_BLE_ON, NULL);
}
void wble_scan_start(void* c, const uint8_t* uuid, uint16_t majorStart, uint16_t majorEnd) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    memcpy(ctx->uuid, uuid, sizeof(_ctx.uuid));
    ctx->majorStart = majorStart;
    ctx->majorEnd = majorEnd;
    sm_sendEvent(ctx->mySMId, ME_BLE_SCAN, NULL);

}
void wble_scan_stop(void* c) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    sm_sendEvent(ctx->mySMId, ME_BLE_STOP, NULL);
}

void wble_ibeacon_start(void* c, const uint8_t* uuid, uint16_t maj, uint16_t min, uint8_t extra) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    sm_sendEvent(ctx->mySMId, ME_BLE_IBEACON, NULL);
}

void wble_ibeacon_stop(void* c) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    sm_sendEvent(ctx->mySMId, ME_BLE_STOP, NULL);
}

void wble_stop(void* c) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    sm_sendEvent(ctx->mySMId, ME_BLE_OFF, NULL);
}

// get number of ibs we have seen so far
int wble_getNbIB(void* c) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    return ctx->ibListSz;
}
// get the list of IBs (best to do this once stopped)
ibeacon_data_t* wble_getIBList(void* c, int* sz) {
    assert(c!=NULL);
    // If error during scan/ble comm etc, then return NULL to indicate this
    // TODO
    struct blectx* ctx = (struct blectx*)c;
    *sz = ctx->ibListSz;
    return &ctx->ibList[0];
}

//Copy 'best' sz elements into the given list (of max sz). Return actual number copied
int wble_getSortedIBList(void* c, int sz, ibeacon_data_t* list) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    assert(list!=NULL);
    // initialise 'worst' rssi to be the max it could be
    int prevRssi = 0;
    //For each place in the output list
    for(int insert=0;insert<sz;) {
        // pass thru list getting element that is best but worse than previous
        int bestRssi = -150;
        int idxToGet = -1;
        for(int i=0;i<ctx->ibListSz;i++) {
            if (ctx->ibList[i].rssi>bestRssi && ctx->ibList[i].rssi<prevRssi) {
                idxToGet = i;
                bestRssi = ctx->ibList[i].rssi;
            }
        }
        if (idxToGet>=0) {
            prevRssi = bestRssi;
            // if N with same RSSI, idxToGet shows the first in the list.
            // go from there to the end, and add all those with the same 'best' rssi.
            for(int i=idxToGet;i<ctx->ibListSz;i++) {
                if (ctx->ibList[i].rssi == bestRssi) {
                    list[insert].major = ctx->ibList[i].major;
                    list[insert].minor = ctx->ibList[i].minor;
                    list[insert].rssi = ctx->ibList[i].rssi;
                    list[insert].extra = ctx->ibList[i].extra;
                    insert++;
                    // check we don't overfill the list
                    if (insert>=sz) {
                        return insert;
                    }
                }
            }
        } else {
            // all done although not filled output list, return actual number
            return insert;
        }
    }
    return sz;
}
/*
// task just runs the callbacks
static void wble_mgr_task(void* arg) {
    while(1) {
        os_eventq_run(&_ctx.myEQ);
    }
}
*/
// manage IB list
static void resetIBList() {
    _ctx.ibListSz = 0;
    // reset stats also
    _ctx.nbRxNew=0;
    _ctx.nbRxNewR=0;
    _ctx.nbRxUpdate=0;
    _ctx.nbRxNoSpace=0;
    _ctx.nbRxBadMajor=0;
}
static ibeacon_data_t* getIB(int idx) {
    if (idx<0 || idx>=_ctx.ibListSz) {
        return NULL;
    }
    return &_ctx.ibList[idx];
}

// Add scanned IB to list if not already present  else update it
static int addIB(ibeacon_data_t* ibp) {
    // Find if already in list, update if so
    int worstRSSIIdx = 0;
    int8_t worstRSSI = 0;
    for(int i=0;i<_ctx.ibListSz; i++) {
        if (ibp->major==_ctx.ibList[i].major &&
            ibp->minor==_ctx.ibList[i].minor) {
                _ctx.ibList[i].rssi = ibp->rssi;
                _ctx.ibList[i].extra = ibp->extra;
                _ctx.nbRxUpdate++;
                return i;
        }
        // Find worst rssi in list while we're here
        if (_ctx.ibList[i].rssi < worstRSSI) {
            worstRSSI = _ctx.ibList[i].rssi;
            worstRSSIIdx = i;
        }
    }
    // space in list to add it?
    if (_ctx.ibListSz<MAX_IBEACONS) {
        // yes. add to end
        _ctx.ibList[_ctx.ibListSz].major = ibp->major;
        _ctx.ibList[_ctx.ibListSz].minor = ibp->minor;
        _ctx.ibList[_ctx.ibListSz].rssi = ibp->rssi;
        _ctx.ibList[_ctx.ibListSz].extra = ibp->extra;
        _ctx.ibListSz++;
        _ctx.nbRxNew++;
        return (_ctx.ibListSz - 1);
    }
    // no, see if it is better rssi than the worst guy, and replace him if so
    if (ibp->rssi > worstRSSI) {
        _ctx.ibList[worstRSSIIdx].major = ibp->major;
        _ctx.ibList[worstRSSIIdx].minor = ibp->minor;
        _ctx.ibList[worstRSSIIdx].rssi = ibp->rssi;
        _ctx.ibList[worstRSSIIdx].extra = ibp->extra;
        _ctx.nbRxNewR++;
        return worstRSSIIdx;
    }
    // tant pis
    _ctx.nbRxNoSpace++;
    return -1;
}

// callback every time the socket gives us a new line of data from the GPS
// Guarenteed to be mono-thread
static void wble_mgr_rxcb(struct os_event* ev) {
    // ev->arg is our line buffer
    const char* line = (char*)(ev->ev_arg);
    assert(line!=NULL);
    int slen = strnlen(line, 100);
    if (slen==0) {
        // too short line ignore
        return;
    }
    /* 
    if (slen<6) {
        log_debug("BLE:[%s]", line);
    } else {
        log_debug("BLE:[%c%c%c%c]", line[0],line[1],line[2],line[3]);
    }
      */
    // Parse line
    // If its "OK" or "ERROR" its the return from previous command
    if (strncmp(line, "OK", 2)==0) {
        sm_sendEvent(_ctx.mySMId, ME_BLE_RET_OK, NULL);
    } else if (strncasecmp(line, "ERROR", 5)==0) {
       sm_sendEvent(_ctx.mySMId, ME_BLE_RET_ERR, NULL);
    } else if (strlen(line)<15) {
        // Any none OK/ERROR/ble info line is considered as OK
        uint32_t val = atoi(line);
//        sm_sendEvent(_ctx.mySMId, ME_BLE_RET_INT, (void*)val);
        log_debug("BLE:[%s]", line);
        sm_sendEvent(_ctx.mySMId, ME_BLE_RET_OK, (void*)val);
    } else {
        // Parse it as ibeacon data
        ibeacon_data_t ib;
        uint32_t uuid;      // we don't care at this point
        // <MMMM>,<mmmm>,<RR>,<EX>,<UUUU>
        if (sscanf(line, "%4x,%4x,%2x,%2x,%4x", (int*)&ib.major, (int*)&ib.minor, (int*)&ib.extra, (int*)&ib.rssi, (int*)&uuid)<0) {
            log_debug("BLE:bad parse");
        } else {
            // Is major vale between majorStart and majorEnd filters?
            if (ib.major>=_ctx.majorStart && ib.major<=_ctx.majorEnd) {
                // Add to ibeacon fifo
                int idx = addIB(&ib);
                if (idx>=0) {
                    // Tell SM
                    sm_sendEvent(_ctx.mySMId, ME_BLE_UPDATE, (void*)idx);
                } else {
                    log_debug("BLE:saw %4x,%4x list full",ib.major, ib.minor);
                }
            } else {
                _ctx.nbRxBadMajor++;
//                log_debug("BLE:saw ib %4x,%4x but outside major filter range",ib.major, ib.minor);
            }
        }
    }

}

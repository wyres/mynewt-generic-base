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
#include "wyres-generic/uartSelector.h"
#include "wyres-generic/sm_exec.h"

#define WBLE_UART    MYNEWT_VAL(WBLE_UART)
#define WBLE_TASK_PRIO       MYNEWT_VAL(WBLE_TASK_PRIO)
#define WBLE_TASK_STACK_SZ   OS_STACK_ALIGN(256)
#define MAX_IBEACONS    MYNEWT_VAL(WBLE_MAXIBS)

static char* BLE_CHECK="AT+WHO\r\n";
//static char* BLE_CONFIG="AT+CONFIG";
static char* BLE_POLL="AT+POLL\r\n";
//static char* BLE_PUSH="AT+PUSH";
static char* BLE_SCAN_START="AT+START\r\n";
static char* BLE_SCAN_STOP="AT+STOP\r\n";

static char* BLE_TYPE_SCANNER="AT+TYPE=0\r\n";
static char* BLE_TYPE_IBEACON="AT+TYPE=2\r\n";

static os_stack_t _wble_task_stack[WBLE_TASK_STACK_SZ];
static struct os_task wble_mgr_task_str;

static struct blectx {
    struct os_event myUARTEvent;
    struct os_eventq myEQ;
    struct os_mutex dataMutex;
    SM_ID_t mySMId;
    const char* uartDevice;
    int8_t pwrPin;
    int8_t uartSelect;
    wskt_t* cnx;
    uint8_t rxbuf[WSKT_BUF_SZ+1];
    uint32_t lastDataTime;
    WBLE_CB_FN_t cbfn;
    uint8_t uuid[16];
    uint8_t ibListSz;
    ibeacon_data_t ibList[MAX_IBEACONS];
} _ctx = {
    .lastDataTime = 0,
    .cbfn=NULL,
};

// predeclare privates
static void wble_mgr_task(void* arg);
static void wble_mgr_rxcb(struct os_event* ev);
// manage IB list
static void resetIBList();
static ibeacon_data_t*  getIB(int idx);
// Add scanned IB to list if not already present else update it
static int addIB(ibeacon_data_t* ibp);

// State machine for BLE control
enum BLEStates { MS_BLE_OFF, MS_BLE_WAITPOWERON, MS_BLE_WAIT_TYPE, MS_BLE_STARTING, MS_BLE_ON, MS_BLE_SCANNING, MS_BLE_IBEACON, MS_BLE_STOPPING, MS_BLE_LAST };
enum BLEEvents { ME_BLE_ON, ME_BLE_OFF, ME_BLE_SCAN, ME_BLE_IBEACON, ME_BLE_STOP, ME_BLE_RET_OK, ME_BLE_RET_ERR, ME_BLE_RET_INT, ME_BLE_UPDATE };
static SM_STATE_ID_t State_Off(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Ensure no open cnx and tuern off the power
            if (ctx->cnx!=NULL) {
                wskt_close(&ctx->cnx);
            }
    
            if (ctx->pwrPin>=0) {
                log_debug("ble power OFF using pin %d", ctx->pwrPin);
                GPIO_write(ctx->pwrPin, 1);     // yup pull UP for OFF
            } else {
                log_debug("ble power always on?");
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
            // initialise comms to the ble via the uart like comms device defined in syscfg
            ctx->cnx = wskt_open(ctx->uartDevice, &ctx->myUARTEvent, &ctx->myEQ);
            assert(ctx->cnx!=NULL);
            // Select it as UART device (if required)
            if (ctx->uartSelect>=0) {
                uart_select(ctx->uartSelect);
            }
                // Set baud rate
            wskt_ioctl_t cmd = {
                .cmd = IOCTL_SET_BAUD,
                .param = MYNEWT_VAL(BLE_UART_BAUDRATE),
            };
            wskt_ioctl(ctx->cnx, &cmd);
            // Power up using power pin?
            if (ctx->pwrPin<0) {
                log_debug("ble power always on?");
                return MS_BLE_WAIT_TYPE;        // no need to wait for poweron
            }
            log_debug("ble power ON using pin %d", ctx->pwrPin);
            GPIO_write(ctx->pwrPin, 0);     // yup pull down for ON
            return MS_BLE_WAITPOWERON;
        }

        default: {
            log_debug("?evt %d in Off", e);
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
            sm_timer_start(ctx->mySMId, 500);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            return MS_BLE_WAIT_TYPE;
        }
        case ME_BLE_RET_OK: {
//            log_debug("ble response in wait power");
            return MS_BLE_WAIT_TYPE;
//            return SM_STATE_CURRENT;
        }
        case ME_BLE_OFF: {
            // gave up
            return MS_BLE_OFF;
        }            
        default: {
            log_debug("?evt %d in starting", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// Put module into scanner mode and wait for response
static SM_STATE_ID_t State_WaitTypeSet(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            sm_timer_start(ctx->mySMId, 500);
            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_SCANNER, strlen(BLE_TYPE_SCANNER));
           return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            return MS_BLE_STARTING;
        }
        case ME_BLE_RET_OK: {
//            log_debug("ble type change ok");
            return MS_BLE_STARTING;
        }
        case ME_BLE_OFF: {
            // gave up
            return MS_BLE_OFF;
        }            
        default: {
            log_debug("?evt %d in starting", e);
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
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            // No response to WHO, back to off
            log_warn("ble no response to who");
            // if cb call it
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLE_COMM_FAIL, NULL);
            }
            return MS_BLE_OFF;
        }
        case ME_BLE_RET_OK: {
//            log_debug("ble response");
            // if cb call it
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLE_COMM_OK, NULL);
            }

            return MS_BLE_ON;
        }
        case ME_BLE_OFF: {
            // gave up
            return MS_BLE_OFF;
        }            
        default: {
            log_debug("?evt %d in starting", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
static SM_STATE_ID_t State_On(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Dont change type if already in the right one (as causes the ble to reset and not respond to scan command)
            // Ensure in scan mode (but shouldn't be actively scanning)
//            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_SCANNER, strlen(BLE_TYPE_SCANNER));
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
            return MS_BLE_OFF;
        }            
        case ME_BLE_SCAN: {
            // Start scanning
            return MS_BLE_SCANNING;
        }
        case ME_BLE_IBEACON: {
            // Start beaconning
            return MS_BLE_IBEACON;
        }
        default: {
            log_debug("?evt %d in On", e);
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
            log_info("scanning..");
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            // Stop scanner
            wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_STOP, strlen(BLE_SCAN_STOP));
            log_info("end scan");
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
            return MS_BLE_OFF;
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
            log_debug("?evt %d in Scaning", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
static SM_STATE_ID_t State_IBeacon(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            log_info("ibeaconning....");
            // Config to type ibeacon and will start beaconning
            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_IBEACON, strlen(BLE_TYPE_IBEACON));
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            // back to scanner mode
            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_SCANNER, strlen(BLE_TYPE_SCANNER));
            log_info("end ibeacon");
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            return SM_STATE_CURRENT;
        }
        case ME_BLE_STOP: {            
            // Back to on+idle mode, this will stop the ibeaconning
            return MS_BLE_ON;
        }
                // go direct to off if requested
        case ME_BLE_OFF: {
            return MS_BLE_OFF;
        }            
        case ME_BLE_IBEACON: {
            // we are already mate
            return SM_STATE_CURRENT;
        }            

        default: {
            log_debug("?evt %d in ibeaconning", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}

// State table : note can be in any order as the 'id' field is what maps the state id to the rest
static SM_STATE_t _bleSM[MS_BLE_LAST] = {
    {.id=MS_BLE_OFF,        .name="BleOff",       .fn=State_Off},
    {.id=MS_BLE_WAITPOWERON,.name="BleWaitPower", .fn=State_WaitPoweron},
    {.id=MS_BLE_WAIT_TYPE,  .name="BleWaitType", .fn=State_WaitTypeSet},
    {.id=MS_BLE_STARTING,   .name="BleStarting",  .fn=State_Starting},    
    {.id=MS_BLE_ON,         .name="BleOnIdle", .fn=State_On},
    {.id=MS_BLE_SCANNING,   .name="BleScanning", .fn=State_Scanning},    
    {.id=MS_BLE_IBEACON,    .name="BleBeaconning", .fn=State_IBeacon},    
};

// Called from sysinit via reference in pkg.yml
void wble_mgr_init(const char* dname, int8_t pwrPin, int8_t uartSelect) {
    _ctx.uartDevice = dname;
    _ctx.uartSelect=uartSelect;
    _ctx.pwrPin = pwrPin;
    if (_ctx.pwrPin>=0) {
        // Note 1 is OFF so start with it off
        GPIO_define_out("blepower", _ctx.pwrPin, 1, LP_DEEPSLEEP);
    }
    // Create eventQ
    os_eventq_init(&_ctx.myEQ);
    // create event with arg pointing to our line buffer
    // TODO how to tell driver limit of size of buffer???
    _ctx.myUARTEvent.ev_cb = wble_mgr_rxcb;
    _ctx.myUARTEvent.ev_arg = _ctx.rxbuf;
    // Create task 
    os_task_init(&wble_mgr_task_str, "wble_task", wble_mgr_task, NULL, WBLE_TASK_PRIO,
               OS_WAIT_FOREVER, _wble_task_stack, WBLE_TASK_STACK_SZ);
    _ctx.mySMId = sm_init("blemgr", _bleSM, MS_BLE_LAST, MS_BLE_OFF, &_ctx);
    sm_start(_ctx.mySMId);

}

void wble_start(WBLE_CB_FN_t cb) {
    _ctx.cbfn = cb;
    sm_sendEvent(_ctx.mySMId, ME_BLE_ON, NULL);
}
void wble_scan_start(const char* uuid) {
    memcpy(_ctx.uuid, uuid, sizeof(_ctx.uuid));
    sm_sendEvent(_ctx.mySMId, ME_BLE_SCAN, NULL);

}
void wble_scan_stop() {
    sm_sendEvent(_ctx.mySMId, ME_BLE_STOP, NULL);
}

void wble_ibeacon_start(const char* uuid, uint16_t maj, uint16_t min, uint8_t extra) {
    sm_sendEvent(_ctx.mySMId, ME_BLE_IBEACON, NULL);
}

void wble_ibeacon_stop() {
    sm_sendEvent(_ctx.mySMId, ME_BLE_STOP, NULL);
}

void wble_stop() {
    sm_sendEvent(_ctx.mySMId, ME_BLE_OFF, NULL);
}

// get the list of IBs (best to do this once stopped)
ibeacon_data_t* wble_getIBList(uint8_t* sz) {
    *sz = _ctx.ibListSz;
    return &_ctx.ibList[0];
}

//Copy 'best' sz elements into the given list (of max sz). Return actual number copied
uint8_t wble_getSortedIBList(uint8_t sz, ibeacon_data_t* list) {
    assert(list!=NULL);
    int prevRssi = 0;
    for(int insert=0;insert<sz;insert++) {
        // pass thru list getting element that is best but worse than previous
        int bestRssi = -150;
        int idxToGet = -1;
        for(int i=0;i<_ctx.ibListSz;i++) {
            if (_ctx.ibList[i].rssi>bestRssi && _ctx.ibList[i].rssi<prevRssi) {
                idxToGet = i;
                bestRssi = _ctx.ibList[i].rssi;
            }
        }
        if (idxToGet>=0) {
            prevRssi = bestRssi;
            list[insert].major = _ctx.ibList[idxToGet].major;
            list[insert].minor = _ctx.ibList[idxToGet].minor;
            list[insert].rssi = _ctx.ibList[idxToGet].rssi;
            list[insert].extra = _ctx.ibList[idxToGet].extra;
        } else {
            // all done
            return insert;
        }
    }
    return sz;
}

// task just runs the callbacks
static void wble_mgr_task(void* arg) {
    while(1) {
        os_eventq_run(&_ctx.myEQ);
    }
}
// manage IB list
static void resetIBList() {
    _ctx.ibListSz = 0;
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
                _ctx.ibList[i].major = ibp->major;
                _ctx.ibList[i].minor = ibp->minor;
                _ctx.ibList[i].rssi = ibp->rssi;
                _ctx.ibList[i].extra = ibp->extra;
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
        return (_ctx.ibListSz - 1);
    }
    // no, see if it is better rssi than the worst guy, and replace him if so
    if (ibp->rssi > worstRSSI) {
        _ctx.ibList[worstRSSIIdx].major = ibp->major;
        _ctx.ibList[worstRSSIIdx].minor = ibp->minor;
        _ctx.ibList[worstRSSIIdx].rssi = ibp->rssi;
        _ctx.ibList[worstRSSIIdx].extra = ibp->extra;
        return worstRSSIIdx;
    }
    // tant pis
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
        log_debug("ble[%s]", line);
    } else {
        log_debug("ble[%c%c%c%c]", line[0],line[1],line[2],line[3]);
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
//        uint32_t val = atoi(line);
//        sm_sendEvent(_ctx.mySMId, ME_BLE_RET_INT, (void*)val);
        sm_sendEvent(_ctx.mySMId, ME_BLE_RET_OK, NULL);
    } else {
        // Parse it as ibeacon data
        ibeacon_data_t ib;
        uint32_t uuid;
        // <MMMM>,<mmmm>,<RR>,<EX>,<????>
        if (sscanf(line, "%4x,%4x,%2x,%2x,%2x", (int*)&ib.major, (int*)&ib.minor, (int*)&ib.extra, (int*)&ib.rssi, (int*)&uuid)<0) {
            log_debug("bad parse");
        } else {
            // Add to ibeacon fifo
            int idx = addIB(&ib);
            if (idx>=0) {
                // Tell SM
                sm_sendEvent(_ctx.mySMId, ME_BLE_UPDATE, (void*)idx);
            } else {
                log_debug("saw ib %4x,%4x but list full",ib.major, ib.minor);
            }
        }
    }

}

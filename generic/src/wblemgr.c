/**
 Wyres private code
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
#define MAX_IBEACONS    16  //MYNEWT_VAL(WBLE_MAXIBS)

static char* BLE_CHECK="AT+WHO";
//static char* BLE_CONFIG="AT+CONFIG";
static char* BLE_POLL="AT+POLL";
//static char* BLE_PUSH="AT+PUSH";
static char* BLE_SCAN_START="AT+START";
static char* BLE_SCAN_STOP="AT+STOP";

static char* BLE_TYPE_SCANNER="AT+TYPE=0";
static char* BLE_TYPE_IBEACON="AT+TYPE=2";

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
enum BLEStates { MS_BLE_OFF, MS_BLE_STARTING, MS_BLE_ON, MS_BLE_SCANNING, MS_BLE_IBEACON, MS_BLE_STOPPING, MS_BLE_LAST };
enum BLEEvents { ME_BLE_ON, ME_BLE_OFF, ME_BLE_SCAN, ME_BLE_IBEACON, ME_BLE_STOP, ME_BLE_RET_OK, ME_BLE_RET_ERR, ME_BLE_RET_INT, ME_BLE_UPDATE };
SM_STATE_ID_t State_Off(void* arg, int e, void* data) {
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
            // Power up using power pin
            if (ctx->pwrPin>=0) {
                log_debug("ble power ON using pin %d", ctx->pwrPin);
                GPIO_write(ctx->pwrPin, 0);     // yup pull down for ON
            } else {
                log_debug("ble poweralways on?");
            }
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
            return MS_BLE_STARTING;
        }

        default: {
            log_debug("unknown event %d in state Off", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// power on, send WHO to check comm ok, then TYPE to be in scan mode (but not scanning)
SM_STATE_ID_t State_Starting(void* arg, int e, void* data) {
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
            log_debug("ble no response to who");
            return MS_BLE_OFF;
        }
        case ME_BLE_RET_INT: {
            // Should be a value (passed directly in data) - we'll assume any response is ok
            log_debug("ble who response:%d",(uint32_t)data);
            return MS_BLE_ON;
        }
        default: {
            log_debug("unknown event %d in state starting", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
SM_STATE_ID_t State_On(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Ensure in scan mode (but shouldn't be actively scanning)
            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_SCANNER, strlen(BLE_TYPE_SCANNER));
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            return SM_STATE_CURRENT;
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
            log_debug("unknown event %d in state On", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
SM_STATE_ID_t State_Scanning(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            resetIBList();      // throw away old list
            // Already in scanner mode... but force anyway
            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_SCANNER, strlen(BLE_TYPE_SCANNER));
            // Ask to start scanning in poll mode
            wskt_write(ctx->cnx, (uint8_t*)BLE_POLL, strlen(BLE_POLL));
            if (ctx->uuid!=NULL) {
                // TODO start srting is ",UUID"
                wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_START, strlen(BLE_SCAN_START));
            } else {
                wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_START, strlen(BLE_SCAN_START));
            }

            // Every second, poll for ibeacon data
            sm_timer_start(ctx->mySMId, 1000);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            // Stop scanner
            wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_STOP, strlen(BLE_SCAN_STOP));
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            wskt_write(ctx->cnx, (uint8_t*)BLE_POLL, strlen(BLE_POLL));
            sm_timer_start(ctx->mySMId, 1000);
            return SM_STATE_CURRENT;
        }
        case ME_BLE_STOP: {            
            // Back to idle but on mode
            return MS_BLE_ON;
        }
        case ME_BLE_UPDATE: {
            // uart callback parsed lines and updates the list of currently visible ibeacons
            // data is directly the index in list TODO could be out of date.... too bad...
            ibeacon_data_t* ib=getIB((int)data);
            // if cb call it
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(ib);
            }
            return SM_STATE_CURRENT;
        }
        default: {
            log_debug("unknown event %d in state Scanning", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
SM_STATE_ID_t State_IBeacon(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Config to type ibeacon and will start beaconning
            wskt_write(ctx->cnx, (uint8_t*)BLE_TYPE_IBEACON, strlen(BLE_TYPE_IBEACON));
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            return SM_STATE_CURRENT;
        }
        case ME_BLE_STOP: {            
            // Back to on+idle mode, this will stop the ibeaconning
            return MS_BLE_ON;
        }
        default: {
            log_debug("unknown event %d in state ibeaconning", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}

// State table : note can be in any order as the 'id' field is what maps the state id to the rest
SM_STATE_t _bleSM[MS_BLE_LAST] = {
    {.id=MS_BLE_OFF,        .name="BleOff",       .fn=State_Off},
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

void wble_start() {
    sm_sendEvent(_ctx.mySMId, ME_BLE_ON, NULL);
}
void wble_scan_start(const char* uuid,  WBLE_CB_FN_t cb) {
    _ctx.cbfn = cb;
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

// Add scanned IB to list if not already present else update it
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
    if (strnlen(line, 10)==0) {
        // too short line ignore
        return;
    }
    log_debug("ble[%c%c%c%c%c%c]", line[0],line[1],line[2],line[3],line[4],line[5]);
    // Parse line
    // If its "OK" or "ERROR" its the return from previous command
    if (strncmp(line, "OK", 2)==0) {
        sm_sendEvent(_ctx.mySMId, ME_BLE_RET_OK, NULL);
    } else if (strncasecmp(line, "ERROR", 5)==0) {
       sm_sendEvent(_ctx.mySMId, ME_BLE_RET_ERR, NULL);
    } else if (strlen(line)<5) {
        // Try it as an int
        uint32_t val = atoi(line);
        sm_sendEvent(_ctx.mySMId, ME_BLE_RET_INT, (void*)val);
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

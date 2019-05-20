/**
 Wyres private code
 */

#include "os/os.h"
#include "wyres-generic/wutils.h"
#include "wyres-generic/wskt_user.h"
#include "wyres-generic/wblemgr.h"

#define WBLE_UART    MYNEWT_VAL(WBLE_UART)
#define WBLE_TASK_PRIO       MYNEWT_VAL(WBLE_TASK_PRIO)
#define WBLE_TASK_STACK_SZ   OS_STACK_ALIGN(256)

static char* BLE_SCAN="AT+BLESCAN";

static os_stack_t _wble_task_stack[WBLE_TASK_STACK_SZ];
static struct os_task wble_mgr_task_str;

static struct os_event _myEvent;
static struct os_eventq _myEQ;
static const char* _uartDevice;
static wskt_t* _cnx;
static uint8_t _rxbuf[WSKT_BUF_SZ+1];
static WBLE_CB_FN_t _cbfn;

// predeclare privates
static void wble_mgr_task(void* arg);
static void wble_mgr_rxcb(struct os_event* ev);

// Called from sysinit via reference in pkg.yml
void wble_mgr_init(const char* dname) {
    _uartDevice = dname;
    // Create eventQ
    os_eventq_init(&_myEQ);
    // create event with arg pointing to our line buffer
    // TODO how to tell driver limit of size of buffer???
    _myEvent.ev_cb = wble_mgr_rxcb;
    _myEvent.ev_arg = _rxbuf;
    // Create task 
    os_task_init(&wble_mgr_task_str, "wble_task", wble_mgr_task, NULL, WBLE_TASK_PRIO,
               OS_WAIT_FOREVER, _wble_task_stack, WBLE_TASK_STACK_SZ);
}

// Get current list of SSIDs. Returns 0 for sucess, -1 for failure
int wble_getBeaconList(uint8_t* sz, ibeacon_data_t** list) {
    // TODO 
    // see code in ble_scanner.c in src/sensors in old firmare
    return -1;
}

void wble_start() {
    // initialise comms to the ble via the uart like comms device defined in syscfg
    _cnx = wskt_open(_uartDevice, &_myEvent, &_myEQ);
    assert(_cnx!=NULL);
    // this should have powered it up also
}
void wble_scan_ble(const char* uuid,  WBLE_CB_FN_t cb) {
    _cbfn = cb;
    if (_cnx==NULL) {
        wble_start();
    }
    // Set UUID filter
    if (uuid!=NULL) {
        // TODO
    }
    // Ask ESP to start scanning
    wskt_write(_cnx, (uint8_t*)BLE_SCAN, strlen(BLE_SCAN));
    // Once got a reasonable result we post the event and you can come read the ibeacon list
}

void wble_stop() {
    if (_cnx!=NULL) {
        wskt_close(&_cnx);
    }
}

// task just runs the callbacks
static void wble_mgr_task(void* arg) {
    while(1) {
        os_eventq_run(&_myEQ);
    }
}

// callback every time the socket gives us a new line of data from the GPS
static void wble_mgr_rxcb(struct os_event* ev) {
    // ev->arg is our line buffer
    log_debug((char*)(ev->ev_arg));
    // parse it
    ibeacon_data_t ib;
    // if cb call it
    if (_cbfn!=NULL) {
        (*_cbfn)(&ib);
    }
    // and continue
}

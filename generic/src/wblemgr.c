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

#define WBLE_UART    MYNEWT_VAL(WBLE_UART)
#define WBLE_TASK_PRIO       MYNEWT_VAL(WBLE_TASK_PRIO)
#define WBLE_TASK_STACK_SZ   OS_STACK_ALIGN(256)

static char* BLE_CHECK="AT+WHO";
static char* BLE_SCAN_START="AT+BLESCAN";
static char* BLE_SCAN_STOP="AT+BLESTOP";
static char* BLE_IBEACON_START="AT+IBSTART";
static char* BLE_IBEACON_STOP="AT+IBSTOP";

static os_stack_t _wble_task_stack[WBLE_TASK_STACK_SZ];
static struct os_task wble_mgr_task_str;

static struct {
    struct os_event myUARTEvent;
    struct os_eventq myEQ;
    struct os_mutex dataMutex;

    const char* uartDevice;
    int8_t pwrPin;
    int8_t uartSelect;
    wskt_t* cnx;
    uint8_t rxbuf[WSKT_BUF_SZ+1];
    ibeacon_data_t ibconfig;
    uint32_t lastFixTime;
    WBLE_CB_FN_t cbfn;
    bool commOk;
} _ctx = {
    .lastFixTime = 0,
    .cbfn=NULL,
};

// predeclare privates
static void wble_mgr_task(void* arg);
static void wble_mgr_rxcb(struct os_event* ev);

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
}

void wble_start() {
    // initialise comms to the ble via the uart like comms device defined in syscfg
    _ctx.cnx = wskt_open(_ctx.uartDevice, &_ctx.myUARTEvent, &_ctx.myEQ);
    assert(_ctx.cnx!=NULL);
        _ctx.commOk = false;        // just starting...
    // Power up using power pin
    if (_ctx.pwrPin>=0) {
        log_debug("ble power ON using pin %d", _ctx.pwrPin);
        GPIO_write(_ctx.pwrPin, 0);     // yup pull down for ON
    } else {
        log_debug("ble poweralways on?");
    }
    // Select it as UART device (if required)
    if (_ctx.uartSelect>=0) {
        uart_select(_ctx.uartSelect);
    }
        // Set baud rate
    wskt_ioctl_t cmd = {
        .cmd = IOCTL_SET_BAUD,
        .param = MYNEWT_VAL(GPS_UART_BAUDRATE),
    };
    wskt_ioctl(_ctx.cnx, &cmd);
    // Send who command to ensure comm ok
    wskt_write(_ctx.cnx, (uint8_t*)BLE_CHECK, strlen(BLE_CHECK));

}
void wble_scan_start(const char* uuid,  WBLE_CB_FN_t cb) {
    _ctx.cbfn = cb;
    if (_ctx.cnx==NULL) {
        wble_start();
    }

    // Send who command to ensure comm ok
    wskt_write(_ctx.cnx, (uint8_t*)BLE_CHECK, strlen(BLE_CHECK));

    // Set UUID filter
    if (uuid!=NULL) {
        // TODO
    }
    // Ask  to start scanning
    wskt_write(_ctx.cnx, (uint8_t*)BLE_SCAN_START, strlen(BLE_SCAN_START));
}
void wble_scan_stop() {

    // Send stop command
    wskt_write(_ctx.cnx, (uint8_t*)BLE_SCAN_STOP, strlen(BLE_SCAN_STOP));

    if (_ctx.pwrPin>=0) {
        log_debug("ble power OFF using pin %d", _ctx.pwrPin);
        GPIO_write(_ctx.pwrPin, 1);     // yup pull UP for OFF
    } else {
        log_debug("ble power always on?");
    }

}

void wble_ibeacon_start(const char* uuid, uint16_t maj, uint16_t min, uint8_t extra) {
    wskt_write(_ctx.cnx, (uint8_t*)BLE_IBEACON_START, strlen(BLE_IBEACON_START));

}

void wble_ibeacon_stop() {
    wskt_write(_ctx.cnx, (uint8_t*)BLE_IBEACON_STOP, strlen(BLE_IBEACON_STOP));

}

void wble_stop() {
    if (_ctx.cnx!=NULL) {
        wskt_close(&_ctx.cnx);
    }
}

// task just runs the callbacks
static void wble_mgr_task(void* arg) {
    while(1) {
        os_eventq_run(&_ctx.myEQ);
    }
}

// callback every time the socket gives us a new line of data from the GPS
static void wble_mgr_rxcb(struct os_event* ev) {
    // ev->arg is our line buffer
    const char* line = (char*)(ev->ev_arg);
    assert(line!=NULL);
    if (strnlen(line, 10)==0) {
        // too short line ignore
        return;
    }
    log_debug("gps[%c%c%c%c%c%c]", line[0],line[1],line[2],line[3],line[4],line[5]);

    // parse it
    // if response to WHO, check it
    _ctx.commOk = true;
    // see code in ble_scanner.c in src/sensors in old firmare
    ibeacon_data_t ib;
    // if cb call it
    if (_ctx.cbfn!=NULL) {
        (*_ctx.cbfn)(&ib);
    }
    // and continue
}

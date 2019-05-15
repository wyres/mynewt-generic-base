/**
 Wyres private code
 */

#include "os/os.h"
#include "wyres-generic/wutils.h"
#include "wyres-generic/wskt_user.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/gpsmgr.h"
#include "wyres-generic/lowpowermgr.h"
#include "wyres-generic/uartSelector.h"
#include "wyres-generic/timemgr.h"
#include "wyres-generic/minmea.h"

#define GPS_UART    MYNEWT_VAL(GPS_UART)
#define GPS_TASK_PRIO       MYNEWT_VAL(GPS_TASK_PRIO)
#define GPS_TASK_STACK_SZ   OS_STACK_ALIGN(256)

static os_stack_t _gps_task_stack[GPS_TASK_STACK_SZ];
static struct os_task gps_mgr_task_str;

static struct os_event _myGPSEvent;
static struct os_eventq _gpsMgrEQ;
static struct os_mutex _dataMutex;

static const char* _uartDevice;
static int8_t _pwrPin;
static int8_t _uartSelect;
static wskt_t* _cnx;
static uint8_t _rxbuf[WSKT_BUF_SZ+1];
static gps_data_t _gpsData ={
    .prec = -1,
    .lat = 0,
    .lon = 0,
    .alt = 0,
};
static GPS_CB_FN_t _cbfn=NULL;

// predeclare privates
static void gps_mgr_task(void* arg);
static void gps_mgr_rxcb(struct os_event* ev);
static bool parseNEMA(const char* line, gps_data_t* nd);

// Called from appinit 
void gps_mgr_init(const char* dname, int8_t pwrPin, int8_t uartSelect) {
    memset(&_gpsData, 0, sizeof(gps_data_t));
    _uartDevice = dname;
    _uartSelect=uartSelect;
    _pwrPin = pwrPin;
    if (_pwrPin>=0) {
        GPIO_define_out("gpspower", _pwrPin, 0, LP_DEEPSLEEP);
    }
    // mutex for data access
    os_mutex_init(&_dataMutex);

    // Create eventQ
    os_eventq_init(&_gpsMgrEQ);
    // create event with arg pointing to our line buffer
    // TODO how to tell driver limit of size of buffer???
    _myGPSEvent.ev_cb = gps_mgr_rxcb;
    _myGPSEvent.ev_arg = _rxbuf;
    // Create task 
    os_task_init(&gps_mgr_task_str, "gps_task", gps_mgr_task, NULL, GPS_TASK_PRIO,
               OS_WAIT_FOREVER, _gps_task_stack, GPS_TASK_STACK_SZ);
}

bool getGPSData(gps_data_t* d) {
    bool ret = false;
        // mutex lock
    os_mutex_pend(&_dataMutex, OS_TIMEOUT_NEVER);
    if (_gpsData.rxAt>0) {
        ret = true;
        d->lat = _gpsData.lat;
        d->lon = _gpsData.lon;
        d->alt = _gpsData.alt;
        d->prec = _gpsData.prec;
        d->nSats = _gpsData.nSats;
        d->rxAt = _gpsData.rxAt;
    }
    // ok
    os_mutex_release(&_dataMutex);

    return ret;
}
void gps_start(GPS_CB_FN_t cbfn) {
    _cbfn = cbfn;
    // Power up using power pin
    if (_pwrPin>=0) {
        GPIO_write(_pwrPin, 1);
    }
    // Select it as UART device (if required)
    if (_uartSelect>=0) {
        uart_select(_uartSelect);
    }
    // initialise comms to the gps via the uart like comms device defined in syscfg
    _cnx = wskt_open(_uartDevice, &_myGPSEvent, &_gpsMgrEQ);
    assert(_cnx!=NULL);

    // Don't need to write to anything
}
void gps_stop() {
    if (_cnx!=NULL) {
        wskt_close(&_cnx);
    }
    if (_pwrPin>=0) {
        GPIO_write(_pwrPin, 0);
    }
    _cbfn = NULL;
}

// task just runs the callbacks
static void gps_mgr_task(void* arg) {
    while(1) {
        os_eventq_run(&_gpsMgrEQ);
    }
}

// callback every time the socket gives us a new line of data from the GPS
static void gps_mgr_rxcb(struct os_event* ev) {
    // ev->arg is our line buffer
    const char* line = (char*)(ev->ev_arg);
    log_debug("received input [%s]", line);
    // parse it
    gps_data_t newdata;
    // if unparseable then tell cb
    if (!parseNEMA(line, &newdata)) {
        log_debug("bad gps line");
        if (_cbfn!=NULL) {
            (*_cbfn)(GPS_FAIL);
        }
        return;
    }

    // update current position if got one
    if (newdata.prec>0) {
        log_debug("new gps data ok %d, %d, %d", newdata.lat, newdata.lon, newdata.alt);
        bool newlock = false;
        // mutex lock
        os_mutex_pend(&_dataMutex, OS_TIMEOUT_NEVER);
        // is it a new fix?
        if (_gpsData.prec<=0) {
            newlock = true;
        }
        _gpsData.lat = newdata.lat;
        _gpsData.lon = newdata.lon;
        _gpsData.alt = newdata.alt;
        _gpsData.prec = newdata.prec;
        _gpsData.nSats = newdata.nSats;
        _gpsData.rxAt = TMMgr_getRelTime();
        // ok
        os_mutex_release(&_dataMutex);

        // tell cb
        if (_cbfn!=NULL) {
            if (newlock) {
                (*_cbfn)(GPS_LOCK);
            } else {
                (*_cbfn)(GPS_NEWFIX);
            }
        }
    } else {
        log_debug("ok gps line but not new fix");
    }
    // and continue
}

// Returns true if parsed ok, false if unparseable.
// sets the 'prec' to 0 if no location data extracted
static bool parseNEMA(const char* line, gps_data_t* nd) {
    if (!minmea_check(line, true)) {
        return false;
    }
    int8_t si = minmea_sentence_id(line, true);
    switch(si) {
        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga ggadata;
            if (minmea_parse_gga(&ggadata, line)) {
                nd->lat = ggadata.latitude.value;
                nd->lon = ggadata.longitude.value;
                nd->alt = ggadata.altitude.value;
                nd->nSats = ggadata.satellites_tracked;
                nd->prec = ggadata.fix_quality;         // TODO MAP?
            } else {
                // hmmm not so good
                nd->prec = 0;
            }
            return true;
        }
        case MINMEA_INVALID: {
            return false;
        }
        case MINMEA_SENTENCE_RMC: 
            // could be useful but GGA has more data - fall thru
        default: {
            // ignore, but not an error
            nd->prec = 0;
            return true;
        }
    }
    // never get here
}
#ifdef UNITTEST
bool unittest_gps() {
    // test NEMA parsing
    gps_data_t newdata;
    bool ret = true;        // assume all will go ok
    // Try bad nemas
    ret &= unittest("empty line", !parseNEMA("", &newdata));
    ret &= unittest("poorly formated", !parseNEMA("", &newdata));
    ret &= unittest("bad CRC", !parseNEMA("", &newdata));
    // GGA but bad
    ret &= unittest("GGA bad CRC", !parseNEMA("$GNGGA,143547.00,4511.10189,N,00542.33219,E,1,09,2.93,193.7,M,47.4,M,,*00", &newdata));
    ret &= unittest("GGA no data", parseNEMA("$GNGGA,093321.00,,,,,0,05,58.77,,,,,,*7A", &newdata));
    ret &= unittest("GGA no data result", newdata.prec==0);
    // Try good but unuseful ones - these return OK for parse but data is prec==-1
    ret &= unittest("GNGLL", parseNEMA("$GNGLL,4511.10224,N,00542.33211,E,143546.00,A,A*73", &newdata));
    ret &= unittest("GNGLL no result", newdata.prec==0);
    ret &= unittest("GNRMC", parseNEMA("$GNRMC,143547.00,A,4511.10189,N,00542.33219,E,0.140,,220318,,,A*68", &newdata));
    ret &= unittest("GNRMC no result", newdata.prec==0);
    ret &= unittest("GNGSA", parseNEMA("$GNGSA,A,3,74,75,83,65,,,,,,,,,3.73,2.93,2.30*1B", &newdata));
    ret &= unittest("GNGSA no result", newdata.prec==0);
    ret &= unittest("GPGSV", parseNEMA("$GPGSV,3,1,12,02,24,104,21,06,21,065,,12,72,031,33,14,24,313,25*7D", &newdata));
    ret &= unittest("GPGSV no result", newdata.prec==0);
    ret &= unittest("GLGSV", parseNEMA("$GLGSV,3,1,09,65,24,283,19,66,11,341,,72,12,237,,73,27,093,*67", &newdata));
    ret &= unittest("GLGSV no result", newdata.prec==0);
    // good ones - test the parse passes and the data is as expected
    ret &= unittest("GGA basic", parseNEMA("$GNGGA,143547.00,4511.10189,N,00542.33219,E,1,09,2.93,193.7,M,47.4,M,,*41", &newdata));
    ret &= unittest("GGA basic", newdata.prec>0 && newdata.lat==451110189 && newdata.lon==54233219 && newdata.alt==1937);
    return ret;
}
#endif /* UNITTEST */

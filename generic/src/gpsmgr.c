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
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/gpsmgr.h"
#include "wyres-generic/lowpowermgr.h"
#include "wyres-generic/uartselector.h"
#include "wyres-generic/timemgr.h"
#include "wyres-generic/minmea.h"
#include "wyres-generic/sm_exec.h"

// Enable/disable detailed debug log stuff
//#define DEBUG_GPS 1

//#define GPS_UART    MYNEWT_VAL(GPS_UART)
//#define GPS_TASK_PRIO       MYNEWT_VAL(GPS_TASK_PRIO)
//#define GPS_TASK_STACK_SZ   OS_STACK_ALIGN(256)
//static os_stack_t _gps_task_stack[GPS_TASK_STACK_SZ];
//static struct os_task gps_mgr_task_str;

// L96 GPS commands    
static char* EASY_ON="$PMTK869,1,1*35\r\n";
static char* HOT_START="$PMTK101*32\r\n";
static char* COLD_START="$PMTK103*30\r\n";
// Standby mode is 500uA, but can be exited by uart data... but must send it a start command?
static char* STANDBY_MODE="$PMTK161,0*28\r\n";
static char* STARTUP_RESP="$PMTK";      // Only need to check start of response

static struct appctx {
    struct os_event myGPSEvent;
//    struct os_eventq gpsMgrEQ;
    struct os_mutex dataMutex;
    SM_ID_t mySMId;
    uint32_t fixTimeoutSecs;
    const char* uartDevice;
    uint32_t baudrate;
    int8_t pwrPin;
    GPS_POWERMODE_t powerMode;
    int8_t uartSelect;
    wskt_t* cnx;
    uint8_t rxbuf[WSKT_BUF_SZ+1];
    gps_data_t gpsData;
    uint32_t lastFixTime;
    GPS_CB_FN_t cbfn;
    uint8_t commOk;     // Count of good lines received or 0 if not active
    uint8_t startupCnt; // count of times we see the gps staryup response in each session to detect brownouts
} _ctx = {
    .powerMode = POWER_ONOFF,
    .lastFixTime = 0,
    .gpsData = {
        .prec = -1,
        .lat = 0,
        .lon = 0,
        .alt = 0,
        .rxAt = 0,
    },
    .cbfn=NULL,
};


// Define my state ids
enum MyStates { MS_IDLE, MS_STARTING_COMM, MS_GETTING_FIX, MS_STOPPING_COMM, MS_LAST };
enum MyEvents { ME_START_GPS, ME_STOP_GPS, ME_UART_FAIL, ME_GPS_CONN_OK, ME_GPS_CONN_NOK, ME_GPS_FIX, ME_GPS_UART_OK, ME_GPS_UART_NOK };

// predeclare privates
//static void gps_mgr_task(void* arg);
static void gps_mgr_rxcb(struct os_event* ev);
static bool parseNEMA(const char* line, gps_data_t* nd);

static void callCB(GPS_EVENT_TYPE_t e) {
    if (_ctx.cbfn!=NULL) {
        (*_ctx.cbfn)(e);
    }
}
// State machine states
static SM_STATE_ID_t State_Idle(void* arg, int e, void* data) {
    struct appctx* ctx = (struct appctx*)arg;
    switch(e) {
        case SM_ENTER: {
            log_debug("GPS: idle");
            // ensure clean state
            ctx->commOk = 0;
            ctx->startupCnt = 0;
            if (ctx->cnx!=NULL) {
                wskt_close(&ctx->cnx);  
                ctx->cnx=NULL;    
            }
            ctx->cbfn = NULL;
            // Note : power pin off done when we exit the run only

            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case ME_START_GPS: {
            return MS_STARTING_COMM;
        }
        default: {
            log_debug("GPS:? %d in Idle", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}

static SM_STATE_ID_t State_StartingComm(void* arg, int e, void* data) {
    struct appctx* ctx = (struct appctx*)arg;

    switch(e) {
        case SM_ENTER: {
            ctx->commOk = 0;        // just starting...
            ctx->startupCnt = 0;
            // Power up using power pin
            if (ctx->pwrPin>=0) {
                log_debug("GPS: ON pin %d", ctx->pwrPin);
                // TODO add a battery check before and after power on as this could be nice to detect battery end of life
                GPIO_write(ctx->pwrPin, 0);     // yup pull down for ON
            } else {
//                log_debug("GPS: no gps power control");
            }
            // initialise comms to the gps via the uart like comms device defined in syscfg
            ctx->cnx = wskt_open(ctx->uartDevice, &ctx->myGPSEvent, os_eventq_dflt_get()); //&ctx->gpsMgrEQ);
//            assert(ctx->cnx!=NULL);
            if (ctx->cnx==NULL) {
                log_warn("GPS: Failed to get uart cnx!");
                sm_sendEvent(ctx->mySMId, ME_GPS_UART_NOK, NULL);
                return SM_STATE_CURRENT;
            }

            if (ctx->powerMode==POWER_ONSTANDBY) {
                // wake it up and help it to know how to progress
                wskt_write(ctx->cnx, (uint8_t*)EASY_ON, strlen(EASY_ON));
                if (gps_lastGPSFixAgeMins() < 0 || gps_lastGPSFixAgeMins()>3*60) {
                    wskt_write(ctx->cnx, (uint8_t*)COLD_START, strlen(COLD_START));
                } else {
                    wskt_write(ctx->cnx, (uint8_t*)HOT_START, strlen(HOT_START));
                }
            }
            // start timeout for comm check - initial timer for 5s to at least get connection up
            sm_timer_start(ctx->mySMId, 5000);

            // flush uart rather than polling...
            sm_sendEvent(ctx->mySMId, ME_GPS_UART_OK, NULL);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            // done GPS checking
            log_debug("GPS: timed out");
            // Tell cb
            callCB(GPS_COMM_FAIL);
            return MS_STOPPING_COMM;
        }
        case ME_GPS_UART_NOK: {
            log_debug("GPS: no uart");
            // Tell cb
            callCB(GPS_COMM_FAIL);
            return MS_STOPPING_COMM;
        }
        case ME_GPS_UART_OK: {
            // check if flush of previous users data done yet?
            wskt_ioctl_t cmd = {
                .cmd = IOCTL_CHECKTX,
                .param = 0,
            };
            // just check if anybody's data waiting
            if (wskt_ioctl(ctx->cnx, &cmd)!=0) {
                // todo if required we could send the event to ourselves again and busywait on the tx being done...
                log_debug("GPS: flush old tx");
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
            return SM_STATE_CURRENT;
        }

        case ME_GPS_CONN_OK: {
            log_debug("GPS: comm ok");
            // Tell cb
            callCB(GPS_COMM_OK);
            return MS_GETTING_FIX;
        }

        case ME_GPS_FIX: {
            // gps got a valid fix. go direct to running with it
            log_debug("GPS: comm ok fix");
            callCB(GPS_NEWFIX);
            return MS_GETTING_FIX;
        }
        case ME_STOP_GPS: {
            return MS_STOPPING_COMM;
        }

        default: {
            log_debug("GPS:? %d in StartingComm", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// Waiting for and processing fixes
static SM_STATE_ID_t State_GettingFix(void* arg, int e, void* data) {
    struct appctx* ctx = (struct appctx*)arg;

    switch(e) {
        case SM_ENTER: {
            if (ctx->fixTimeoutSecs>0) {
                sm_timer_start(ctx->mySMId,  ctx->fixTimeoutSecs*1000); 
            }
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            // done GPS checking
            log_debug("GPS:timed out (%d s)",ctx->fixTimeoutSecs);
            callCB(GPS_SATLOSS);
            return MS_STOPPING_COMM;
        }
        // Shouldn't happen but... treat as if we're done (and failed)
        case ME_GPS_CONN_NOK: {
            callCB(GPS_COMM_FAIL);
            return MS_STOPPING_COMM;
        }

        // Got a fix, goto state where we can read good gps data
        case ME_GPS_FIX: {
            callCB(GPS_NEWFIX);
            return SM_STATE_CURRENT;
        }
        case ME_STOP_GPS: {
            return MS_STOPPING_COMM;
        }

        default: {
            log_debug("GPS:? %d in GettingFix", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// State machine states
static SM_STATE_ID_t State_StoppingComm(void* arg, int e, void* data) {
    struct appctx* ctx = (struct appctx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Not sure this is good for it?
            if (ctx->powerMode==POWER_ONSTANDBY) {
                if (ctx->cnx!=NULL) {
                    log_debug("GPS:->standby");
                    wskt_write(ctx->cnx, (uint8_t*)STANDBY_MODE, strlen(STANDBY_MODE));
                }
            } else {
                log_debug("GPS:stopping");
            }
            // basically it gets 200ms to absorb this last command before the uart goes away
            sm_timer_start(ctx->mySMId, 200);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {                        
            if (ctx->pwrPin>=0) {
                if (ctx->powerMode==POWER_ONOFF) {
                    log_debug("GPS: OFF pin %d", ctx->pwrPin);
                    GPIO_write(ctx->pwrPin, 1);
                } else {
                    log_debug("GPS:LEFT ON pin %d",ctx->pwrPin);
                }
            }
            if (ctx->cnx!=NULL) {
                wskt_close(&ctx->cnx);      // should take effect after empty of current tx buffer...
            }
//            uart_select(-1);        // free the UART
            // tell the user we're done with uart etc
            callCB(GPS_DONE);
            ctx->cbfn = NULL;

            return MS_IDLE;
        }
        case ME_STOP_GPS: {
            // ignore we're already stopping
        }
        default: {
            log_debug("GPS:? %d in StoppingComm", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// State table : note can be in any order as the 'id' field is what maps the state id to the rest
static const SM_STATE_t _mySM[MS_LAST] = {
    {.id=MS_IDLE,           .name="Idle",       .fn=State_Idle},
    {.id=MS_STARTING_COMM,    .name="StartingComm", .fn=State_StartingComm},    
    {.id=MS_GETTING_FIX,    .name="GettingFix", .fn=State_GettingFix},    
    {.id=MS_STOPPING_COMM,    .name="StoppingComm", .fn=State_StoppingComm},    
};

// Called from appinit or app core module
void gps_mgr_init(const char* dname, uint32_t baudrate, int8_t pwrPin, int8_t uartSelect) {
    _ctx.uartDevice = dname;
    _ctx.baudrate=baudrate;
    _ctx.uartSelect=uartSelect;
    _ctx.pwrPin = pwrPin;
    if (_ctx.pwrPin>=0) {
        // Note 1 is OFF so start with it off
        GPIO_define_out("gpspower", _ctx.pwrPin, 1, LP_DEEPSLEEP);
    }
    // mutex for data access
    os_mutex_init(&_ctx.dataMutex);

    // create event with arg pointing to our line buffer
    // TODO how to tell driver limit of size of buffer???
    _ctx.myGPSEvent.ev_cb = gps_mgr_rxcb;
    _ctx.myGPSEvent.ev_arg = _ctx.rxbuf;
    // TODO Do we really need a whole task/eventQ just to rx the uart strings? as it only ever sends sm events as a result? (so callbacks run on the SM task...)
    // Nah, just use default task/default eventq
    // Create task 
//    os_task_init(&gps_mgr_task_str, "gps_task", gps_mgr_task, NULL, GPS_TASK_PRIO,
//               OS_WAIT_FOREVER, _gps_task_stack, GPS_TASK_STACK_SZ);
                   // Do gps immediately on boot
    // Create eventQ
//    os_eventq_init(&_ctx.gpsMgrEQ);
    // Start state machine
    _ctx.mySMId = sm_init("modgps", _mySM, MS_LAST, MS_IDLE, &_ctx);
    sm_start(_ctx.mySMId);

}

void gps_setPowerMode(GPS_POWERMODE_t m) {
    _ctx.powerMode = m;
}

bool gps_getData(gps_data_t* d) {
    bool ret = false;
    memset(d, 0, sizeof(gps_data_t));
        // mutex lock
    os_mutex_pend(&_ctx.dataMutex, OS_TIMEOUT_NEVER);
    if (_ctx.gpsData.rxAt>0) {
        ret = true;
        d->lat = _ctx.gpsData.lat;
        d->lon = _ctx.gpsData.lon;
        d->alt = _ctx.gpsData.alt;
        d->prec = _ctx.gpsData.prec;
        d->nSats = _ctx.gpsData.nSats;
        d->rxAt = _ctx.gpsData.rxAt;
    }
    // ok
    os_mutex_release(&_ctx.dataMutex);

    return ret;
}
// Get age of the last fix we got, or -1 if never had a fix
int32_t gps_lastGPSFixAgeMins() {
    if (_ctx.gpsData.rxAt>0) {
        return (TMMgr_getRelTimeSecs() - _ctx.gpsData.rxAt)/(60);
    } else {
        return -1;
    }
}

// TODO Should be a SM?
void gps_start(GPS_CB_FN_t cbfn, uint32_t tsecs) {
    _ctx.cbfn = cbfn;
    _ctx.fixTimeoutSecs = tsecs;
    sm_sendEvent(_ctx.mySMId, ME_START_GPS, NULL);
}
void gps_stop() {
    sm_sendEvent(_ctx.mySMId, ME_STOP_GPS, NULL);
}
/*
// task just runs the callback from UART
static void gps_mgr_task(void* arg) {
    while(1) {
        os_eventq_run(&_ctx.gpsMgrEQ);
    }
}
*/
// callback every time the socket gives us a new line of data from the GPS
static void gps_mgr_rxcb(struct os_event* ev) {
    // ev->arg is our line buffer
    const char* line = (char*)(ev->ev_arg);
    assert(line!=NULL);
    if (strnlen(line, 10)<10) {
        // too short line ignore
#ifdef DEBUG_GPS
        log_debug("GM:bad [%s]", line);
#endif /* DEBUG_GPS */
        return;
    }
    // parse it
    gps_data_t newdata;
    // if unparseable then count as bad comm credit (and if no credit left tell user)
    if (!parseNEMA(line, &newdata)) {
        if (_ctx.commOk>0) {
            _ctx.commOk--;
            if (_ctx.commOk==0) {
                sm_sendEvent(_ctx.mySMId, ME_GPS_CONN_NOK, NULL);
            }
        }
        return;
    }

    // good comm credit
    if (_ctx.commOk<5) {
        if (_ctx.commOk==0) {
            sm_sendEvent(_ctx.mySMId, ME_GPS_CONN_OK, NULL);
        }
        _ctx.commOk++;
    }
//    log_debug("gps [%s]", line);

    // update current position if got one
    if (newdata.prec>0) {
        log_debug("GPS: fix (%d, %d, %d) (%d) (%d)", newdata.lat, newdata.lon, newdata.alt, newdata.prec, newdata.nSats);
        // mutex lock
        os_mutex_pend(&_ctx.dataMutex, OS_TIMEOUT_NEVER);
        _ctx.gpsData.lat = newdata.lat;
        _ctx.gpsData.lon = newdata.lon;
        _ctx.gpsData.alt = newdata.alt;
        _ctx.gpsData.prec = newdata.prec;
        _ctx.gpsData.nSats = newdata.nSats;
        _ctx.gpsData.rxAt = TMMgr_getRelTimeSecs();
        // ok
        os_mutex_release(&_ctx.dataMutex);

        // tell sm
        sm_sendEvent(_ctx.mySMId, ME_GPS_FIX, NULL);
    } else {
        // Check for specific response strings received during startup of module 
        // [$PMTK010,001] is explicit startup message but we only check the start as any PMKT response is only at startup
        // if returned 5 times during session with no 'normal' messages, the GPS is in brownout -> fail it
        if (strncmp(line, STARTUP_RESP, 5)==0) {
            if (_ctx.startupCnt++ > 5) {
                log_warn("GPS:brownout");
                sm_sendEvent(_ctx.mySMId, ME_GPS_CONN_NOK, NULL);
            }
        }
    }
    // and done
}

// Returns true if parsed ok, false if unparseable.
// sets the 'prec' to 0 if no location data extracted
static bool parseNEMA(const char* line, gps_data_t* nd) {
    nd->prec = 0;       // default result - no new fix
    if (!minmea_check(line, true)) {
        return false;
    }
    int8_t si = minmea_sentence_id(line, true);
    switch(si) {
        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga ggadata;
            if (minmea_parse_gga(&ggadata, line)) {
                if (ggadata.fix_quality>0) {
                    nd->lat = ggadata.latitude.value;
                    nd->lon = ggadata.longitude.value;
                    nd->alt = ggadata.altitude.value;
                    nd->nSats = ggadata.satellites_tracked;
                    nd->prec = ggadata.hdop.value*2; // precision diameter for 95% is 50% *2? 
                    if (nd->prec < 1) {
                        nd->prec = 5;
                    }
                    log_debug("GPS:gga fix %d", ggadata.satellites_tracked);
                } else {
                    log_debug("GPS:gga no fix %d", ggadata.satellites_tracked);
                }
            } else {
                // hmmm not so good
#ifdef DEBUG_GPS
                log_debug("GPS:gga bad[%s]",line);
#endif
                return false;
            }
            return true;
        }
        case MINMEA_INVALID: {
#ifdef DEBUG_GPS
            log_debug("GPS:bad line [%s]", line);
#endif /* DEBUG_GPS */
            return false;
        }
        case MINMEA_SENTENCE_GLL: {
#ifdef DEBUG_GPS
            log_debug("GPS:GLL");
#endif /* DEBUG_GPS */
            return true;
        }
        case MINMEA_SENTENCE_GSA: {
#ifdef DEBUG_GPS
            log_debug("GPS:GSA");
#endif /* DEBUG_GPS */
            return true;
        }
        case MINMEA_SENTENCE_GST: {
#ifdef DEBUG_GPS
            log_debug("GPS:GST");
#endif /* DEBUG_GPS */
            return true;
        }
        case MINMEA_SENTENCE_GSV: {
#ifdef DEBUG_GPS
            struct minmea_sentence_gsv gsv;
            if (minmea_parse_gsv(&gsv, line)) {
                int goodsats=0;
                for(int i=0;i<4;i++) {
                    if (gsv.sats[i].snr>0) {
                        goodsats++;
                    }
                }
                log_debug("GPS:GSV %d,%d", gsv.total_sats, goodsats);
            } else {
                log_debug("GPS:GSV bad");
            }
#endif /* DEBUG_GPS */
            return true;
        }
        case MINMEA_SENTENCE_VTG: {
#ifdef DEBUG_GPS
            log_debug("GPS:VTG");
#endif /* DEBUG_GPS */
            return true;
        }
        case MINMEA_SENTENCE_ZDA: {
#ifdef DEBUG_GPS
            log_debug("GPS:ZDA");
#endif /* DEBUG_GPS */
            return true;
        }
        case MINMEA_SENTENCE_RMC: {
#ifdef DEBUG_GPS
            struct minmea_sentence_rmc rmcdata;
            if (minmea_parse_rmc(&rmcdata, line)) {
                if (rmcdata.valid) {
                    log_debug("GPS:rmc fix");
                } else {
                    log_debug("GPS:rmc no fix");
                }
            } else {
                log_debug("GPS:rmc nok");
            }
#endif /* DEBUG_GPS */
            // could be useful but GGA has more data - ignore
            return true;
        }
        default: {
#ifdef DEBUG_GPS
            // unknown, ignore, but not an error
            log_debug("GPS:[%d][%s]", si,  line);
//            log_debug("gps[%d][%c%c%c%c%c%c%c%c]", si, 
//                line[0],line[1],line[2],line[3],line[4],line[5],line[6],line[7]);
#endif /* DEBUG_GPS */
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

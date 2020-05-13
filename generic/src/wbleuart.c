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

/* Put the Wyres BLE module connected via UART into 'pass-thru serial' mode, and run it as a UART link.
 */

#include "os/os.h"
#include "wyres-generic/wutils.h"
#include "wyres-generic/wskt_user.h"
#include "wyres-generic/timemgr.h"
#include "wyres-generic/wblemgr.h"
#include "wyres-generic/lowpowermgr.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/uartselector.h"
#include "wyres-generic/sm_exec.h"
#include "wyres-generic/wbleuart.h"

// Enable/disable detailed debug log stuff
//#define DEBUG_BLE 1

#define WBLE_UART    MYNEWT_VAL(WBLE_UART)

static char* BLE_CHECKCONN="AT+CONN?\r\n";

//static char* BLE_ENABLE_SERIAL="AT+CONN\r\n";         we never initiate the cross-connect on BLE in case remote is using AT console of the BLE firmware!
static char* BLE_DISABLE_SERIAL="AT+DISC\r\n";  

static struct bleuartctx {
    struct os_event myUARTEvent;
    struct os_mutex dataMutex;
    SM_ID_t mySMId;
    const char* myDevice;
    const char* uartDevice;
    uint32_t baudrate;
    int8_t pwrPin;
    int8_t uartSelect;
    wskt_t* uartSkt;
    uint8_t rxbuf[WSKT_BUF_SZ+1];
    uint32_t lastDataTime;
    WBLEUART_CB_FN_t cbfn;
    uint8_t fwVersionMaj;
    uint8_t fwVersionMin;
} _ctx;     // in bss so set to all 0 by definition

// State machine for BLE control
enum BLEStates { MS_BLE_OFF, MS_BLE_WAITPOWERON, MS_BLE_STARTING, MS_BLE_SERIAL_RUNNING, MS_BLE_STOPPINGCOMM, MS_BLE_LAST };
enum BLEEvents { ME_BLE_ON, ME_BLE_OFF, ME_BLE_RET_OK, ME_BLE_RET_ERR, ME_BLE_RET_INT,
     ME_BLE_UPDATE, ME_BLE_UART_OK, ME_BLE_UART_NOK };

// predeclare privates
static void wbleuart_rxcb(struct os_event* ev);

static SM_STATE_ID_t State_Off(void* arg, int e, void* data) {
    struct bleuartctx* ctx = (struct bleuartctx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Ensure no open cnx and tuern off the power
            if (ctx->uartSkt!=NULL) {
                wskt_close(&ctx->uartSkt);      // nulls the skt id also
            }
    
            if (ctx->pwrPin>=0) {
                log_debug("BLU: OFF pin %d", ctx->pwrPin);
                // TODO add a battery check before and after power on as this could be nice to detect battery end of life
                GPIO_write(ctx->pwrPin, 1);     // yup pull UP for OFF
            } else {
                log_debug("BLU: always on?");
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
            sm_default_event_log(ctx->mySMId, "BLU", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// Wait 500ms or for a "READY" for module to get its act together
static SM_STATE_ID_t State_WaitPoweron(void* arg, int e, void* data) {
    struct bleuartctx* ctx = (struct bleuartctx*)arg;
    switch(e) {
        case SM_ENTER: {            
            // initialise comms to the ble via the uart like comms device defined in syscfg
            // This is async as we ask for exclusive access... we set a timeout of 100ms for other users data to be flushed
            ctx->uartSkt = wskt_open(ctx->uartDevice, &ctx->myUARTEvent, os_eventq_dflt_get()); // &ctx->myEQ);
//            assert(ctx->cnx!=NULL);
            if (ctx->uartSkt==NULL) {
                log_debug("BLU: Failed open uart!");
                sm_sendEvent(ctx->mySMId, ME_BLE_UART_NOK, NULL);
                return SM_STATE_CURRENT;
            }
            // Power up using power pin if required
            if (ctx->pwrPin<0) {
                log_debug("BLU: no pwr mgmt");
            } else {
                GPIO_write(ctx->pwrPin, 0);     // yup pull down for ON
                log_debug("BLU: ON pin %d", ctx->pwrPin);
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
            if (wskt_ioctl(ctx->uartSkt, &cmd)!=0) {
                log_debug("BLU: flushing old tx");
                cmd.cmd = IOCTL_FLUSHTXRX;
                cmd.param = 0;
                wskt_ioctl(ctx->uartSkt, &cmd);
            }
            // Set baud rate
            cmd.cmd = IOCTL_SET_BAUD;
            cmd.param = ctx->baudrate;
            wskt_ioctl(ctx->uartSkt, &cmd);
            // Set eol to be LF
            cmd.cmd = IOCTL_SETEOL;
            cmd.param = 0x0A;
            wskt_ioctl(ctx->uartSkt, &cmd);
            // only want ascii please
            cmd.cmd = IOCTL_FILTERASCII;
            cmd.param = 1;
            wskt_ioctl(ctx->uartSkt, &cmd);
            cmd.cmd = IOCTL_SELECTUART;
            cmd.param = ctx->uartSelect;
            wskt_ioctl(ctx->uartSkt, &cmd);
            // not going to set a type, assume its flashed as configurable scanner, go straight to starting check
            return MS_BLE_STARTING;
        }
        case ME_BLE_UART_NOK: {
            // ooops, we didnt get our exclusive access...
            log_debug("BLU: Failed uart!");
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLEUART_COMM_FAIL, NULL);
            }

            return MS_BLE_OFF;
        }
        case ME_BLE_UART_OK: {
            // init of uart cnx will be done once powerup init timeout 
            log_debug("BLU: uart ready");
            return SM_STATE_CURRENT;
        }
        case ME_BLE_RET_OK: {
            log_debug("BLU: response waiting powerup");
//            return MS_BLE_WAIT_TYPE;
            // ignore any input, wait for powerup timer as might be from a previous uart user
            return SM_STATE_CURRENT;
        }
        case ME_BLE_OFF: {
            // gave up
            return MS_BLE_OFF;
        }            
        default: {
            sm_default_event_log(ctx->mySMId, "BLU", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}

// power on, check if cross-connected comm
static SM_STATE_ID_t State_Starting(void* arg, int e, void* data) {
    struct bleuartctx* ctx = (struct bleuartctx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Send a "AT+CONN?" to check if cross connected to NUS serial BLE client already..
            // need a return of "2" to continue
            wskt_write(ctx->uartSkt, (uint8_t*)BLE_CHECKCONN, strlen(BLE_CHECKCONN));
            sm_timer_start(ctx->mySMId, 1000);      // allow 1s for response
            log_debug("BLU: check CC sent");
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            // No response to WHO, back to off
            log_warn("BLU: no CC");
            // if cb call it
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLEUART_COMM_FAIL, NULL);
            }
            return MS_BLE_OFF;
        }
        case ME_BLE_RET_OK: {       // return is just OK or READY - really should wait for proper WHO response
            log_debug("BLU: comm ok - reCC");
            wskt_write(ctx->uartSkt, (uint8_t*)BLE_CHECKCONN, strlen(BLE_CHECKCONN));
            return SM_STATE_CURRENT;
        }
        case ME_BLE_RET_INT: {        // return is an integer which tells us connection status
#ifdef DEBUG_BLE
            log_debug("BLU: con status=%d", (uint32_t)data);
#endif
            // 0=no ble nus client, 1=ble nus client but not connected to uart, 2=cross-connection so go!
            if (((uint32_t)data) == 2) {
                log_debug("BLU: cc ok");
                // if cb call it
                if (ctx->cbfn!=NULL) {
                    (*ctx->cbfn)(WBLEUART_COMM_OK, NULL);
                }
                // Go and enable serial connections from remote people
                return MS_BLE_SERIAL_RUNNING;
            } else {
                log_debug("BLU: no cc (%d)", (uint32_t)data);
                // if cb call it
                if (ctx->cbfn!=NULL) {
                    (*ctx->cbfn)(WBLEUART_COMM_FAIL, NULL);
                }
                return MS_BLE_OFF;
            }
        }
        case ME_BLE_OFF: {
            // gave up - directly off
            return MS_BLE_OFF;
        }            
        default: {
            sm_default_event_log(ctx->mySMId, "BLU", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}


static SM_STATE_ID_t State_SerialRunning(void* arg, int e, void* data) {
    struct bleuartctx* ctx = (struct bleuartctx*)arg;
    switch(e) {
        case SM_ENTER: {
            log_info("BLU:serialing");
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            log_info("BLU:end serial");
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            return SM_STATE_CURRENT;
        }
        // go direct to off if requested
        case ME_BLE_OFF: {
            return MS_BLE_STOPPINGCOMM;
        }            

        case ME_BLE_RET_OK: {
            // ignore any non-ib return data
            return SM_STATE_CURRENT;
        }
        case ME_BLE_RET_ERR: {
            // TODO what could cause this?
            return SM_STATE_CURRENT;
        }

        default: {
            sm_default_event_log(ctx->mySMId, "BLU", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}

// delay before closing connection for any last requests
static SM_STATE_ID_t State_StoppingComm(void* arg, int e, void* data) {
    struct bleuartctx* ctx = (struct bleuartctx*)arg;
    switch(e) {
        case SM_ENTER: {
            wskt_write(ctx->uartSkt, (uint8_t*)BLE_DISABLE_SERIAL, strlen(BLE_DISABLE_SERIAL));
            log_debug("BLU: set connnections serial as off");
            sm_timer_start(ctx->mySMId, 500);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            if (ctx->uartSkt!=NULL) {
                wskt_close(&ctx->uartSkt);  // sets uartSkt to NULL
            }
            return MS_BLE_OFF;
        }
        case ME_BLE_OFF: {
            // gave up
            if (ctx->uartSkt!=NULL) {
                wskt_close(&ctx->uartSkt);  // sets uartSkt to NULL
            }
            return MS_BLE_OFF;
        }            
        case ME_BLE_ON: {
            // comm ok already
            if (ctx->cbfn!=NULL) {
                (*ctx->cbfn)(WBLEUART_COMM_OK, NULL);
            }
            return MS_BLE_SERIAL_RUNNING;
        }
        default: {
            sm_default_event_log(ctx->mySMId, "BLU", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// State table : note can be in any order as the 'id' field is what maps the state id to the rest
static const SM_STATE_t _bleSM[MS_BLE_LAST] = {
    {.id=MS_BLE_OFF,        .name="BluOff",       .fn=State_Off},
    {.id=MS_BLE_WAITPOWERON,.name="BluWaitPower", .fn=State_WaitPoweron},
    {.id=MS_BLE_STARTING,   .name="BluStarting",  .fn=State_Starting},    
    {.id=MS_BLE_SERIAL_RUNNING,   .name="BluSerialRunning", .fn=State_SerialRunning},    
    {.id=MS_BLE_STOPPINGCOMM, .name="BluStopping", .fn=State_StoppingComm},    
};

/* The external API for this code is via the wskt system ie this code emulates a line based system
 * Create 'bleuart' device with given name, and the underlying mynewt device to open to talk to the BLE, 
 * at given baud rate, with hardwar uart select/power management
 * This case is only iff no other ble mgr is also using it at the same time...
 */
void* wbleuart_create(const char* dname, uint32_t baudrate, int8_t pwrPin, int8_t uartSelect) {
    // Ignore multiple inits as this code can't handle them....
    // TODO if required to support multiple BLEs on multiple UARTs....
    if (_ctx.uartDevice!=NULL) {
        if (strcmp(_ctx.uartDevice, dname)==0) {
            // already inited on same device... not an issue
            log_debug("BLU: device %s already inited", dname);
            return &_ctx;
        } else {
            log_debug("BLU: FAIL init %s but already on %s", dname, _ctx.uartDevice);
            assert(0);
        }
    }
    _ctx.uartDevice = dname;
    _ctx.baudrate = baudrate;
    _ctx.uartSelect=uartSelect;
    _ctx.pwrPin = pwrPin;
    if (_ctx.pwrPin>=0) {
        // Note 1 is OFF so start with it off
        GPIO_define_out("blepower", _ctx.pwrPin, 1, LP_DEEPSLEEP, PULL_UP);
    }
    // create event with arg pointing to our line buffer
    // TODO how to tell driver limit of size of buffer???
    _ctx.myUARTEvent.ev_cb = wbleuart_rxcb;
    _ctx.myUARTEvent.ev_arg = _ctx.rxbuf;
    // Create SM
    _ctx.mySMId = sm_init("bleuart", _bleSM, MS_BLE_LAST, MS_BLE_OFF, &_ctx);
    sm_start(_ctx.mySMId);

    return &_ctx;
}

void wbleuart_line_open(void* c, WBLEUART_CB_FN_t cb) {
    assert(c!=NULL);
    struct bleuartctx* ctx = (struct bleuartctx*)c;
    ctx->cbfn = cb;
    // Ask SM to run (if it is aleady this will be ignored)
    sm_sendEvent(ctx->mySMId, ME_BLE_ON, NULL);
    return;
}

int wbleuart_line_write(void* c, uint8_t* data, uint32_t sz) {
    assert(c!=NULL);
    struct bleuartctx* ctx = (struct bleuartctx*)c;

    if (ctx->uartSkt==NULL) {
        log_warn("BLU:can't write as no uart dev..");
        return SKT_NODEV;
    }
    // Write thru directly if in correct state (note can't pass to SM 'cleanly' as would need to copy buffer...)
    wskt_write(ctx->uartSkt, data, sz);
    return SKT_NOERR; 
}

void wbleuart_line_close(void* c) {
    assert(c!=NULL);
    struct bleuartctx* ctx = (struct bleuartctx*)c;

    sm_sendEvent(ctx->mySMId, ME_BLE_OFF, NULL);
    // leave any buffers to be tx'd in their own time
    return;
}

// callback every time the socket gives us a new line of data 
// Guarenteed to be mono-thread
// Can't process in state machine as can't copy the data buffer... soz
static void wbleuart_rxcb(struct os_event* ev) {
    // ev->arg is our line buffer
    char* line = (char*)(ev->ev_arg);
    assert(line!=NULL);

    // pass up to our users
    if (_ctx.cbfn!=NULL) {
        (*_ctx.cbfn)(WBLEUART_RX, (void*)line);
    }
}

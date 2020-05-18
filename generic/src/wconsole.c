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

// Basic AT commands based console. Can use any UART device, and also be aware of a 'uart selector' hardware
// Console is activate/deactivate on demand by the application layer.
// Uses the default task/eventq for execution of its rx'd commands
#include "os/os.h"
#include "wyres-generic/wutils.h"
#include "wyres-generic/wskt_user.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/wconsole.h"
#include "wyres-generic/lowpowermgr.h"
#include "wyres-generic/uartselector.h"
#include "wyres-generic/timemgr.h"
#include "wyres-generic/sm_exec.h"
#include "wyres-generic/rebootmgr.h"

#define MAX_TXSZ (256)

static struct appctx {
    struct os_event myUARTEvent;
    SM_ID_t mySMId;
    uint32_t lastInputTS;       // secs since boot
    uint32_t idleTimeoutS;
    const char* uartDevice;
    int8_t uartSelect;
    uint32_t baudrate;
    wskt_t* cnx;
    uint8_t rxbuf[WSKT_BUF_SZ+1];
    uint8_t txbuf[MAX_TXSZ+3];
    uint8_t ncmds;
    ATCMD_DEF_t* cmds;
} _ctx = {
    .mySMId=NULL,
    .ncmds=0,
    .idleTimeoutS=0,
};

// prompts
//static char* HELLO="WCONSOLE : Hello.\r\n";
static char* STARTPROMPT="\r\n> ";       // tell user we're here, waiting for input
static char* PROMPT="\r\n> ";
static char* BYEBYE="!\r\n";        // not any more

// Define my state ids
enum MyStates { MS_IDLE, MS_STARTING_COMM, MS_ACTIVE, MS_STOPPING_COMM, MS_LAST };
enum MyEvents { ME_CHECK_CONSOLE, ME_START_CONSOLE, ME_STOP_CONSOLE, ME_NEW_DATA, ME_CN_UART_OK, ME_CN_UART_NOK };

// predeclare privates
static void uart_mgr_rxcb(struct os_event* ev);
static void processATCmd(char* line);


// State machine states
static SM_STATE_ID_t State_Idle(void* arg, int e, void* data) {
    struct appctx* ctx = (struct appctx*)arg;
    switch(e) {
        case SM_ENTER: {
            log_debug("CN: idle");
            // ensure clean state
            if (ctx->cnx!=NULL) {
                wskt_close(&ctx->cnx);  // sets cnx to null
            }
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case ME_START_CONSOLE: {
            return MS_STARTING_COMM;
        }
        default: {
            sm_default_event_log(ctx->mySMId, "CN", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}

static SM_STATE_ID_t State_StartingComm(void* arg, int e, void* data) {
    struct appctx* ctx = (struct appctx*)arg;

    switch(e) {
        case SM_ENTER: {
            // initialise comms to the gps via the uart like comms device defined in syscfg
            ctx->cnx = wskt_open(ctx->uartDevice, &ctx->myUARTEvent, os_eventq_dflt_get());
            if (ctx->cnx==NULL) {
                log_debug("CN: Failed to open uart[%s]!", ctx->uartDevice);
                sm_sendEvent(ctx->mySMId, ME_CN_UART_NOK, NULL);
                return SM_STATE_CURRENT;
            }
            // check if uart tx in progress
            sm_sendEvent(ctx->mySMId, ME_CN_UART_OK, NULL);

            return SM_STATE_CURRENT;
        }

        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case ME_CN_UART_NOK: {
            // ooops, we didnt get our exclusive access...
            log_debug("CN: uart[%s] access failed", ctx->uartDevice);
            return MS_IDLE;
        }
        case ME_CN_UART_OK: {
            // check if flush of previous users data done yet?
            wskt_ioctl_t cmd = {
                .cmd = IOCTL_CHECKTX,
                .param = 0,
            };
            // just check if anybody's data waiting
            if (wskt_ioctl(ctx->cnx, &cmd)!=0) {
                // todo if required we could send the event to ourselves again and busywait on the tx being done...
                log_debug("CN: flushing old tx");
                cmd.cmd = IOCTL_FLUSHTXRX;
                cmd.param = 0;
                wskt_ioctl(ctx->cnx, &cmd);
            }
            // Set baud rate
            cmd.cmd = IOCTL_SET_BAUD;
            cmd.param = ctx->baudrate;
            wskt_ioctl(ctx->cnx, &cmd);
            // Set eol to be CR
            cmd.cmd = IOCTL_SETEOL;
            cmd.param = 0x0D;
            wskt_ioctl(ctx->cnx, &cmd);
            // only want ascii please
            cmd.cmd = IOCTL_FILTERASCII;
            cmd.param = 1;
            wskt_ioctl(ctx->cnx, &cmd);
            cmd.cmd = IOCTL_SELECTUART;
            cmd.param = ctx->uartSelect;
            wskt_ioctl(ctx->cnx, &cmd);
            log_debug("CN: uart ready");
            // Say hi
//            wskt_write(ctx->cnx, (uint8_t*)HELLO, strlen(HELLO));
            wskt_write(ctx->cnx, (uint8_t*)STARTPROMPT, strlen(STARTPROMPT));
            return SM_STATE_CURRENT;
        }
        case ME_NEW_DATA: {
            log_debug("CN: comm ok");
            wskt_write(ctx->cnx, (uint8_t*)PROMPT, strlen(PROMPT));            
            ctx->lastInputTS = TMMgr_getRelTimeSecs();
            return MS_ACTIVE;
        }

        case ME_STOP_CONSOLE: { 
            // sync stop
            return MS_IDLE;
        }

        default: {
            sm_default_event_log(ctx->mySMId, "CN", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// Waiting for and processing commands
static SM_STATE_ID_t State_Active(void* arg, int e, void* data) {
    struct appctx* ctx = (struct appctx*)arg;

    switch(e) {
        case SM_ENTER: {
            // the console driving code does timeout to release UART iff asked to by the higher level
            // Checks every 60s if enabled
            if (ctx->idleTimeoutS>0) {
                sm_timer_start(ctx->mySMId, 60000);
            }
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            if (ctx->idleTimeoutS>0) {
                // Every X we check if had an input in the last 60s, if not then exit at mode
                if ((TMMgr_getRelTimeSecs() - ctx->lastInputTS) > (ctx->idleTimeoutS)) {
                    return MS_STOPPING_COMM;
                }
                // set for next check
                sm_timer_start(ctx->mySMId, 60000);
            }
            return SM_STATE_CURRENT;
        }

        case ME_STOP_CONSOLE: {
            // this is synchronous for this use as shutdown is done in the stop call...
            return MS_IDLE;
        }
        case ME_NEW_DATA: {
            wskt_write(ctx->cnx, (uint8_t*)PROMPT, strlen(PROMPT));
            ctx->lastInputTS = TMMgr_getRelTimeSecs();
            return SM_STATE_CURRENT;
        }

        default: {
            sm_default_event_log(ctx->mySMId, "CN", e);
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
            log_debug("CN:stopping");
            wskt_write(ctx->cnx, (uint8_t*)BYEBYE, strlen(BYEBYE));
            // basically it gets 200ms to absorb this last command before the uart goes away
            sm_timer_start(ctx->mySMId, 200);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {                        
            if (ctx->cnx!=NULL) {
                wskt_close(&ctx->cnx);      // should take effect after empty of current tx buffer...
            }
            return MS_IDLE;
        }
        case ME_STOP_CONSOLE: {
            // ignore we're already stopping
        }
        default: {
            sm_default_event_log(ctx->mySMId, "CN", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// State table : note can be in any order as the 'id' field is what maps the state id to the rest
static SM_STATE_t _mySM[MS_LAST] = {
    {.id=MS_IDLE,           .name="Idle",       .fn=State_Idle},
    {.id=MS_STARTING_COMM,    .name="StartingComm", .fn=State_StartingComm},    
    {.id=MS_ACTIVE,    .name="Active", .fn=State_Active},    
    {.id=MS_STOPPING_COMM,    .name="StoppingComm", .fn=State_StoppingComm},    
};

// Called from appinit or app core module
void wconsole_mgr_init(const char* dname, uint32_t baudrate, int8_t uartSelect) {
    _ctx.uartDevice = dname;
    _ctx.uartSelect=uartSelect;
    _ctx.baudrate = baudrate;
    // create event with arg pointing to our line buffer
    // TODO how to tell driver limit of size of buffer???
    _ctx.myUARTEvent.ev_cb = uart_mgr_rxcb;
    _ctx.myUARTEvent.ev_arg = _ctx.rxbuf;
    // Start state machine
    _ctx.mySMId = sm_init("wconsole", _mySM, MS_LAST, MS_IDLE, &_ctx);
    sm_start(_ctx.mySMId);

}

void wconsole_start(uint8_t ncmds, ATCMD_DEF_t* cmds, uint32_t idleTimeoutS) {
    // Ignore if not initialised
    if (_ctx.mySMId!=NULL) {
        _ctx.ncmds = ncmds;
        _ctx.cmds = cmds;
        _ctx.idleTimeoutS = idleTimeoutS;
        sm_sendEvent(_ctx.mySMId, ME_START_CONSOLE, NULL);
    }
}
// synchronous halt of the console and free its use of the uart
void wconsole_stop() {
    // Ignore if not initialised
    if (_ctx.mySMId!=NULL) {
        if (_ctx.cnx!=NULL) {
            wskt_write(_ctx.cnx, (uint8_t*)BYEBYE, strlen(BYEBYE));
            wskt_close(&_ctx.cnx);      // should take effect after empty of current tx buffer...
        }
        // Tell SM
        sm_sendEvent(_ctx.mySMId, ME_STOP_CONSOLE, NULL);
    }
}
bool wconsole_isInit() {
    return (_ctx.mySMId!=NULL);
}
bool wconsole_isActive() {
    // Ignore if not initialised
    if (_ctx.mySMId!=NULL) {
        // Console can only be active if its in the active state
        if (sm_getCurrentState(_ctx.mySMId)==MS_ACTIVE) {
            // If got input in last 30s we count it as 'in use'
            if ((TMMgr_getRelTimeSecs() - _ctx.lastInputTS) < 30) {
                return true;
            }
            // else its not active really (ie caller could do something else with uart - but must stop/start the console in this case)
        }
    }
    return false;
}

static bool wconsole_println(const char* l, ...) {
    bool ret = true;
    va_list vl;
    va_start(vl, l);
    vsprintf((char*)&_ctx.txbuf[0], l, vl);
    int len = strnlen((const char*)&_ctx.txbuf[0], MAX_TXSZ);
    if (len>=MAX_TXSZ) {
        // oops might just have broken stuff...
        len = MAX_TXSZ-1;
        ret = false;        // caller knows there was an issue
    }
    _ctx.txbuf[len]='\n';
    _ctx.txbuf[len+1]='\r';         // This is to play nice with putty...
    _ctx.txbuf[len+2]=0;
    len+=2;     // Don't send the null byte!
    if (_ctx.cnx!=NULL) {
        int res = wskt_write(_ctx.cnx, &_ctx.txbuf[0], len);
        if (res<0) {
            _ctx.txbuf[0] = '*';
            wskt_write(_ctx.cnx, &_ctx.txbuf[0], 1);      // so user knows he missed something.
            ret = false;        // caller knows there was an issue
            // Not actually a lot we can do about this especially if its a flow control (SKT_NOSPACE) condition - ignore it
           log_noout_fn("console write FAIL %d", res);      // just for debugger to watch
       }
    } else {
        ret = false;        // caller knows there was an issue
        log_noout_fn("console write no open uart");      // just for debugger to watch
    }
    va_end(vl);
    return ret;
}
// callback every time the socket gives us a new line of data 
static void uart_mgr_rxcb(struct os_event* ev) {
    // ev->arg is our line buffer
    char* line = (char*)(ev->ev_arg);
    assert(line!=NULL);
    // process it for commands
    processATCmd(line);
    // tell SM so it presents new prompt
    sm_sendEvent(_ctx.mySMId, ME_NEW_DATA, NULL);
    // and done
}

static void processATCmd(char* line) {
    // parse line into : command, args
    char* els[5];
    char* s = line;
    int elsi = 0;
    // first segment
    els[elsi++] = s;
    while (*s!=0 && elsi<5) {
        if (*s==' ' || *s=='=' || *s==',') {
            // make end of string at seperator
            *s='\0';
            s++;
            // consume blank space (but not seperators)
            while(*s==' ') {
                s++;
            }
            // If more string, then its the next element..
            if (*s!='\0') {
                els[elsi++] = s;
            } // else leave it on the end of string as we're done
        } else {
            s++;
        }
    }
    // check we are not talking to another console (return of OK or ERROR)
    if (strncmp("OK", els[0], 2)==0 || strncmp("ERROR", els[0], 5)==0) {
        // give up immediately
        log_debug("detected remote is console[%s], stopping", els[0]);
        wconsole_stop();
        return;
    }
    // find it in the list
    for(int i=0;i<_ctx.ncmds;i++) {
        if (strcmp(els[0], _ctx.cmds[i].cmd)==0) {
            // gotcha
            log_debug("got cmd %s with %d args", els[0], elsi-1);
            // call the specific command processor function as registered
            ATRESULT res = (*_ctx.cmds[i].fn)(&wconsole_println, elsi, els);
            switch(res) {
                case ATCMD_OK: {
                    wconsole_println("OK\r\n");
                    break;
                }
                case ATCMD_GENERR: {
                    wconsole_println("ERROR\r\n");
                    break;
                }
                default:
                    // Command processing already did return
                    break;
            }
            return;
        }
    }
    // not found
    wconsole_println("ERROR\r\n");
    wconsole_println("Unknown command [%s].", els[0]);
    log_debug("no cmd %s with %d args", els[0], elsi-1);
}

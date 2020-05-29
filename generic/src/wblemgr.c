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
#include "wyres-generic/timemgr.h"
#include "wyres-generic/wblemgr.h"
#include "wyres-generic/lowpowermgr.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/uartselector.h"
#include "wyres-generic/sm_exec.h"

// Enable/disable detailed debug log stuff
//#define DEBUG_BLE 1

#define WBLE_UART    MYNEWT_VAL(WBLE_UART)
//#define WBLE_TASK_PRIO       MYNEWT_VAL(WBLE_TASK_PRIO)
//#define WBLE_TASK_STACK_SZ   OS_STACK_ALIGN(256)
//#define MAX_IBEACONS    MYNEWT_VAL(WBLE_MAXIBS)

#define UART_ENABLE_TIMEMS (100)
#define UART_CMD_RETRY_TIMEMS (200)

static char* BLE_PAD="\n\n\n\n";        // to sync uart after wakeup

static char* BLE_CHECK="AT+WHO\r\n";
// Latest version of BLE scanner using push mode works best for large numbers of BLE devices scanned...
static char* BLE_RXMODE="AT+PUSH\r\n";

//static char BLE_SCAN_START[50]; //AT+START,UUID\r\n\0, need to be sufficient size   - uses txLine in ctx
static char* BLE_SCAN_STOP="AT+STOP\r\n";

static char* BLE_IB_STOP="AT+IB_STOP\r\n";

static char* BLE_CHECKCONN="AT+CONN?\r\n";

//static char* BLE_ENABLE_SERIAL="AT+CONN\r\n";         we never initiate the cross-connect on BLE in case remote is using AT console of the BLE firmware!
static char* BLE_DISABLE_SERIAL="AT+DISC\r\n";  


// the 'standard' wyres UUID for ibeacons is "E2C56DB5-DFFB-48D2-B060-D0F5A71096E0" (actually its that used by Minew...)
static uint8_t WYRES_UUID[] = { 0xE2,0xC5,0x6D,0xB5,0xDF,0xFB,0x48,0xD2,0xB0,0x60,0xD0,0xF5,0xA7,0x10,0x96,0xE0 };

#define UUID_SZ (16)

static struct blectx {
    struct os_event myUARTEvent;
    struct os_mutex dataMutex;
    SM_ID_t mySMId;
    const char* uartDevice;
    uint32_t baudrate;
    int8_t pwrPin;
    int8_t uartPin;
    int8_t uartSelect;
    wskt_t* cnx;
    uint8_t rxbuf[WSKT_BUF_SZ+1];
    char txLine[100];                // For dynamic string creation to send to BLE
    uint32_t lastDataTime;
    WBLE_CB_FN_t cbfn;
    uint8_t uuid[16];
    uint8_t fwVersionMaj;
    uint8_t fwVersionMin;
    uint16_t ibMajor;
    uint16_t ibMinor;
    uint8_t ibExtra;
    uint16_t ibInterMS;
    int8_t ibTxPower;
    uint16_t majorStart;
    uint16_t majorEnd;
    uint8_t ibListSz;
    ibeacon_data_t* ibList;
    uint8_t nbRxNew;
    uint8_t nbRxNewR;
    uint8_t nbRxUpdate;
    uint8_t nbRxNoSpace;
    uint8_t nbRxBadMajor;
} _ctx;     // in bss so set to all 0 by definition

// State machine for BLE control
enum BLEStates { MS_BLE_OFF, MS_BLE_WAITPOWERON, MS_BLE_STARTING, MS_BLE_ON, MS_BLE_SCANNING, MS_BLE_UART_CHECK, MS_BLE_UART_RUNNING, MS_BLE_START_IB, 
    MS_BLE_STOPPINGCOMM, MS_BLE_LAST };
enum BLEEvents { ME_BLE_ON, ME_BLE_OFF, ME_BLE_START_SCAN, ME_BLE_START_IB, ME_BLE_STOP_SCAN, ME_BLE_STOP_IB, ME_BLE_RET_OK, ME_BLE_RET_ERR, ME_BLE_RET_INT,
     ME_BLE_UPDATE, ME_BLE_UART_OK, ME_BLE_UART_NOK, ME_BLE_UART_CONN, ME_BLE_UART_DISC, ME_CC_RETRY };

// predeclare privates
//static void wble_mgr_task(void* arg);
static void wble_mgr_rxcb(struct os_event* ev);
static ibeacon_data_t*  getIB(int idx);
// Add scanned IB to list if not already present else update it
static int addIB(ibeacon_data_t* ibp);
// Send ibeaconning on command
static void sendIBStart(struct blectx* ctx);
// Send ibeaconning off command
static void sendIBStop(struct blectx* ctx);

static void callCB(struct blectx* ctx, WBLE_EVENT_t e, void* d) {
    if (ctx->cbfn!=NULL) {
        (*ctx->cbfn)(e, d);
    }
}

static bool configComm(struct blectx* ctx) {
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
    return true;
}

static int powerRequest(struct blectx* ctx, bool state) {
    // Power up/down using power pin if required
    if (ctx->pwrPin<0) {
        log_debug("BLE: no power control");
        return 50;      // power up delay is minimal
    } else {
        GPIO_write(ctx->pwrPin, state?0:1);     // yup pull down for ON, up for OFF
        log_debug("BLE: power [%d]:%s", ctx->pwrPin, state?"ON":"OFF");
        return 500;     // pifometric time it takes the ble module to be ready after a power up
    }
}
static void uartRequest(struct blectx* ctx, bool state) {
    if (state) {
        // UART on, make sure our cnx is open
        if (ctx->cnx==NULL) {
            ctx->cnx = wskt_open(ctx->uartDevice, &ctx->myUARTEvent, os_eventq_dflt_get()); // &ctx->myEQ);
//            assert(ctx->cnx!=NULL);
            if (ctx->cnx==NULL) {
                log_debug("BLE: Failed open uart!");
                sm_sendEvent(ctx->mySMId, ME_BLE_UART_NOK, NULL);        
                return;
            }
            configComm(ctx);
        }
    } else {
        // UART off, close connection to save power
        if (ctx->cnx!=NULL) {
            wskt_close(&ctx->cnx);      // nulls our cnx id also
        }
    }
    // ask ble to turn its uart on via io pin if required
    if (ctx->uartPin<0) {
        log_debug("BLE: no uart control");
    } else {
        GPIO_write(ctx->uartPin, state?0:1);     // yup pull down for active, up for inactive (as the base card inverts the signal)
        log_debug("BLE: uart [%d]:%s", ctx->uartPin, state?"ON":"OFF");
    }
}

static SM_STATE_ID_t State_Off(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Ensure no open cnx and tuern off the power/disbale remote uart
/*
            if (ctx->cnx!=NULL) {
                wskt_close(&ctx->cnx);      // nulls our cnx id also
            }
*/
            powerRequest(ctx, false);
            uartRequest(ctx, false);         // closes our uart cnx and (if required) tells ble card to deinit its uart too
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
            sm_default_event_log(ctx->mySMId, "BLE", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// Wait 500ms for module to get its act together
static SM_STATE_ID_t State_WaitPoweron(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {            
            // initialise comms to the ble via the uart like comms device defined in syscfg
            // This is async as we ask for exclusive access... we set a timeout of 100ms for other users data to be flushed
            int pwrDelay = powerRequest(ctx, true);     // power up BLE if needed, returns required dely

            // And set the timer for the powerup time to finish 
            sm_timer_start(ctx->mySMId, pwrDelay);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            // Should be ready to roll
            return MS_BLE_STARTING;
        }
        case ME_BLE_UART_NOK: {
            // shouldn't happen as uart not open
            log_debug("BLE: poweron says failed uart!");
            callCB(ctx, WBLE_COMM_FAIL, NULL);

            return MS_BLE_OFF;
        }
        case ME_BLE_UART_OK: {
            // shouldn't happen as uart not open
            // init of uart cnx will be done once powerup init timeout in next state (Starting)
            log_debug("BLE: poweron says uart ready");
            return SM_STATE_CURRENT;
        }
        case ME_BLE_RET_OK: {
            // shouldn't happen as uart not open
            log_debug("BLE: poweron says response waiting powerup");
            // ignore any input, wait for powerup timer as might be from a previous uart user
            return SM_STATE_CURRENT;
        }
        case ME_BLE_OFF: {
            // gave up
            return MS_BLE_OFF;
        }            
        default: {
            sm_default_event_log(ctx->mySMId, "BLE", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}

// power on, open uart and send WHO to check comm ok
static SM_STATE_ID_t State_Starting(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            uartRequest(ctx, true);      // request uart comm to BLE and open connection

            sm_timer_startE(ctx->mySMId, UART_ENABLE_TIMEMS, ME_CC_RETRY);      // give it some time to start its uart on remote end

            sm_timer_start(ctx->mySMId, 1000);      // allow 1s for response overall
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            sm_timer_stopE(ctx->mySMId, ME_CC_RETRY);       // As not stoppped automatically by change of state
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            // No response to WHO, back to off
            log_warn("BLE: no who");
            // if cb call it
            callCB(ctx, WBLE_COMM_FAIL, NULL);
            return MS_BLE_OFF;
        }

        case ME_CC_RETRY: {
            // Send who command to ensure comm ok
            log_debug("BLE: who-?");
            wskt_write(ctx->cnx, (uint8_t*)BLE_CHECK, strlen(BLE_CHECK));
            return SM_STATE_CURRENT;
        }

        case ME_BLE_RET_ERR: {
            log_debug("BLE: who-X");
            sm_timer_startE(ctx->mySMId, UART_CMD_RETRY_TIMEMS, ME_CC_RETRY);      // give it some space and retry
            return SM_STATE_CURRENT;
        }

        case ME_BLE_RET_INT: {        // return is an integer which is what we expect from WHO
            // Normally the who response is the data value. Store it for later
            // from BLEV2 it is the firmware version stored in a uint16 (MSB=major, LSB=minor)
            ctx->fwVersionMaj = ((((uint32_t)data) >>8) & 0xFF);
            ctx->fwVersionMin = (((uint32_t)data) & 0xFF);
            if (ctx->fwVersionMaj < 2) {
                // bad ble version
                log_warn("BLE : module fw v%d.%d not > 2.0: fail", ctx->fwVersionMaj, ctx->fwVersionMin);
                callCB(ctx, WBLE_COMM_FAIL, NULL);
                return MS_BLE_OFF;
            } else {
                log_info("BLE: fw v%d.%d", ctx->fwVersionMaj, ctx->fwVersionMin);
            }
            // if cb call it
            callCB(ctx, WBLE_COMM_OK, NULL);
            return MS_BLE_ON;
        }
        case ME_BLE_UART_CONN: {
            // Not allowed
            callCB(ctx, WBLE_UART_DISC, NULL);
            return SM_STATE_CURRENT;
        }
        case ME_BLE_OFF: {
            // gave up - directly off
            return MS_BLE_OFF;
        }            
        default: {
            sm_default_event_log(ctx->mySMId, "BLE", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
static SM_STATE_ID_t State_On(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            sm_timer_start(ctx->mySMId, 500);   // Set timer to disable uart in a mo (let previous state's uart tx's drain first)
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            uartRequest(ctx, false);      // BLE can deinit its uart as we're not actively talking to it (required for low power!)
            return SM_STATE_CURRENT;
        }
        case ME_BLE_ON: {
            // We _are_ on - but we'll check with the BLE anyway before saying so
            return MS_BLE_STARTING;
        }
        case ME_BLE_OFF: {
            // Nothing happening, go off
            return MS_BLE_OFF;
        }            
        case ME_BLE_START_SCAN: {
            // Start scanning - always allowed
            return MS_BLE_SCANNING;
        }
        case ME_BLE_UART_CONN: {
            // Check if BLE is in the right state to be connected for remote uart 
            return MS_BLE_UART_CHECK;
        }
        case ME_BLE_UART_DISC: {
            // ok, whatever
            return SM_STATE_CURRENT;
        }
        case ME_BLE_START_IB: {
            log_info("BLE:ib go");
            return MS_BLE_START_IB;
        }            
        case ME_BLE_STOP_IB: {
            // beaconning is not a state so can just request it any time
            log_info("BLE:ib end");
            // This is not optimum... TODO
            uartRequest(ctx, true);
            wskt_write(ctx->cnx, (uint8_t*)BLE_PAD, strlen(BLE_PAD));       // sync uart
            sendIBStop(ctx);
            uartRequest(ctx, false);
            return SM_STATE_CURRENT;
        }            
        default: {
            sm_default_event_log(ctx->mySMId, "BLE", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}

static SM_STATE_ID_t State_Scanning(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            uartRequest(ctx, true);      // request uart comm to BLE
            // And wait a little to ensure other end is listening
            sm_timer_startE(ctx->mySMId, UART_ENABLE_TIMEMS, ME_CC_RETRY);
            // remind every second we're in push
            sm_timer_start(ctx->mySMId, 1000);
            log_info("BLE:scanning[%d->%d]", ctx->majorStart, ctx->majorEnd);
            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            sm_timer_stopE(ctx->mySMId, ME_CC_RETRY);       // As not stoppped automatically by change of state
            // Stop scanner
            wskt_write(ctx->cnx, (uint8_t*)BLE_SCAN_STOP, strlen(BLE_SCAN_STOP));
            log_info("BLE:end scan %d %d %d %d %d", ctx->nbRxNew, ctx->nbRxNewR, ctx->nbRxUpdate, ctx->nbRxNoSpace, ctx->nbRxBadMajor);
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            wskt_write(ctx->cnx, (uint8_t*)BLE_RXMODE, strlen(BLE_RXMODE));
            // Every second, just remind we're in push mode
            sm_timer_start(ctx->mySMId, 1000);
            return SM_STATE_CURRENT;
        }
        
        case ME_CC_RETRY: {
            // (re)try scan command (assuming noone used txLine elsewhere!)
            // rx data mode is PUSH
            wskt_write(ctx->cnx, (uint8_t*)BLE_RXMODE, strlen(BLE_RXMODE));
            // And start the scanning - create the start command dynamically to include UUID
            if (Util_notAll0(ctx->uuid,UUID_SZ)==true) {
                sprintf(ctx->txLine,"AT+START,%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\r\n",
                    ctx->uuid[0],ctx->uuid[1],ctx->uuid[2],ctx->uuid[3],ctx->uuid[4],ctx->uuid[5],ctx->uuid[6],ctx->uuid[7],
                    ctx->uuid[8],ctx->uuid[9],ctx->uuid[10],ctx->uuid[11],ctx->uuid[12],ctx->uuid[13],ctx->uuid[14],ctx->uuid[15]);    
            } else {
                sprintf(ctx->txLine,"AT+START\r\n");
            }
            log_debug(ctx->txLine); 
            wskt_write(ctx->cnx, (uint8_t*)(&ctx->txLine[0]), strlen(ctx->txLine));

            return SM_STATE_CURRENT;
        }

        // go to off (via exit and delay to get the STOP to the ble)
        case ME_BLE_OFF: {
            return MS_BLE_STOPPINGCOMM;
        }       
        case ME_BLE_ON: {
            // We are on, tell caller
            callCB(ctx, WBLE_COMM_OK, NULL);
            // caller is a little confused, make sure we are in the 'ON' state ready for their next move
            // Back to idle but on mode (exit does the stop)
            return MS_BLE_ON;
        }
     
        case ME_BLE_START_IB: {
            // beaconning is not a state so can just request it any time
            log_info("BLE:ib go in scan");
            sendIBStart(ctx);
            return SM_STATE_CURRENT;
        }            
        case ME_BLE_STOP_IB: {
            // beaconning is not a state so can just request it any time
            log_info("BLE:ib end");
            sendIBStop(ctx);
            return SM_STATE_CURRENT;
        }            
        // Can ignore a start scan request? - no might have changed uuid
        case ME_BLE_START_SCAN: {
            // just send ourselves a CC_RETRY to redo start
            sm_sendEvent(ctx->mySMId, ME_CC_RETRY, NULL);
            return SM_STATE_CURRENT;
        }
        case ME_BLE_UART_CONN: {
            // Not allowed
            callCB(ctx, WBLE_UART_DISC, NULL);
            return SM_STATE_CURRENT;
        }

        case ME_BLE_STOP_SCAN: {            
            // Back to idle but on mode (exit does the stop)
            return MS_BLE_ON;
        }
        case ME_BLE_RET_OK: {
            // ignore any non-ib return data
            return SM_STATE_CURRENT;
        }
        case ME_BLE_RET_ERR: {
            sm_timer_startE(ctx->mySMId, UART_CMD_RETRY_TIMEMS, ME_CC_RETRY);      // give it some space and retry
            return SM_STATE_CURRENT;
        }

        case ME_BLE_UPDATE: {
            // uart callback parsed lines and updates the list of currently visible ibeacons
            // data is directly the index in list TODO could be out of date.... too bad...
            ibeacon_data_t* ib=getIB((int)data);
            // if cb call it
            callCB(ctx, WBLE_SCAN_RX_IB, ib);
#ifdef DEBUG_BLE
            log_debug(".");
#endif
            return SM_STATE_CURRENT;
        }
        default: {
            sm_default_event_log(ctx->mySMId, "BLE", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}

// check if cross-connected comm
static SM_STATE_ID_t State_CheckUARTConn(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            uartRequest(ctx, true);      // request uart comm to BLE

            sm_timer_startE(ctx->mySMId, UART_ENABLE_TIMEMS, ME_CC_RETRY);      // wait for uart to be ready

            sm_timer_start(ctx->mySMId, 500);           // give up if no response in this time

            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            sm_timer_stopE(ctx->mySMId, ME_CC_RETRY);       // As not stoppped automatically by change of state
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            // No response, back to on
            log_warn("BLU: no CC");
            // if cb call it
            callCB(ctx, WBLE_COMM_FAIL, NULL);
            return MS_BLE_ON;
        }
        case ME_CC_RETRY: {
            // Send a "AT+CONN?" to check if cross connected to NUS serial BLE client already..
            // need a return of "2" to continue
            if (wskt_write(ctx->cnx, (uint8_t*)BLE_CHECKCONN, strlen(BLE_CHECKCONN))<0) {
                log_warn("BLU:CC2 fail");
                // if cb call it
                callCB(ctx, WBLE_COMM_FAIL, NULL);
                return MS_BLE_ON;
            }
            log_debug("BLU: CC2 sent");
            return SM_STATE_CURRENT;
        }
        case ME_BLE_UART_DISC: {
            return MS_BLE_ON;       // abandon
        }

        case ME_BLE_RET_OK: {       // return is just OK or READY - must wait for proper AT+CONN? response
            log_debug("BLU: cok-reCC");
            sm_timer_startE(ctx->mySMId, UART_CMD_RETRY_TIMEMS, ME_CC_RETRY);      // wait for uart to be ready
            return SM_STATE_CURRENT;
        }
        case ME_BLE_RET_ERR: {       // return is just OK or READY - must wait for proper AT+CONN? response
            log_debug("BLU: cok-reCC");
            sm_timer_startE(ctx->mySMId, UART_CMD_RETRY_TIMEMS, ME_CC_RETRY);      // wait for uart to be ready
            return SM_STATE_CURRENT;
        }
        case ME_BLE_RET_INT: {        // return is an integer which tells us connection status
#ifdef DEBUG_BLE
            log_debug("BLU: con status=%d", (uint32_t)data);
#endif
            // 0=no ble nus client, 1=ble nus client but not connected to uart, 2=cross-connection so go!
            if (((uint32_t)data) == 2) {                
                // Go and enable serial connections from remote people
                return MS_BLE_UART_RUNNING;
            } else {
                log_debug("BLU: no cc (%d)", (uint32_t)data);
                // if cb call it
                callCB(ctx, WBLE_UART_DISC, NULL);
                // And back to on
                return MS_BLE_ON;
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


static SM_STATE_ID_t State_UARTRunning(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            log_info("BLU:serialing");
            // if cb call it (now we are in the right state, so if user writes to line it should go thru)
            callCB(ctx, WBLE_UART_CONN, NULL);
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
            // Ensure we break connection 
            wskt_write(ctx->cnx, (uint8_t*)BLE_DISABLE_SERIAL, strlen(BLE_DISABLE_SERIAL));
            log_debug("BLU: set connnections serial as off");
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
        case ME_BLE_UART_DISC: {
            return MS_BLE_ON;
        }

        default: {
            sm_default_event_log(ctx->mySMId, "BLU", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}


// start ibeaconning and return to ON once confirmed
static SM_STATE_ID_t State_StartIB(void* arg, int e, void* data) {
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            uartRequest(ctx, true);      // request uart comm to BLE

            sm_timer_startE(ctx->mySMId, UART_ENABLE_TIMEMS, ME_CC_RETRY);      // wait for uart to be ready to send command

            // and overall timer in case of no response
            sm_timer_start(ctx->mySMId, 500);          

            return SM_STATE_CURRENT;
        }
        case SM_EXIT: {
            sm_timer_stopE(ctx->mySMId, ME_CC_RETRY);       // As not stoppped automatically by change of state
            return SM_STATE_CURRENT;
        }
        case SM_TIMEOUT: {
            // No response, back to on
            log_warn("BLU: no IB Resp");
            callCB(ctx, WBLE_COMM_FAIL, NULL);
            return MS_BLE_ON;
        }
        case ME_CC_RETRY: {
            // Send the ib start command
            sendIBStart(ctx);
            log_debug("BLU: ib start sent");
            return SM_STATE_CURRENT;
        }
        case ME_BLE_UART_DISC: {
            callCB(ctx, WBLE_COMM_FAIL, NULL);
            return MS_BLE_ON;       // abandon
        }

        case ME_BLE_RET_OK: {       // return is just OK or READY
            log_debug("BLU: ib ok");
            callCB(ctx, WBLE_COMM_IB_RUNNING, NULL);        // Tell user
            return MS_BLE_ON;
        }
        case ME_BLE_RET_ERR: {       // return is ERROR
            log_debug("BLU: ib nok");
            // Retry...
            sm_timer_startE(ctx->mySMId, UART_CMD_RETRY_TIMEMS, ME_CC_RETRY); 
            return SM_STATE_CURRENT;
        }
        case ME_BLE_RET_INT: {        // not expected an int return
            log_debug("BLU: con status=%d", (uint32_t)data);
            // retry
            sm_timer_startE(ctx->mySMId, UART_CMD_RETRY_TIMEMS, ME_CC_RETRY); 
            return SM_STATE_CURRENT;
        }
        case ME_BLE_OFF: {
            // gave up - shut down
            callCB(ctx, WBLE_COMM_FAIL, NULL);
            return MS_BLE_STOPPINGCOMM;
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
    struct blectx* ctx = (struct blectx*)arg;
    switch(e) {
        case SM_ENTER: {
            // Just delays before going back to OFF
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
            callCB(ctx, WBLE_COMM_OK, NULL);
            
            return MS_BLE_ON;
        }
        default: {
            sm_default_event_log(ctx->mySMId, "BLE", e);
            return SM_STATE_CURRENT;
        }
    }
    assert(0);      // shouldn't get here
}
// State table : note can be in any order as the 'id' field is what maps the state id to the rest
static const SM_STATE_t _bleSM[MS_BLE_LAST] = {
    {.id=MS_BLE_OFF,        .name="BleOff",       .fn=State_Off},
    {.id=MS_BLE_WAITPOWERON,.name="BleWaitPower", .fn=State_WaitPoweron},
    {.id=MS_BLE_STARTING,   .name="BleStarting",  .fn=State_Starting},    
    {.id=MS_BLE_ON,         .name="BleOnIdle", .fn=State_On},
    {.id=MS_BLE_SCANNING,   .name="BleScanning", .fn=State_Scanning},    
    {.id=MS_BLE_UART_CHECK,   .name="BleUARTCheck", .fn=State_CheckUARTConn},    
    {.id=MS_BLE_UART_RUNNING,   .name="BleUARTRun", .fn=State_UARTRunning},    
    {.id=MS_BLE_START_IB,   .name="BleStartIB", .fn=State_StartIB},    
    {.id=MS_BLE_STOPPINGCOMM, .name="BleStopping", .fn=State_StoppingComm},    
};

// Called from sysinit via reference in pkg.yml
void* wble_mgr_init(const char* dname, uint32_t baudrate, int8_t pwrPin, int8_t uartPin, int8_t uartSelect) {
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
    _ctx.uartPin = uartPin;
    if (_ctx.pwrPin>=0) {
        // Note 1 is OFF so start with it off
        GPIO_define_out("blepower", _ctx.pwrPin, 1, LP_SLEEP, PULL_UP);
    }
    if (_ctx.uartPin>=0) {
        // Note 1 is OFF so start with it off
        GPIO_define_out("bleuart", _ctx.uartPin, 1, LP_SLEEP, PULL_UP);
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

// Request to use ble comm directly as remote uart (possible during ibeaconning). 
void wble_line_open(void* c) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    // Ask SM to run (if it is aleady this will be ignored)
    sm_sendEvent(ctx->mySMId, ME_BLE_UART_CONN, NULL);
    return;
}

int wble_line_write(void* c, uint8_t* data, uint32_t sz) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;

    if (ctx->cnx==NULL) {
        log_warn("BLU:can't write as no uart dev..");
        return SKT_NODEV;
    }
    // Write thru directly if in correct state (note can't pass to SM 'cleanly' as would need to copy buffer...)
    if (sm_getCurrentState(ctx->mySMId)==MS_BLE_UART_RUNNING) {
        return wskt_write(ctx->cnx, data, sz);
    }
    return SKT_EINVAL;       // No write for you my friend
}

void wble_line_close(void* c) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;

    sm_sendEvent(ctx->mySMId, ME_BLE_UART_DISC, NULL);
    // leave any buffers to be tx'd in their own time
    return;
}

void wble_scan_start(void* c, const uint8_t* uuid, uint16_t majorStart, uint16_t majorEnd, uint32_t sz, ibeacon_data_t* list) {
    assert(c!=NULL);
    assert(list!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    // reset stats for new scan
    ctx->nbRxNew=0;
    ctx->nbRxNewR=0;
    ctx->nbRxUpdate=0;
    ctx->nbRxNoSpace=0;
    ctx->nbRxBadMajor=0;
    if (uuid!=NULL) {
        memcpy(ctx->uuid, uuid, UUID_SZ);
    } else {
        // No uuid filter, set to 0
        memset(ctx->uuid, 0, UUID_SZ);
    }
    ctx->majorStart = majorStart;
    ctx->majorEnd = majorEnd;
    ctx->ibListSz = sz;
    ctx->ibList = list;

    sm_sendEvent(ctx->mySMId, ME_BLE_START_SCAN, NULL);

}
void wble_scan_stop(void* c) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    sm_sendEvent(ctx->mySMId, ME_BLE_STOP_SCAN, NULL);
}

void wble_ibeacon_start(void* c, const uint8_t* uuid, uint16_t maj, uint16_t min, uint8_t extra, uint16_t interMS, int8_t txpower) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;

    if (uuid!=NULL && Util_notAll0(uuid, UUID_SZ)) {
        memcpy(ctx->uuid, uuid, UUID_SZ);
    } else {
        // No uuid given, set to "default"
        memcpy(ctx->uuid, WYRES_UUID, UUID_SZ);
    }
    ctx->ibMajor = maj;
    ctx->ibMinor = min;
    ctx->ibExtra = extra;
    ctx->ibInterMS = interMS;
    ctx->ibTxPower = txpower;
    sm_sendEvent(ctx->mySMId, ME_BLE_START_IB, NULL);
}

void wble_ibeacon_stop(void* c) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    sm_sendEvent(ctx->mySMId, ME_BLE_STOP_IB, NULL);
}

void wble_stop(void* c) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    sm_sendEvent(ctx->mySMId, ME_BLE_OFF, NULL);
}

// get number of ibs we have seen so far  (optionally count only those active in last X seconds if activeInLastX >0)
int wble_getNbIBActive(void* c, uint32_t activeInLastX) {
    assert(c!=NULL);
    uint32_t now = TMMgr_getRelTimeSecs();
    int nb=0;
    struct blectx* ctx = (struct blectx*)c;
    for(int i=0;i<ctx->ibListSz; i++) {
        // lastSeenAt == 0 -> unused entry
        if (ctx->ibList[i].lastSeenAt!=0 &&
            (activeInLastX==0 || ((now - ctx->ibList[i].lastSeenAt) < activeInLastX))) {
            nb++;
        }
    }
    return nb;
}
// reset the list of ibeacon data actives, all or just those not see in the last X seconds
void wble_resetList(void* c, uint32_t notSeenInLastX) {
    assert(c!=NULL);
    struct blectx* ctx = (struct blectx*)c;
    uint32_t now = TMMgr_getRelTimeSecs();
    for(int i=0;i<ctx->ibListSz; i++) {
        // lastSeenAt == 0 -> unused entry
        if (ctx->ibList[i].lastSeenAt!=0 &&
               (notSeenInLastX==0 || ((now - ctx->ibList[i].lastSeenAt) > notSeenInLastX))) {
            ctx->ibList[i].lastSeenAt=0;
        }
    }
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
            if (_ctx.ibList[i].lastSeenAt!=0 && ctx->ibList[i].rssi>bestRssi && ctx->ibList[i].rssi<prevRssi) {
                idxToGet = i;
                bestRssi = ctx->ibList[i].rssi;
            }
        }
        if (idxToGet>=0) {
            prevRssi = bestRssi;
            // if N with same RSSI, idxToGet shows the first in the list.
            // go from there to the end, and add all those with the same 'best' rssi.
            for(int i=idxToGet;i<ctx->ibListSz;i++) {
                if (_ctx.ibList[i].lastSeenAt!=0 && ctx->ibList[i].rssi == bestRssi) {
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
// Send ibeaconning on command
static void sendIBStart(struct blectx* ctx) {
    // Config to type ibeacon and will start beaconning using params passed
    // AT_IB_START <uuid>,<major>,<minor>,<extrabyte>,<interval in ms>,<txpower>
    // All values in hex with leading 0s for fixed length
    /// Note non v2.0 BLE module doesnt support this
    sprintf(ctx->txLine,"AT+IB_START,%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x,%04x,%04x,%02x,%04x,%d\r\n",
            ctx->uuid[0],ctx->uuid[1],ctx->uuid[2],ctx->uuid[3],ctx->uuid[4],ctx->uuid[5],ctx->uuid[6],ctx->uuid[7],
            ctx->uuid[8],ctx->uuid[9],ctx->uuid[10],ctx->uuid[11],ctx->uuid[12],ctx->uuid[13],ctx->uuid[14],ctx->uuid[15], 
            ctx->ibMajor, ctx->ibMinor, ctx->ibExtra, ctx->ibInterMS, ctx->ibTxPower);    
    wskt_write(ctx->cnx, (uint8_t*)&ctx->txLine[0], strlen(ctx->txLine));
}
// Send ibeaconning off command
static void sendIBStop(struct blectx* ctx) {
    // stop ibeaconning
    wskt_write(ctx->cnx, (uint8_t*)BLE_IB_STOP, strlen(BLE_IB_STOP));
}

// manage IB list
static ibeacon_data_t* getIB(int idx) {
    if (idx<0 || idx>=_ctx.ibListSz) {
        return NULL;
    }
    return &_ctx.ibList[idx];
}

// Add scanned IB to list if not already present  else update it
static int addIB(ibeacon_data_t* ibp) {
    // Find if already in list, update if so
    int freeEntry = -1;
    for(int i=0;i<_ctx.ibListSz; i++) {
        // lastSeenAt == 0 -> unused entry
        if (_ctx.ibList[i].lastSeenAt!=0) {
            if (ibp->major==_ctx.ibList[i].major &&
                    ibp->minor==_ctx.ibList[i].minor) {
                _ctx.ibList[i].rssi = ibp->rssi;
                _ctx.ibList[i].extra = ibp->extra;
                _ctx.ibList[i].lastSeenAt = TMMgr_getRelTimeSecs();
                _ctx.nbRxUpdate++;
                return i;
            }
        } else {
            // Find free entry in list while we're here in case we need to insert
            if (freeEntry < 0) {
                freeEntry = i;
            }
        }
    }
    // Not in the last, do we have  space in list to add it?
    if (freeEntry>=0) {
        // yes.
        _ctx.ibList[freeEntry].major = ibp->major;
        _ctx.ibList[freeEntry].minor = ibp->minor;
        _ctx.ibList[freeEntry].rssi = ibp->rssi;
        _ctx.ibList[freeEntry].extra = ibp->extra;
        // Nota : if the ibeacon changes its devAddr randomly (as they do) then this field is not useful
        memcpy(_ctx.ibList[freeEntry].devaddr, ibp->devaddr, DEVADDR_SZ);
        _ctx.ibList[freeEntry].new = true;
        _ctx.ibList[freeEntry].inULCnt = 0;            // Flag it as new and not UL'd yet
        _ctx.ibList[freeEntry].firstSeenAt = TMMgr_getRelTimeSecs();
        _ctx.ibList[freeEntry].lastSeenAt = TMMgr_getRelTimeSecs();
        _ctx.nbRxNew++;
        return freeEntry;
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
    // Uart pass-thru mode : just send line up to the user
    if (sm_getCurrentState(_ctx.mySMId)==MS_BLE_UART_RUNNING) {
        log_debug("wbu:[%s]", line);
        callCB(&_ctx, WBLE_UART_RX, (void*)line);
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
    } else if (strncasecmp(line, "READY", 5)==0) {
        sm_sendEvent(_ctx.mySMId, ME_BLE_RET_OK, NULL);
    } else if (strlen(line)<15) {
        int val = -1;
        if (sscanf(line, "%d", &val)<1) {
            // Any none OK/ERROR/ble info line is considered as OK
#ifdef DEBUG_BLE
            log_debug("BLE:[%s]", line);
#endif
            sm_sendEvent(_ctx.mySMId, ME_BLE_RET_OK, NULL);
        } else  {
            // Got a decimal value
#ifdef DEBUG_BLE
            log_debug("BLE:[%s]=%d", line, val);
#endif
            sm_sendEvent(_ctx.mySMId, ME_BLE_RET_INT, (void*)val);
        }
    } else {
        // Parse it as ibeacon data
        ibeacon_data_t ib;
        // <MMMM>,<mmmm>,<EX>,<RSSI>,<devAddr>
        if (sscanf(line, "%4x,%4x,%2x,%2x,%02x%02x%02x%02x%02x%02x", (int*)&ib.major, (int*)&ib.minor, (int*)&ib.extra, (int*)&ib.rssi, 
                (int*)&ib.devaddr[0],(int*)&ib.devaddr[1],(int*)&ib.devaddr[2],(int*)&ib.devaddr[3],(int*)&ib.devaddr[4],(int*)&ib.devaddr[5])<4) {
#ifdef DEBUG_BLE
            log_debug("BLE:bad parse [%s]", line);
#endif
        } else {
            // Is major vale between majorStart and majorEnd filters?
            if (ib.major>=_ctx.majorStart && ib.major<=_ctx.majorEnd) {
                // Add to ibeacon fifo
                int idx = addIB(&ib);
                if (idx>=0) {
                    // Tell SM
                    sm_sendEvent(_ctx.mySMId, ME_BLE_UPDATE, (void*)idx);
                } else {
                    log_warn("BLE:saw %4x,%4x list full",ib.major, ib.minor);
                }
            } else {
                _ctx.nbRxBadMajor++;
#ifdef DEBUG_BLE
                log_debug("?");
#endif
//                log_debug("BLE:saw ib %4x,%4x but outside major filter range",ib.major, ib.minor);
            }
        }
    }

}

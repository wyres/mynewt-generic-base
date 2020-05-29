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
/**
 * UART manager device that maps char by char to line by line operation, using a socket paradigm
 * This is a wyres device, the methods are only called via the socket emulator (not directly)
 * Each instance of a UART must be initialised at startup by registering it with the socket driver (see wskt)
 */

#include <stdint.h>
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "uart/uart.h"

#include "wyres-generic/uartlinemgr.h"
#include "wyres-generic/wutils.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/wskt_driver.h"
#include "wyres-generic/circbuf.h"
#include "wyres-generic/uartselector.h"
#include "wyres-generic/ledmgr.h"

// If got a separate debug uart line, then allowed to do real logs in here, otherwise its the debugger only version that does actually output!
#define log_uartbdg log_noout_fn

#define MAX_NB_UARTS MYNEWT_VAL(MAX_UARTS)
#define UART_LINE_SZ (WSKT_BUF_SZ)

// Candidates for the end of line char
#define LF (0x0A)           // \n  - default end of line
#define CR (0x0D)           // \r

static struct UARTDeviceCfg {
    const char* dname;
    struct os_dev* uartDev;
    uint32_t baud;
    uint8_t rxBuff_data_space[UART_LINE_SZ+1];
    circ_bbuf_t rxBuff;
    uint8_t txBuff_data_space[UART_LINE_SZ+1];
    circ_bbuf_t txBuff;
    uint8_t rxIdx;
    uint8_t txIdx;
    bool filterASCII;
    bool isSuspended;       // for power management
    char eol;
    int8_t uartSelect;
} _cfgs[MAX_NB_UARTS];          
static int _nbUARTCfgs=0;

// predefine privates
static bool openuart(struct UARTDeviceCfg* cfg);
static int uart_line_open(wskt_t* skt);
static int uart_line_ioctl(wskt_t* skt, wskt_ioctl_t* cmd);
static int uart_line_write(wskt_t* skt, uint8_t* data, uint32_t sz);
static int uart_line_close(wskt_t* skt);
static int uart_rx_cb(void*, uint8_t c);
//static void uart_tx_ready(void* ctx);
static int uart_tx_cb(void* ctx);
//static void lp_change(LP_MODE_t p, LP_MODE_t n);

static wskt_devicefns_t _myDevice = {
    .open = &uart_line_open,
    .ioctl = &uart_line_ioctl,
    .write = &uart_line_write,
    .close = &uart_line_close
};

static uint8_t _lineBuffer[UART_LINE_SZ];
// mutex to protect it (only used in passing)
static struct os_mutex _lbMutex;
static LP_ID_t _lpUserId;

// Called from sysinit via reference in pkg.yml
void uart_line_comm_init(void) {
    // TODO should we use mempools to handle per-device structures?
    os_mutex_init(&_lbMutex);
    // register with low power manager so we can set the level of sleep we can take.
    // The operation is essentially : if a UART device socket is OPEN, we permit SLEEP, if all are closed, we allow DEEPSLEEP
    // No action when idle sleep is entered however
    _lpUserId = LPMgr_register(NULL);

}

// Create uart device with given name (used as my dev name and also the mynewt device to open), at given baud rate
bool uart_line_comm_create(const char* dname, uint32_t baud) {
    // allocate new device cfg element
    if (_nbUARTCfgs>=MAX_NB_UARTS) {
        log_uartbdg("too many uart creates");
        return false;
    }
    // check if already created and ignore
    for(int i=0;i<_nbUARTCfgs;i++) {
        if (strncmp(dname, _cfgs[i].dname, MAX_WKST_DNAME_SZ)==0) {
            return true;        // its ok
        }
    }
    struct UARTDeviceCfg* myCfg = &_cfgs[_nbUARTCfgs++];
    myCfg->dname = dname;
    myCfg->baud = baud;
    circ_bbuf_init(&myCfg->rxBuff, &(myCfg->rxBuff_data_space[0]), UART_LINE_SZ+1);
    circ_bbuf_init(&myCfg->txBuff, &(myCfg->txBuff_data_space[0]), UART_LINE_SZ+1);
    myCfg->uartDev = NULL;
    myCfg->filterASCII = true;      // by default
    myCfg->isSuspended = false;
    // LF is default end of line as this works for BLE code and GPS
    // Note that CR is used by console (it will set the config)
    myCfg->eol = LF;
    myCfg->uartSelect = -1;
    // and register ourselves as a 'uart like' comms provider so procesing routines can read the data
    wskt_registerDevice(dname, &_myDevice, myCfg);
    return true;
}

/* OLD CODE USING HAL DIRECT
    //  Define the UART callbacks.
    int rc = hal_uart_init_cbs(uartNb,
        uart_tx_char, uart_tx_ready,
        uart_rx_cb, myCfg);
    if (rc != 0) { return rc; }
    //  Set UART parameters.
    assert(baud != 0);
    rc = hal_uart_config(uartNb,
        baud,
        8,
        1,
        HAL_UART_PARITY_NONE,
        HAL_UART_FLOW_CTL_NONE
    );
    if (rc != 0) { return rc; }
    hal_uart_start_rx(uartNb);  //  Start receiving UART data.
    hal_uart_start_tx(uartNb);  //  Start transmitting UART data.
*/

static bool openuart(struct UARTDeviceCfg* cfg) {
    // If already open, close device
    if (cfg->uartDev!=NULL) {
        os_dev_close(cfg->uartDev);
    }
    // switch to correct input
    uart_select(cfg->uartSelect);

    struct uart_conf uc = {
        .uc_speed = cfg->baud,
        .uc_databits = 8,
        .uc_stopbits = 1,
        .uc_parity = UART_PARITY_NONE,
        .uc_flow_ctl = 0,
        .uc_tx_char = uart_tx_cb,
        .uc_rx_char = uart_rx_cb,
        .uc_tx_done = NULL,
        .uc_cb_arg = cfg,
    };

    cfg->uartDev = os_dev_open(cfg->dname,
                            OS_TIMEOUT_NEVER, &uc);
    return (cfg->uartDev!=NULL);
}
// Called via device manager
static int uart_line_open(wskt_t* skt) {
    // skt is device config (shared amongst all who open same device) + per socket event/eventq to notify
    struct UARTDeviceCfg* cfg=((struct UARTDeviceCfg*)WSKT_DEVICE_CFG(skt));  
    // Note the event arg must point to the apps own buffer to copy line into
    // if 1st skt opened on this device, mae sure its powered on and open it
    if (cfg->uartDev==NULL) {
        // setup uart by opening mynewt device
        if (!openuart(cfg)) {
            log_uartbdg("open uart fail");
            return SKT_NODEV;
        }
        log_uartbdg("open uart ok");
        // A UART is active, don't go deep sleeping please
        LPMgr_setLPMode(_lpUserId, LP_DOZE);
    }
    // Not much to do, UART io already running
    return SKT_NOERR;
}
static int uart_line_ioctl(wskt_t* skt, wskt_ioctl_t* cmd) {
    struct UARTDeviceCfg* cfg=((struct UARTDeviceCfg*)WSKT_DEVICE_CFG(skt));  
    switch (cmd->cmd) {
        case IOCTL_SET_BAUD: {
            if (cfg->baud != cmd->param) {
                cfg->baud = cmd->param;
                // If already open, gotta close the device and reopen to set baud
                if (!openuart(cfg)) {
                    log_uartbdg("reopen uart fail");
                    return SKT_NODEV;
                }                
                log_uartbdg("reopen uart ok baud set to %d", cfg->baud);
            } else {
                log_uartbdg("no reopen, baud already set to %d", cfg->baud);
            }
            break;
        }

        case IOCTL_RESET: {
            if (!openuart(cfg)) {
                return SKT_NODEV;
            }
            break;
        }
        // Allow to only take ASCII chars (32-127) with LF (0x0a) as line end
        case IOCTL_FILTERASCII: {
            cfg->filterASCII = (cmd->param!=0);
            break;
        }
        // Allow to explicitly set eol char
        case IOCTL_SETEOL: {
            cfg->eol = (char)cmd->param;
            break;
        }
        case IOCTL_SELECTUART: {
            cfg->uartSelect = (int8_t)cmd->param;
            uart_select(cfg->uartSelect);
            break;
        }
        // Flush any tx or rx bytes left hanging about
        case IOCTL_FLUSHTXRX: {
            os_sr_t sr;
            OS_ENTER_CRITICAL(sr);
            circ_bbuf_flush(&cfg->txBuff);
            circ_bbuf_flush(&cfg->rxBuff);
            OS_EXIT_CRITICAL(sr);
            break;
        }
        case IOCTL_CHECKTX: {
            // check if the tx buffer empty or not (return number of bytes)
            return circ_bbuf_data_available(&cfg->txBuff);
        }
        default: {
            return SKT_EINVAL; 
        }
    }
    return SKT_NOERR; 
}
static int uart_line_write(wskt_t* skt, uint8_t* data, uint32_t sz) {
    struct UARTDeviceCfg* cfg=((struct UARTDeviceCfg*)WSKT_DEVICE_CFG(skt));  

    if (cfg->uartDev==NULL) {
        log_uartbdg("can't write as no uart dev..");
        return SKT_NODEV;
    }
    circ_bbuf_t* buf = &cfg->txBuff;
    // check if space in buffer for ALL the data
    if (sz>circ_bbuf_free_space(buf)) {
        log_uartbdg("no space in buffer for line of sz %d...", sz);
        // if not, don't take any
        return SKT_NOSPACE;
    }
    // IRQ disable TODO
    // copy it in
    for(int i=0; i<sz; i++) {
        os_sr_t sr;
        OS_ENTER_CRITICAL(sr);
        circ_bbuf_push(buf, data[i]);
        OS_EXIT_CRITICAL(sr);
    }
    // IRQ enable

    // Tell uart more tx data
    if (cfg->uartDev!=NULL) {
        uart_start_tx((struct uart_dev*)(cfg->uartDev));
    }
    return SKT_NOERR; 
}
static int uart_line_close(wskt_t* skt) {
    struct UARTDeviceCfg* cfg=((struct UARTDeviceCfg*)WSKT_DEVICE_CFG(skt));  

    // Iff last skt then close mynewt uart device
    if (wskt_getOpenSockets(cfg->dname, NULL, 0)<=1) {
        // hmmmm.. should wait for tx to finish : TODO
        if (cfg->uartDev!=NULL) {
            os_dev_close(cfg->uartDev);
            cfg->uartDev = NULL;
        }
        // Dont do logging in here (as will re-open the debug uart potentially, and stop deep sleeping!!!)
        log_noout("closed last socket on uart %s", cfg->dname);
        // Check if ALL sockets on ALL devices are closed, in which case we allow DEEP SLEEP
        bool allClosed = true;
        for (int i=0;i<_nbUARTCfgs; i++) {
            if (_cfgs[i].uartDev!=NULL) {
                allClosed = false;
            }
        }
        if (allClosed) {
            LPMgr_setLPMode(_lpUserId, LP_OFF);     // no more uart cnx open, low power can go as deep as you want!
        }
    }
    // leave any buffers to be tx'd in their own time
    return SKT_NOERR; 
}
// IRQ for rx byte
static int uart_rx_cb(void* ctx, uint8_t c) {
    struct UARTDeviceCfg* myCfg = (struct UARTDeviceCfg*)ctx;
    // Add to line in circ buffer iff not filtering, or is EOL or TAB (used as a seperator)
    if (myCfg->filterASCII && (c<0x20 || c>0x7E) && c!=myCfg->eol && c!=0x09) {
        return 0;
    }
    circ_bbuf_push(&(myCfg->rxBuff), c);
    // if full or EOL, copy to all sockets (get list from wskt mgr)
    if (c==myCfg->eol || circ_bbuf_free_space(&(myCfg->rxBuff))==0) {
        // copy out line first to local STATIC buffer (stack space!)
        // MUTEX NOT REQUIRED IN ISR CALLED ROUTINE (normally)
//        os_mutex_pend(&_lbMutex, OS_TIMEOUT_NEVER);
        uint8_t lineLen = 0;
        bool copied = false;
        while(!copied && lineLen<UART_LINE_SZ) {
            if (circ_bbuf_pop(&(myCfg->rxBuff), &(_lineBuffer[lineLen]))<0) {
                // done, its empty
                copied = true;
            } else {
                // did we just copy a EOL?
                if (_lineBuffer[lineLen]==myCfg->eol) {
                    // done, EOL
                    // back up 1 coz don't want the EOL
                    lineLen--;
                    copied = true;                
                }
            }
            lineLen++;
        }
        // Make it a null terminated string
        _lineBuffer[lineLen++] = 0;
//        os_mutex_release(&_lbMutex);
        // We don't give up empty lines
        if (lineLen>1) {
//            log_uartbdg("%s got line", myCfg->dname);
            // now send it off
            wskt_t* os[8];      // no more than 8 open sockets on my device please
            uint8_t ns = wskt_getOpenSockets(myCfg->dname, &(os[0]), 8);
            // For each socket
            for(int i=0;i<ns;i++) {
                // get event out of socket
                struct os_event* e = os[i]->evt;
                if (e!=NULL) {
                    if (!e->ev_queued) {
                        // else copy in line (including the null terminator)
                        uint8_t* sbuf = (uint8_t*)(e->ev_arg);
                        memcpy(sbuf, _lineBuffer, lineLen);
                        // and post event to the listener's task
                        os_eventq_put(os[i]->eq, e);
                    } else {
                        // if already on their q then... discard for this guy???
                    }
                } else {
                    // ok, this guy doesn't care about RX - thats ok...
                }
            }        
        }
    }

    // return -1 if no more rx space
    if (circ_bbuf_free_space(&(myCfg->rxBuff))==0) {
        return -1;      // full
    }
    return 0;
}

static int uart_tx_cb(void* ctx) {
    struct UARTDeviceCfg* myCfg = (struct UARTDeviceCfg*)ctx;
    // next char from circular bufer
    // note that the circular buffer is protected from this IRQ CB via OS_ENTER/EXIT_CRITICAL() which disables IRQs
    uint8_t c;
    if (circ_bbuf_pop(&(myCfg->txBuff), &c)<0) {
        // No more data to tx
        return -1;
    }
    return c;
}
/*
// IRQ for tx can take a byte
static void uart_tx_ready(void* ctx) {
    struct UARTDeviceCfg* myCfg = (struct UARTDeviceCfg*)ctx;
    // give it byte from tx buffer if available
    uint8_t c;
    if (circ_bbuf_pop(&(myCfg->txBuff), &c)<0) {
        // No more data to tx, dont care
        return;
    }
    // push it back
    circ_bbuf_push(&(myCfg->txBuff), c);
}
*/
/* uart deals with LP by blocking DEEPSLEEP unless all uarts are closed
// LOw power mode change - disable UART device(s) in deep low power: DO NOT LOG OR TAKE TOO MUCH STACK
static void lp_change(LP_MODE_t oldmode, LP_MODE_t newmode) {
    if (newmode>=LP_DEEPSLEEP) {
        // need to call all devices we know about to deinit their hw
        for (int i=0;i<_nbUARTCfgs;i++) {
            if (!_cfgs[i].isSuspended) {
                _cfgs[i].isSuspended = true;
                log_noout("UM:suspend %s", _cfgs[i].dname);
                // we check if already de-inited as may not be robust on double suspend...
    //            os_dev_suspend(_cfgs[i].dname, 0, 1);
            }
        }
    } else {
        // and here to reinit them...
        for (int i=0;i<_nbUARTCfgs;i++) {
            // check if alredy inited as may not be robust on double resume......
            if (_cfgs[i].isSuspended) {
                _cfgs[i].isSuspended = false;
//              os_dev_resume(_cfgs[i].dname, 0, 1);
                log_noout("UM:resume %s", _cfgs[i].dname);   // log after resume...
            }
        }
    }
}
*/
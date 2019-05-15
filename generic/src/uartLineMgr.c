/**
 * Wyres private code
 * UART manager device that maps char by char to line by line operation, using a socket paradigm
 * This is a wyres device, the methods are only called via the socket emulator (not directly)
 * Each instance of a UART must be initialised at startup by registering it with the socket driver (see wskt)
 */

#include <stdint.h>
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "uart/uart.h"

#include "wyres-generic/uartLineMgr.h"
#include "wyres-generic/wutils.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/wskt_driver.h"
#include "wyres-generic/circbuf.h"

#define MAX_NB_UARTS MYNEWT_VAL(MAX_UARTS)
#define UART_LINE_SZ (WSKT_BUF_SZ)

//  task should be high pri as does very little but wants to do it in real time
#define UART_COMM_TASK_PRIO       MYNEWT_VAL(UART_TASK_PRIO)
#define UART_COMM_TASK_STACK_SZ   OS_STACK_ALIGN(256)

static os_stack_t _uart_task_stack[UART_COMM_TASK_STACK_SZ];
static struct os_task _uart_task_str;

static struct UARTDeviceCfg {
    char dname[MAX_WKST_DNAME_SZ];
    struct os_dev* uartDev;
    uint32_t baud;
    uint8_t rxBuff_data_space[UART_LINE_SZ+1];
    circ_bbuf_t rxBuff;
    uint8_t txBuff_data_space[UART_LINE_SZ+1];
    circ_bbuf_t txBuff;
    uint8_t rxIdx;
    uint8_t txIdx;
} _cfgs[MAX_NB_UARTS];          // TODO use mempool
static int _nbUARTCfgs=0;

// predefine privates
static bool openuart(struct UARTDeviceCfg* cfg);
static void uart_comm_task(void* arg);
static int uart_line_open(wskt_t* skt);
static int uart_line_ioctl(wskt_t* skt, wskt_ioctl_t* cmd);
static int uart_line_write(wskt_t* skt, uint8_t* data, uint32_t sz);
static int uart_line_close(wskt_t* skt);
static int uart_rx_cb(void*, uint8_t c);
//static void uart_tx_ready(void* ctx);
static int uart_tx_cb(void* ctx);

static wskt_devicefns_t _myDevice = {
    .open = &uart_line_open,
    .ioctl = &uart_line_ioctl,
    .write = &uart_line_write,
    .close = &uart_line_close
};

static uint8_t _lineBuffer[UART_LINE_SZ];
// mutex to protect it (only used in passing)
static struct os_mutex _lbMutex;

// Called from sysinit via reference in pkg.yml
void uart_line_comm_init(void) {
    // TODO should we use mempools to handle per-device structures?
        // Create the comm handler task
    os_task_init(&_uart_task_str, "uart_task", uart_comm_task, NULL, UART_COMM_TASK_PRIO,
                 OS_WAIT_FOREVER, _uart_task_stack, UART_COMM_TASK_STACK_SZ);
    os_mutex_init(&_lbMutex);
}

// Create uart device with given name (used as my dev name and also the mynewt device to open), at given baud rate
bool uart_line_comm_create(char* dname, uint32_t baud) {
    // allocate new device cfg element
    if (_nbUARTCfgs>=MAX_NB_UARTS) {
        //log("too many uart creates");
        return false;
    }
    struct UARTDeviceCfg* myCfg = &_cfgs[_nbUARTCfgs++];
    strncpy(myCfg->dname, dname, MAX_WKST_DNAME_SZ-1);
    myCfg->dname[MAX_WKST_DNAME_SZ-1]= '\0';
    myCfg->baud = baud;
    circ_bbuf_init(&myCfg->rxBuff, &(myCfg->rxBuff_data_space[0]), UART_LINE_SZ+1);
    circ_bbuf_init(&myCfg->txBuff, &(myCfg->txBuff_data_space[0]), UART_LINE_SZ+1);
    myCfg->uartDev = NULL;

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
    struct uart_conf uc = {
        .uc_speed = cfg->baud,
        .uc_databits = 8,
        .uc_stopbits = 1,
        .uc_parity = UART_PARITY_NONE,
        .uc_flow_ctl = 0,
        .uc_tx_char = uart_tx_cb,
        .uc_rx_char = uart_rx_cb,
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
            log_noout("open uart fail");
            return SKT_NODEV;
        }
        log_noout("open uart ok");
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
                    log_warn("reopen uart fail");
                    return SKT_NODEV;
                }                
                log_noout("reopen uart ok baud set to %d", cfg->baud);
            } else {
                log_noout("no reopen, baud already set to %d", cfg->baud);
            }
            break;
        }

        case IOCTL_RESET: {
            if (!openuart(cfg)) {
                return SKT_NODEV;
            }
            break;
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
        log_noout("can't write as no uart dev..");
        return SKT_NODEV;
    }
    circ_bbuf_t* buf = &cfg->txBuff;
    // check if space in buffer for ALL the data
    if (sz>circ_bbuf_free_space(buf)) {
        log_noout("no space in buffer for line of sz %d...", sz);
        // if not, don't take any
        return SKT_NOSPACE;
    }
    // Mutex protect
    // copy it in
    for(int i=0; i<sz; i++) {
        circ_bbuf_push(buf, data[i]);
    }
    // mutex release

    // Tell uart more tx data
    if (cfg->uartDev!=NULL) {
        uart_start_tx((struct uart_dev*)(cfg->uartDev));
    }
    return SKT_NOERR; 
}
static int uart_line_close(wskt_t* skt) {
    struct UARTDeviceCfg* cfg=((struct UARTDeviceCfg*)WSKT_DEVICE_CFG(skt));  

    // Iff last skt then power down
    if (wskt_getOpenSockets(cfg->dname, NULL, 0)<=1) {
        if (cfg->uartDev!=NULL) {
            os_dev_close(cfg->uartDev);
            cfg->uartDev = NULL;
        }
        // clean buffers
        circ_bbuf_init(&cfg->rxBuff, &(cfg->rxBuff_data_space[0]), UART_LINE_SZ+1);
        circ_bbuf_init(&cfg->txBuff, &(cfg->txBuff_data_space[0]), UART_LINE_SZ+1);
        log_noout("closed last socket on uart %s", cfg->dname);
    }

    return SKT_NOERR; 
}

// IRQ for rx byte
static int uart_rx_cb(void* ctx, uint8_t c) {
    struct UARTDeviceCfg* myCfg = (struct UARTDeviceCfg*)ctx;
    // Add to line in circ buffer
    circ_bbuf_push(&(myCfg->rxBuff), c);
    // if full or CR, copy to all sockets (get list from wskt mgr)
    if (c=='\n' || circ_bbuf_free_space(&(myCfg->rxBuff))==0) {
        // Send event to be processed by task? or just do it here?
        // copy out line first to local STATIC buffer (stack space!)
        // MUTEX
        os_mutex_pend(&_lbMutex, OS_TIMEOUT_NEVER);
        uint8_t lineLen = 0;
        bool copied = false;
        while(!copied && lineLen<UART_LINE_SZ) {
            if (circ_bbuf_pop(&(myCfg->rxBuff), &(_lineBuffer[lineLen]))<0) {
                // done, its empty
                _lineBuffer[lineLen] = '\n';
                copied = true;
            } else {
                // did we just copy a CR?
                if (_lineBuffer[lineLen]=='\n') {
                    // done, EOL
                    copied = true;                
                }
            }
            lineLen++;
        }
        // Make it a null terminated string
        _lineBuffer[lineLen++] = '\0';
        os_mutex_release(&_lbMutex);
        log_noout("%s for line for listeners", myCfg->dname);
        // now send it off
        wskt_t* os[8];      // no more than 8 open sockets on my device please
        uint8_t ns = wskt_getOpenSockets(myCfg->dname, &(os[0]), 8);
        // For each socket
        for(int i=0;i<ns;i++) {
            // get event out of socket
            struct os_event* e = os[i]->evt;
            if (e!=NULL) {
                if (e->ev_queued==false) {
                    // if already on their q then... discard for this guy???
                } else {
                    // else copy in line (including the null terminator)
                    uint8_t* sbuf = (uint8_t*)(e->ev_arg);
                    memcpy(_lineBuffer, sbuf, lineLen);
                    // and post event to the listener's task
                    os_eventq_put(os[i]->eq, e);
                }
            } else {
                // ok, this guy doesn't care about RX - thats ok...
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
// TODO not sure we need a task in the uart driver in fact
static void uart_comm_task(void* arg) {
    while(1) {
        os_time_delay(OS_TICKS_PER_SEC);
    }

}

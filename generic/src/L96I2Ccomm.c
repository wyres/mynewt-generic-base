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
 * L96 manager device that maps I2C L96 operation to a uart like line by line operation, using a socket paradigm
 * This is a wyres device, the methods are only called via the socket emulator (not directly)
 * Each instance of a UART must be initialised at startup by registering it with the socket driver (see wskt)
 * The device can then be used like a UART socket...
 */

#include <stdint.h>

#include "sysinit/sysinit.h"
#include "os/os.h"
#if MYNEWT_VAL(USE_BUS_I2C)
#include "bus/drivers/i2c_common.h"
#else
#include "hal/hal_i2c.h"
#endif  /* USE_BUS_I2C */
#include "bsp/bsp.h"
#include "syscfg/syscfg.h"

#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/wutils.h"
#include "wyres-generic/wskt_driver.h"
#include "wyres-generic/circbuf.h"

// Timeout for I2C accesses in 'ticks'
#define I2C_ACCESS_TIMEOUT (100)

#define MAX_NB_L96  MYNEWT_VAL(MAX_NB_L96)
#define L96_LINE_SZ  MYNEWT_VAL(WSKT_BUF_SZ)

// Led task should be high pri as does very little but wants to do it in real time
#define L96COMM_TASK_PRIO       MYNEWT_VAL(L96COMM_TASK_PRIO)
#define L96COMM_TASK_STACK_SZ   OS_STACK_ALIGN(256)

static os_stack_t _l96_task_stack[L96COMM_TASK_STACK_SZ];
static struct os_task _l96_task_str;

static struct L96DeviceCfg {
    char dname[MAX_WKST_DNAME_SZ];
    bool active;
#if MYNEWT_VAL(USE_BUS_I2C)
    struct bus_i2c_node_cfg i2cCfg;
    struct bus_i2c_node i2cDev;
#else
    uint8_t i2cDev;
    uint8_t i2cAddr;
#endif  /* USE_BUS_I2C */
    uint8_t rxBuff_data_space[L96_LINE_SZ+1];
    circ_bbuf_t rxBuff;
    uint8_t txBuff_data_space[L96_LINE_SZ+1];
    circ_bbuf_t txBuff;
    struct os_event rxEvt;
    struct os_event txEvt;
    struct os_callout rxtimer;
    struct os_callout txtimer;
} _cfgs[MAX_NB_L96];                // TODO use mempools
static int _nbL96Cfgs=0;

// predefine private functions
static void l96_comm_task(void* arg);
static int L96_I2C_open(wskt_t* skt);
static int L96_I2C_ioctl(wskt_t* skt, wskt_ioctl_t* cmd);
static int L96_I2C_write(wskt_t* skt, uint8_t* data, uint32_t sz);
static int L96_I2C_close(wskt_t* skt);
//static int addRxByte(struct L96DeviceCfg* myCfg, uint8_t c);
static void i2c_rx_cb(struct os_event* e);
static void i2c_tx_cb(struct os_event* e);

static wskt_devicefns_t _myDevice = {
    .open = &L96_I2C_open,
    .ioctl = &L96_I2C_ioctl,
    .write = &L96_I2C_write,
    .close = &L96_I2C_close
};


static uint8_t _rxLineBuffer[L96_LINE_SZ];
static uint8_t _i2cLineBuffer[L96_LINE_SZ];
// mutex to protect it (only used in passing)
static struct os_mutex _lbRXMutex;
static struct os_mutex _lbI2CMutex;
static struct os_eventq _l96eventQ;


// Called from sysinit via reference in pkg.yml
void L96_I2C_comm_init(void) {
    // TODO should we use mempools to handle per-device structures?
        // Create eventQ
    os_eventq_init(&_l96eventQ);
    os_mutex_init(&_lbRXMutex);
    os_mutex_init(&_lbI2CMutex);
        // Create the comm handler task
    os_task_init(&_l96_task_str, "l96_task", l96_comm_task, NULL, L96COMM_TASK_PRIO,
                 OS_WAIT_FOREVER, _l96_task_stack, L96COMM_TASK_STACK_SZ);
}

// Called from initialisation steps to create each I2C channel that goes to a L96
bool L96_I2C_comm_create(char* dname, const char* i2cname, uint8_t i2caddr, int i2cpowerGPIO) {
    // allocate new device cfg element
    if (_nbL96Cfgs>=MAX_NB_L96) {
        //log("too many L96 creates");
        return false;
    }
    struct L96DeviceCfg* myCfg = &_cfgs[_nbL96Cfgs++];
    myCfg->active=false;        // no active sockets yet
#if MYNEWT_VAL(USE_BUS_I2C)
    myCfg->i2cCfg.node_cfg.bus_name=i2cname;
    myCfg->i2cCfg.node_cfg.lock_timeout_ms=0;
    myCfg->i2cCfg.addr = i2caddr;
//    myCfg->i2cCfg.freq = ?;
    myCfg->i2cCfg.quirks = 0;
#else
    myCfg->i2cDev = i2cname[strlen(i2cname)-1] - '0';       // clunky
    myCfg->i2cAddr = i2caddr;
#endif  /* USE_BUS_I2C */
    circ_bbuf_init(&myCfg->rxBuff, &(myCfg->rxBuff_data_space[0]), L96_LINE_SZ+1);
    circ_bbuf_init(&myCfg->txBuff, &(myCfg->txBuff_data_space[0]), L96_LINE_SZ+1);
    myCfg->rxEvt.ev_cb = i2c_rx_cb;
    myCfg->rxEvt.ev_arg = myCfg;
    myCfg->txEvt.ev_cb = i2c_tx_cb;
    myCfg->txEvt.ev_arg = myCfg;

    // timers for this device
    os_callout_init(&(myCfg->rxtimer), &_l96eventQ,
                    i2c_rx_cb, myCfg);
    os_callout_init(&(myCfg->txtimer), &_l96eventQ,
                    i2c_tx_cb, myCfg);
    // setup I2C to L96
#if MYNEWT_VAL(USE_BUS_I2C)
    // Create L96 as I2C node, and probe to make sure it exists
    int rc = bus_i2c_node_create(dname, &(myCfg->i2cDev), &(myCfg->i2cCfg), &(myCfg->i2cCfg.node_cfg));
#else
    // I2C controller configured in bsp
    int rc = hal_i2c_master_probe(myCfg->i2cDev, myCfg->i2cAddr, 10);
#endif /* USE_BUS_I2C */
    if (rc==0) {
        // and register ourselves as a 'uart like' comms provider so procesing routines can read the data
        wskt_registerDevice(dname, &_myDevice, myCfg);
        return true;
    } else {
        // No L96 found, soz
        return false;
    }
}

static bool openI2C(struct L96DeviceCfg* cfg) {
    // via hal or the drivers layer?
#if MYNEWT_VAL(USE_BUS_I2C)
    // Not required for bus 
    return true;
#else
    // or hal? 
    return true;
#endif  /* USE_BUS_I2C */
}
static int L96_I2C_open(wskt_t* skt) {
    // skt is device config (shared amongst all who open same device) + per socket event/eventq to notify
    struct L96DeviceCfg* cfg=((struct L96DeviceCfg*)WSKT_DEVICE_CFG(skt));  
    // Note the event arg must point to the apps own buffer to copy line into
    // if 1st skt opened on this device, mae sure its powered on and open it
    if (!cfg->active) {
        // setup I2C by opening mynewt device
        if (!openI2C(cfg)) {
            log_noout("open I2C fail");
            return SKT_NODEV;
        }
        cfg->active=true;
        log_noout("open I2C ok");
        // tell task to start reading I2C
        os_eventq_put(&_l96eventQ, &(cfg->rxEvt));
    }
    // Not much to do, io already running
    return SKT_NOERR;
}
static int L96_I2C_ioctl(wskt_t* skt, wskt_ioctl_t* cmd) {
    struct L96DeviceCfg* cfg=((struct L96DeviceCfg*)WSKT_DEVICE_CFG(skt));  
    switch (cmd->cmd) {
        case IOCTL_RESET: {
            if (!openI2C(cfg)) {
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
static int L96_I2C_write(wskt_t* skt, uint8_t* data, uint32_t sz) {
    struct L96DeviceCfg* cfg=((struct L96DeviceCfg*)WSKT_DEVICE_CFG(skt));  

    if (!cfg->active) {
        log_noout("can't write as no I2C dev..");
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

    // Tell task to try more tx data if not already on it
    os_eventq_put(&_l96eventQ, &(cfg->txEvt));
    return SKT_NOERR; 
}
static int L96_I2C_close(wskt_t* skt) {
    struct L96DeviceCfg* cfg=((struct L96DeviceCfg*)WSKT_DEVICE_CFG(skt));  

    // Iff last skt then power down
    if (wskt_getOpenSockets(cfg->dname, NULL, 0)<=1) {
        cfg->active=false;
        // clean buffers
        circ_bbuf_init(&cfg->rxBuff, &(cfg->rxBuff_data_space[0]), L96_LINE_SZ+1);
        circ_bbuf_init(&cfg->txBuff, &(cfg->txBuff_data_space[0]), L96_LINE_SZ+1);
        log_noout("closed last socket on L96 I2C %s", cfg->dname);
    }
    return SKT_NOERR; 
}


// need a task to do the I2C read/writing

static void l96_comm_task(void* arg) {
    while(1) {
        os_eventq_run(&_l96eventQ);
    }
}

static int addRxByte(struct L96DeviceCfg* myCfg, uint8_t c) {
        // Add to line in circ buffer
    circ_bbuf_push(&(myCfg->rxBuff), c);
    // if full or CR, copy to all sockets (get list from wskt mgr)
    if (c=='\n' || circ_bbuf_free_space(&(myCfg->rxBuff))==0) {
        // Send event to be processed by task? or just do it here?
        // copy out line first to local STATIC buffer (stack space!)
        // MUTEX
        os_mutex_pend(&_lbRXMutex, OS_TIMEOUT_NEVER);
        uint8_t lineLen = 0;
        bool copied = false;
        while(!copied && lineLen<L96_LINE_SZ) {
            if (circ_bbuf_pop(&(myCfg->rxBuff), &(_rxLineBuffer[lineLen]))<0) {
                // done, its empty
                _rxLineBuffer[lineLen] = '\n';
                copied = true;
            } else {
                // did we just copy a CR?
                if (_rxLineBuffer[lineLen]=='\n') {
                    // done, EOL
                    copied = true;                
                }
            }
            lineLen++;
        }
        // Make it a null terminated string
        _rxLineBuffer[lineLen++] = '\0';
        os_mutex_release(&_lbRXMutex);
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
                    memcpy(_rxLineBuffer, sbuf, lineLen);
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


// run rx on I2C
static void i2c_rx_cb(struct os_event* e) {
    // device context is pointed to by the arg
    struct L96DeviceCfg* cfg = (struct L96DeviceCfg*)(e->ev_arg);
    // MUTEX
    os_mutex_pend(&_lbI2CMutex, OS_TIMEOUT_NEVER);
    // read a buffer ito _i2cLineBuffer
#if MYNEWT_VAL(USE_BUS_I2C)
    int rc = bus_node_simple_read((struct os_dev*)&(cfg->i2cDev), _i2cLineBuffer, L96_LINE_SZ);
#else
    struct hal_i2c_master_data mdata = {
        .address = cfg->i2cAddr,
        .buffer = _i2cLineBuffer,
        .len = L96_LINE_SZ,
    };
    int rc = hal_i2c_master_read(cfg->i2cDev, &mdata, I2C_ACCESS_TIMEOUT, 1);
#endif /* USE_BUS_I2C */
    if (rc==0) {
        // and call addChar for each non-0x0A char in it
        for(int i=0;i<L96_LINE_SZ;i++) {
            if (_i2cLineBuffer[i]!=0x0a) {
                addRxByte(cfg, _i2cLineBuffer[i]);
            }        
        }
    } else {
        log_warn("badness reading I2C for L96 %s : %d",cfg->dname, rc);
    }
    // and release
    os_mutex_release(&_lbI2CMutex);
    // add callout timer for in 500ms time to do rx again
    os_callout_reset(&(cfg->rxtimer), OS_TICKS_PER_SEC/2);

}

// run tx on I2C
static void i2c_tx_cb(struct os_event* e) {
    // device context is pointed to by the arg
    struct L96DeviceCfg* cfg = (struct L96DeviceCfg*)(e->ev_arg);
    // Stop tx timer if running
    os_callout_stop(&(cfg->txtimer));

    // anything to send in circular buffer?
    if (circ_bbuf_free_space(&(cfg->txBuff))>0) {
        uint8_t c;
        uint8_t lineLen = 0;
        // MUTEX
        os_mutex_pend(&_lbI2CMutex, OS_TIMEOUT_NEVER);
        while (circ_bbuf_pop(&(cfg->txBuff), &c)==0 && c!='\n' && lineLen<L96_LINE_SZ) {
            _i2cLineBuffer[lineLen++] = c;
        }
        _i2cLineBuffer[lineLen++] = '\n';
        // I2C write the buffer
#if MYNEWT_VAL(USE_BUS_I2C)
        int rc = bus_node_simple_write((struct os_dev*)&(cfg->i2cDev), _i2cLineBuffer, lineLen);
#else
        struct hal_i2c_master_data mdata = {
            .address = cfg->i2cAddr,
            .buffer = _i2cLineBuffer,
            .len = L96_LINE_SZ,
        };
        int rc = hal_i2c_master_write(cfg->i2cDev, &mdata, I2C_ACCESS_TIMEOUT, 1);
#endif  /* USE_BUS_I2C */
        if (rc!=0) {
            log_warn("badness writing I2C for L96 %s : %d",cfg->dname, rc);
        }
        // and release
        os_mutex_release(&_lbI2CMutex);
        // and come back in a few ms to see if anything else to do
        os_callout_reset(&(cfg->txtimer), OS_TICKS_PER_SEC/10);
    }
    // tx empty, can wait for a write to kick us
}

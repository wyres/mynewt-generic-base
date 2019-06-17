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

#include <string.h>

#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_spi.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/configmgr.h"
#include "wyres-generic/loraapp.h"
#include "lorawan_api/lorawan_api.h"

#define LORAAPP_TASK_PRIO       MYNEWT_VAL(LORAAPP_TASK_PRIO)
#define LORAAPP_TASK_STACK_SZ   OS_STACK_ALIGN(256)

static os_stack_t _loraapp_task_stack[LORAAPP_TASK_STACK_SZ];
static struct os_task _loraapp_task_str;
static struct os_sem _lora_tx_sem;

static lorawan_sock_t _sock_tx=0;
static lorawan_sock_t _sock_rx=0;
static bool _canTx = false;

static struct loraapp_config {
     bool useAck;
     bool useAdr;
     uint8_t txPort;
     uint8_t rxPort;
     uint8_t loraDR;
     int8_t txPower;
     uint32_t txTimeoutMs;
     uint8_t deveui[8];
     uint8_t appeui[8];
     uint8_t appkey[16];
     uint32_t devAddr;
     uint8_t nwkSkey[16];
     uint8_t appSkey[16];
} _loraCfg = {
    .useAck = false,
    .useAdr = false,
    .txPort=3,
    .rxPort=3,
    .loraDR=0,
    .txPower=14,
    .txTimeoutMs=10000,
//    .deveui = {0x38,0xE8,0xEB,0xE0, 0x00, 0x00, 0x0d, 0x78},
    .deveui = {0x38,0xb8,0xeb,0xe0, 0x00, 0x00, 0x0d, 0x73},
    .appeui = {0x01,0x23,0x45,0x67, 0x89, 0xab, 0xcd, 0xef},
//    .appeui = {0x38,0xE8,0xEB,0xEA, 0xBC, 0xDE, 0xF0, 0x42},
    .appkey = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF},
    .devAddr=0,
    .nwkSkey = {0},
    .appSkey = {0},
};
static LORA_RES_CB_FN_t _txcbfn=NULL;
static LORA_RX_CB_FN_t _rxcbfn=NULL;

static void loraapp_task(void* data);

static lorawan_region_t getCfgdRegion() {
    if (MYNEWT_VAL(LORAWAN_REGION_EU868)) {
        return LORAWAN_REGION_EU868;
    }
    if (MYNEWT_VAL(LORAWAN_REGION_IN865)) {
        return LORAWAN_REGION_IN865;
    }
//    if (MYNEWT_VAL(LORAWAN_REGION_RU864)) {
//        return LORAWAN_REGION_RU864;
//    }
    // By default we are EU
    return LORAWAN_REGION_EU868;
}

/*
static bool spi0_read_buffer(uint8_t addr, uint8_t* buffer, uint8_t sz) {
    uint8_t spiId = 0;
    uint8_t spiNss = SPI_0_MASTER_PIN_NSS;
    int ret=0;
    // Config SPI
    struct hal_spi_settings spi0_setting = {
        .data_order = HAL_SPI_MSB_FIRST,
        .data_mode = HAL_SPI_MODE0,
        .baudrate = 2000,
        .word_size = HAL_SPI_WORD_SIZE_8BIT,
    };

    if ((ret = hal_spi_disable(spiId))!=0) {
        log_debug("spi disable fails %d",ret);
    }
    if ((ret=hal_spi_config(spiId, &spi0_setting))!=0) {
        log_debug("spi config fails %d",ret);
    }
    if ((ret = hal_spi_enable(spiId))!=0) {
        log_debug("spi enable fails %d",ret);
    }
    // Read chip id from reg 0x42
    //NSS = 0;
    hal_gpio_init_out(spiNss, 0 );
    hal_gpio_write(spiNss, 0);

    // Write the reg address
    hal_spi_tx_val(spiId, addr & 0x7F);

    // read the data
    for(int i = 0; i < sz; i++ ) {
        buffer[i] = hal_spi_tx_val(spiId, 0);
    }

    //NSS = 1;
    hal_gpio_write(spiNss, 1);
    return true;
}
static bool checkRadio() {
    uint8_t id=0;
    if (!spi0_read_buffer(0x42, &id, 1)) {
        return false;
    }
    if (id == 0x22) {
        return true;
    }
    log_debug("radio id read returns unexpected id %d", id);
    return false;
}
*/

// Must call these BEFORE doing lora_init()
void lora_app_setDevEUI(uint8_t* d) {
    for(int i=0;i<8;i++) {
        _loraCfg.deveui[i] = d[i];
    }
    CFMgr_setElement(CFG_UTIL_KEY_DEVEUI, &_loraCfg.deveui, 8);
}
void lora_app_setAppEUI(uint8_t* d) {
    for(int i=0;i<8;i++) {
        _loraCfg.appeui[i] = d[i];
    }
    CFMgr_setElement(CFG_UTIL_KEY_APPEUI, &_loraCfg.appeui, 8);
}
void lora_app_setAppKey(uint8_t* d) {
    for(int i=0;i<16;i++) {
        _loraCfg.appkey[i] = d[i];
    }
    CFMgr_setElement(CFG_UTIL_KEY_APPKEY, &_loraCfg.appkey, 16);
}

void lora_app_setAck(bool useAck) {
    _loraCfg.useAck = useAck;
    CFMgr_setElement(CFG_UTIL_KEY_ACKEN, &_loraCfg.useAck, sizeof(bool));
    if (_sock_tx > 0) {
        lorawan_attribute_t mib;
        mib.Type = LORAWAN_ATTR_MCPS_TYPE;
        if (_loraCfg.useAck) {
            mib.Param.McpsType = LORAWAN_MCPS_CONFIRMED;
        } else {
            mib.Param.McpsType = LORAWAN_MCPS_UNCONFIRMED;
        }
        if (lorawan_setsockopt(_sock_tx, &mib) != LORAWAN_STATUS_OK) {
            log_warn("failed to set ACK");
        }
    }
}
void lora_app_setAdr(bool useAdr) {
    _loraCfg.useAdr = useAdr;
    CFMgr_setElement(CFG_UTIL_KEY_ADREN, &_loraCfg.useAdr, sizeof(bool));
    if (_sock_tx > 0) {
        lorawan_attribute_t mib;
        mib.Type = LORAWAN_ATTR_ADR;
        mib.Param.AdrEnable = _loraCfg.useAdr;
        if (lorawan_setsockopt(_sock_tx, &mib) != LORAWAN_STATUS_OK) {
            log_warn("failed to set ADR");
        }
    }
}
void lora_app_setTxPort(uint8_t p) {
    _loraCfg.txPort = p;
    CFMgr_setElement(CFG_UTIL_KEY_TXPORT, &_loraCfg.txPort, sizeof(uint8_t));
}
void lora_app_setRxPort(uint8_t p) {
    _loraCfg.rxPort = p;
    CFMgr_setElement(CFG_UTIL_KEY_RXPORT, &_loraCfg.rxPort, sizeof(uint8_t));
    if (_sock_rx > 0) {
        assert(lorawan_bind(_sock_rx, 0, p)==0);
    }
}
void lora_app_setTxPower(int8_t p) {
    _loraCfg.txPower = p;
    CFMgr_setElement(CFG_UTIL_KEY_TXPOWER, &_loraCfg.txPower, sizeof(int8_t));
    if (_sock_tx > 0 ) {
        lorawan_attribute_t mib;
        mib.Type = LORAWAN_ATTR_CHANNELS_DEFAULT_TX_POWER;
        mib.Param.ChannelsDefaultTxPower = ((14-_loraCfg.txPower)/2);
        if (lorawan_setsockopt(_sock_tx, &mib) != LORAWAN_STATUS_OK) {
            log_warn("failed to set tx power");
        }
    }
}
void lora_app_setDR(int8_t d) {
    _loraCfg.loraDR = d;
    CFMgr_setElement(CFG_UTIL_KEY_DR, &_loraCfg.loraDR, sizeof(int8_t));
    if (_sock_tx > 0 ) {
        lorawan_attribute_t mib;
        mib.Type = LORAWAN_ATTR_CHANNELS_DEFAULT_DATARATE;
        mib.Param.ChannelsDefaultDatarate = _loraCfg.loraDR;
        if (lorawan_setsockopt(_sock_tx, &mib) != LORAWAN_STATUS_OK) {
            log_warn("failed to set data rate");
        }
    }
}

static bool configSocket(lorawan_sock_t skt) {
    bool ret = true;
    lorawan_status_t status;
    lorawan_attribute_t mib;
    mib.Type = LORAWAN_ATTR_ADR;
    mib.Param.AdrEnable = _loraCfg.useAdr;
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("failed to set ADR");
        ret = false;
    }
    mib.Type = LORAWAN_ATTR_MCPS_TYPE;
    if (_loraCfg.useAck) {
        mib.Param.McpsType = LORAWAN_MCPS_CONFIRMED;
    } else {
        mib.Param.McpsType = LORAWAN_MCPS_UNCONFIRMED;
    }
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("failed to set ACK");
        ret = false;
    }
    mib.Type = LORAWAN_ATTR_CHANNELS_DATARATE;
    mib.Param.ChannelsDefaultDatarate = _loraCfg.loraDR;
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("failed to set data rate");
        ret = false;
    }
    mib.Type = LORAWAN_ATTR_CHANNELS_TX_POWER;
    mib.Param.ChannelsDefaultTxPower = ((14-_loraCfg.txPower)/2);        // lorawan stack power does 0-7, no one knows why
    status = lorawan_setsockopt(skt, &mib);
    if (status != LORAWAN_STATUS_OK) {
        log_warn("failed to set tx power");
        ret = false;
    }
    return ret;
}

// initialise lorawan stack with our config
void lora_app_init( LORA_RES_CB_FN_t txcb, LORA_RX_CB_FN_t rxcb) {
/*
    // Check SX1272 is alive
    if (checkRadio()) {
        log_debug("radio check says its good");
    } else {
        log_warn("radio check fails for spi0...");
    }
    */
    // get devEUI/appEUI/appKey/devAddr/nbRep/txPower/default DR/ack/ADR etc from config
    CFMgr_getOrAddElement(CFG_UTIL_KEY_DEVEUI, &_loraCfg.deveui, 8);
    CFMgr_getOrAddElement(CFG_UTIL_KEY_APPEUI, &_loraCfg.appeui, 8);
    CFMgr_getOrAddElement(CFG_UTIL_KEY_APPKEY, &_loraCfg.appkey, 16);
    CFMgr_getOrAddElement(CFG_UTIL_KEY_DEVADDR, &_loraCfg.devAddr, sizeof(uint32_t));
    CFMgr_getOrAddElement(CFG_UTIL_KEY_NWKSKEY, &_loraCfg.nwkSkey, 16);
    CFMgr_getOrAddElement(CFG_UTIL_KEY_APPSKEY, &_loraCfg.appSkey, 16);
    CFMgr_getOrAddElement(CFG_UTIL_KEY_ADREN, &_loraCfg.useAdr, sizeof(bool));
    CFMgr_getOrAddElement(CFG_UTIL_KEY_ACKEN, &_loraCfg.useAck, sizeof(bool));
    CFMgr_getOrAddElement(CFG_UTIL_KEY_DR, &_loraCfg.loraDR, sizeof(uint8_t));
    CFMgr_getOrAddElement(CFG_UTIL_KEY_TXPOWER, &_loraCfg.txPower, sizeof(int8_t));
    CFMgr_getOrAddElement(CFG_UTIL_KEY_TXPORT, &_loraCfg.txPower, sizeof(uint8_t));
    CFMgr_getOrAddElement(CFG_UTIL_KEY_RXPORT, &_loraCfg.txPower, sizeof(uint8_t));

    uint8_t nb_rep = 1;

    // check if ABP keys available
    // if so, go ABP mode else go OTAA mode
    if (_loraCfg.devAddr==0) {
        int status = lorawan_configure_OTAA(_loraCfg.deveui, _loraCfg.appeui, _loraCfg.appkey, nb_rep, ((14-_loraCfg.txPower)/2), getCfgdRegion());
        assert(status == LORAWAN_STATUS_OK);
        log_debug("Starting LoRaWAN in OTAA mode [with devEUI: %02x%02x%02x%02x%02x%02x%02x%02x]",
            _loraCfg.deveui[0],_loraCfg.deveui[1],_loraCfg.deveui[2],_loraCfg.deveui[3],_loraCfg.deveui[4],_loraCfg.deveui[5],_loraCfg.deveui[6],_loraCfg.deveui[7]);
    } else {
        int status = lorawan_configure_ABP(_loraCfg.devAddr, _loraCfg.nwkSkey, _loraCfg.appSkey, nb_rep, ((14-_loraCfg.txPower)/2), getCfgdRegion());
        assert(status == LORAWAN_STATUS_OK);
        log_debug("Starting LoRaWAN in ABP mode [with devAddr:%08lx]", _loraCfg.devAddr);
    }

    // TODO this is not reglo
    lorawan_set_dutycycle(false);
        // set cbfn, indcating lock of lora
    _txcbfn = txcb;
    _rxcbfn = rxcb;

    if (_txcbfn!=NULL) {
        /* 1st action: obtain a socket from the LoRaWAN API */
        _sock_tx = lorawan_socket(SOCKET_TYPE_TX);
        assert(_sock_tx > 0);
        configSocket(_sock_tx);
        _canTx = true;
    }
    if (_rxcbfn!=NULL) {
        /* 1st action: obtain a socket from the LoRaWAN API */
        _sock_rx = lorawan_socket(SOCKET_TYPE_RX);
        assert(_sock_rx > 0);
        // Doesn't need same config as for TX, but does need to be bound
        assert(lorawan_bind(_sock_rx, 0, _loraCfg.rxPort)==0);

    }
    // need a semaphore...
    os_sem_init(&_lora_tx_sem,0);
    // Create task to run TX/RX as KLK wrapper uses blocking calls... thanks guys...
    os_task_init(&_loraapp_task_str, "lw_eventq",
                 loraapp_task, NULL,
                 LORAAPP_TASK_PRIO, OS_WAIT_FOREVER,
                 _loraapp_task_stack,
                 LORAAPP_TASK_STACK_SZ);


}

// Use static buffer in case api doesn't copy it
static uint8_t _txBuffer[255]={0};
// tx a buffer. returns result enum indicating tx ok, ack rx, or error
LORA_TX_RESULT_t lora_app_tx(uint8_t* data, uint8_t sz, uint32_t timeoutMs) {
    assert(_sock_tx>0);        // no txing if you didnt init for it
    if (!_canTx) {
        return LORA_TX_ERR_RETRY;
    }
    _loraCfg.txTimeoutMs = timeoutMs;
    memcpy(_txBuffer, data, sz);
    /* put the data into the queue, the message will be sent
     * when the LoRaWAN stack is ready. 
     **/
    int ret = lorawan_send(_sock_tx, _loraCfg.txPort, _txBuffer, sz);
    switch(ret) {
        case LORAWAN_STATUS_OK: {
            log_debug("LoRaWAN API tx queued %d bytes ok on port:%d dr:%d txpower:%d ackReq:%d]\r\n",
                sz, _loraCfg.txPort, 
                _loraCfg.loraDR, _loraCfg.txPower, _loraCfg.useAck );
            // release the task to wait for result 
            os_sem_release(&_lora_tx_sem);
            return LORA_TX_OK;
        }
        case LORAWAN_STATUS_PORT_BUSY: {
            log_debug("LoRaWAN API tx has busy return code.");
            return LORA_TX_ERR_RETRY;
        }
        default: {
            log_warn("LoRaWAN API tx has fatal error code (%d).",
                ret);
            return LORA_TX_ERR_FATAL;       // best you reset mate
        }
    }
}

static void loraapp_task(void* data) {
    while(1) {
        if (_sock_tx>0) {
            // TODO should have sema to stop app trying a tx before we're ready to go back to listen for tx result...
            // but without blocking the tx caller.... just a flag?
            _canTx = true;
            // Wait till we tx something
            os_sem_pend(&_lora_tx_sem, OS_TIMEOUT_NEVER);
            _canTx = false;
            lorawan_event_t txev = lorawan_wait_ev(_sock_tx, (LORAWAN_EVENT_ACK|LORAWAN_EVENT_ERROR|LORAWAN_EVENT_SENT), _loraCfg.txTimeoutMs);
            // note sema is now taken (==0), so next time round will block until a tx call adds a token
            assert(_txcbfn!=NULL);      // must have a cb fn if we created the socket...
            if (txev & LORAWAN_EVENT_ACK) {
                log_debug("tx ev OK ACK");
                (*_txcbfn)(LORA_TX_OK_ACKD);
            }
            if (txev & LORAWAN_EVENT_SENT) {
                log_debug("tx ev OK NOACK");
                (*_txcbfn)(LORA_TX_OK);
            }
            if (txev & LORAWAN_EVENT_ERROR) {
                // Check if we are joined
                if (lora_app_isJoined()) {
                    log_debug("tx ev ERR (joined)");
                    (*_txcbfn)(LORA_TX_ERR_RETRY);
                } else {
                    // Not joined this is why an error
                    log_debug("tx ev ERR (NOT joined)");
                    (*_txcbfn)(LORA_TX_ERR_NOTJOIN);

                }
            }
            if (txev==0) {
                // timeout
                log_debug("tx ev TIMEOUT");
                (*_txcbfn)(LORA_TX_TIMEOUT);
            }
        }
        // TX done. try for an RX while we're here... 
        if (_sock_rx>0) {
            uint32_t devAddr;
            uint8_t port;
            uint8_t payload[256];
            assert(_rxcbfn!=NULL);      // Must have cb fn if created socket
            // Wait 10s as gotta wait for RX2 delay (up to 7s) + SF12 (2s)
            uint8_t rxsz = lorawan_recv(_sock_rx, &devAddr, &port, payload, 255, 9000);
            log_debug("lora rx says got [%d] bytes", rxsz);
            if (rxsz>0) {
                (*_rxcbfn)(port, payload, rxsz);
            }
        }
        // If no socket open, then just wait
        if (!(_sock_tx>0 || _sock_rx>0)) {
            os_time_delay(OS_TICKS_PER_SEC*60);
        }
    }
}

// TODO force join independantly of tx?
bool lora_app_join() {
    return false;
}

// return join status
bool lora_app_isJoined() {
    if(_sock_tx>0) {
        lorawan_attribute_t mib;
        mib.Type = LORAWAN_ATTR_NETWORK_JOINED;
        if (lorawan_getsockopt(_sock_tx, &mib) == LORAWAN_STATUS_OK) {
            return mib.Param.IsNetworkJoined;
        } else {
            log_warn("failed to get join status");
        }
    }
    return false;       // as far as we know
}

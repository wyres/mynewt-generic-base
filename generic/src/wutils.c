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

#include <stdio.h>
#include <stdarg.h>

#include "os/os.h"
#include "bsp.h"
#include "console/console.h"
#include "hal/hal_gpio.h"
#include "hal/hal_uart.h"
//#include <execinfo.h>

#include "wyres-generic/wutils.h"
#include "wyres-generic/wskt_user.h"
#include "wyres-generic/uartlinemgr.h"
#include "wyres-generic/uartselector.h"
#include "wyres-generic/rebootmgr.h"
#include "wyres-generic/timemgr.h"

// TODO handle versioning
uint8_t _fver = 0;

// assert gets stack to determine address of line that called it to dump in log
// also write stack into prom for reboot analysis
/**
 * replaces system assert with a more useful one
 * Note we don't generally use the file/line info as this increases the binary size too much (many strings)
 */
void wassert_fn(const char* file, int lnum) {
    // see https://gcc.gnu.org/onlinedocs/gcc/Return-Address.html
    void* assert_caller = 0;
    assert_caller = __builtin_extract_return_addr(__builtin_return_address(0));
    log_debug("assert from [%8x] see APP.elf.lst", assert_caller);
    RMMgr_saveAssertCaller(assert_caller);
    // flash led
    hal_gpio_init_out(LED_1, 1);
    hal_gpio_init_out(LED_2, 0);
    // if ebugging,  halt busy flashing leds
    // else reboot
    uint32_t et = TMMgr_getRelTime();
    while(1) {
        int total = 0;
        // busy wait - don't let OS do anything or run another task
        for(int i=0; i<1000000;i++) {
            // do something to avoid being optimised out
            total++;
        }
        hal_gpio_toggle(LED_1);        
        hal_gpio_toggle(LED_2);        
        // reboot after 30s
        if ((TMMgr_getRelTime()-et)>30000) {
            RMMgr_reboot(RM_ASSERT);
        }
    }
}

// This is the assert fn mapped to by OS_CRASH() in os_fault.h which is mapped by the system assert.h in mynewt
void wassert_fn_cb(const char* file, int lnum, const char *func, const char *e) {
    wassert_fn(file, lnum);
}

void os_assert_cb() {
    wassert_fn(NULL, 0);
}
/* can't do this... this is defined in mynewt kernel/os/src/arch/cortex_m3/os_fault.c
void __assert_func(const char *file, int line, const char *func, const char *e) {
    wassert_fn(NULL,line);
    // actually wassert_fn never returns
    exit(-1);
}
*/
// Override/callback from reboot.
void wreboot_cb(void) {
    RMMgr_reboot(RM_ASSERT);
}

#define MAX_LOGSZ 256
    // Default log level depending on build (can be changed by app)
#ifdef NDEBUG
static uint8_t _logLevel = LOGS_RUN;
#else /* NDEBUG */
static uint8_t _logLevel = LOGS_DEBUG;
#endif /* NDEBUG */

// Must be static buffer NOT ON STACK
static char _buf[MAX_LOGSZ];
static char _noutbuf[MAX_LOGSZ];        // for nout log

// By default use mynewt console, not some random uart
static bool _useConsole = true;
static int _uartNb = -1;
static int8_t _uartSelect = -1;
static wskt_t* _uartSkt = NULL;

// note the doout is to allow to break here in debugger and see the log, without actually accessing UART
static void do_log(char lev, const char* ls, va_list vl) {
    // protect here with a mutex?
    _buf[0]=lev;
    _buf[1]=':';
    // need timestamp
    vsprintf(_buf+2, ls, vl);
    int len = strnlen(_buf, MAX_LOGSZ);
    if ((len+3)>=MAX_LOGSZ) {
        // oops might just have broken stuff...
        _buf[MAX_LOGSZ-1] = '\0';
    } else {
        _buf[len]='\n';
        _buf[len+1]='\r';
        _buf[len+2]='\0';
        len+=3;
    }
    // send to mynewt logger if enabled
    if (_useConsole) {
        console_write(_buf, len);
    }
    if (_uartNb>=0) {
        // blocking write to uart?
        for(int i=0;i<len;i++) {
            hal_uart_blocking_tx(_uartNb, _buf[i]);
        }
    }
   if (_uartSkt!=NULL) {
        // select it on uart switcher..
        uart_select(_uartSelect);

        int res = wskt_write(_uartSkt, (uint8_t*)_buf, len);
        if (res<0) {
            _buf[0] = '*';
            wskt_write(_uartSkt, (uint8_t*)_buf, 1);      // so user knows he missed something.
            // Not actually a lot we can do about this especially if its a flow control (SKT_NOSPACE) condition - ignore it
           log_noout_fn("log FAIL[%s]", _buf);      // just for debugger to watch
       }
   }
}

void log_level(uint8_t l) {
    _logLevel = l;
}

void log_init_console(bool enable) {
    _useConsole = enable;
}
// Enable direct hal uart writing for low level debug
void log_init_dbg(uint8_t u) {
    _uartNb = u;
}
int log_init_uart(const char* dev, uint32_t baud, int8_t uartSelect) {
    // allow re-init
    if (_uartSkt!=NULL) {
        wskt_close(&_uartSkt);
    }
    _uartSelect = uartSelect;
    // open it and configure
    _uartSkt = wskt_open(dev, NULL, NULL);
    if (_uartSkt==NULL) {
        return -1;
    }
    wskt_ioctl_t cmd = {
        .cmd = IOCTL_SET_BAUD,
        .param = baud,
    };
    return wskt_ioctl(_uartSkt, &cmd);
    /*
    // get uart device number from end of device name
    _uartNb = '0'-dev[strlen(dev)-1];
    int rc = hal_uart_config(_uartNb,
        baud,
        8,
        1,
        HAL_UART_PARITY_NONE,
        HAL_UART_FLOW_CTL_NONE
    );
    return rc;
    */
}

void log_debug_fn(const char* ls, ...) {
    if (_logLevel<=LOGS_DEBUG) {
        va_list vl;
        va_start(vl, ls);
        do_log('D', ls, vl);
        va_end(vl);
    }
}
void log_info_fn(const char* ls, ...) {
    if (_logLevel<=LOGS_INFO) {
        va_list vl;
        va_start(vl, ls);
        do_log('I', ls, vl);
        va_end(vl);
    }
}
void log_warn_fn(const char* ls, ...) {
    if (_logLevel<=LOGS_RUN) {
        va_list vl;
        va_start(vl, ls);
        do_log('W', ls, vl);
        va_end(vl);
    }
}
void log_error_fn(const char* ls, ...) {
    if (_logLevel<=LOGS_RUN) {
        va_list vl;
        va_start(vl, ls);
        do_log('E', ls, vl);
        va_end(vl);
    }
}
void log_noout_fn(const char* ls, ...) {
    va_list vl;
    va_start(vl, ls);
    vsprintf(_noutbuf, ls, vl);
    // watch _noutbuf to see the log
    int l = strlen(_noutbuf);
    _noutbuf[l++] = '\n';
    _noutbuf[l++] = '\r';
    _noutbuf[l++] = '\0';
    va_end(vl);
}
void log_blocking_fn(int u, const char* ls, ...) {
    va_list vl;
    va_start(vl, ls);
    vsprintf(_noutbuf, ls, vl);
    int l = strlen(_noutbuf);
    // blocking write to uart 
    for(int i=0;i<l;i++) {
        hal_uart_blocking_tx(u, _noutbuf[i]);
    }
    hal_uart_blocking_tx(u, '\n');
    hal_uart_blocking_tx(u, '\r');
    va_end(vl);
}

bool unittest(const char* tn, bool res) {
    if (!res) {
        log_warn("Unittest[%s] failed", tn);
    }
    return res;
}

// Called from sysinit once uarts etc are up
void wlog_init(void) {
    bool res = true;
    // no console
#if MYNEWT_VAL(CONSOLE_UART) 
    log_init_console(true);
#else
    log_init_console(false);
#endif
    // If logging to a uart is required, tell logging system
#if (MYNEWT_VAL(LOG_UART_ENABLED))
    // If specific device for logging, create its wskt driver driver
    res=uart_line_comm_create(MYNEWT_VAL(LOG_UART), MYNEWT_VAL(LOG_UART_BAUDRATE));
    assert(res);
    // And tell logging to use it
    log_init_uart(MYNEWT_VAL(LOG_UART), MYNEWT_VAL(LOG_UART_BAUDRATE), MYNEWT_VAL(LOG_UART_SELECT));  
#endif
//    log_init_dbg(0);      // dont do blocking tx please
    assert(res);
}
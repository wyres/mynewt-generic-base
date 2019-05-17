/**
 * Wyres private code
 * Time/Date management
 */
#include "os/os.h"
#include "bsp.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/lowpowermgr.h"
#include "wyres-generic/uartSelector.h"

static int8_t _gpio0;
static int8_t _gpio1;
static uint8_t _current;

void uart_selector_init(int8_t gpio0, int8_t gpio1) {
    _gpio0 = gpio0;
    _gpio1 = gpio1;
    if (_gpio0<0 || _gpio1<0) {
        return;
    }
    _current = 0;
    GPIO_define_out("US0", gpio0, 0, LP_DEEPSLEEP);
    GPIO_define_out("US1", gpio1, 0, LP_DEEPSLEEP);
}
uint8_t uart_select(uint8_t id) {
    uint8_t ret = _current;
    if (_gpio0<0 || _gpio1<0) {
        return -1;
    }
    if (id>=0) {
        _current = id;
        GPIO_write(_gpio0, (id & 0x01));
        GPIO_write(_gpio1, (id & 0x02)>1);
    }
    return ret;
}
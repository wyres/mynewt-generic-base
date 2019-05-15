#ifndef H_GPIOMGR_H
#define H_GPIOMGR_H

#include <inttypes.h>
#include <mcu/mcu.h>
#include <hal/hal_gpio.h>
#include "lowpowermgr.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 Manage GPIOs centrally to be able to deal with low power enter/exit
*/
#define GPIO_NAME_SZ    (31)
typedef enum  { GPIO_OUT, GPIO_IN, GPIO_IRQ } GPIO_TYPE;

/**
 *  gpio creation mirrors hal calls with extra info
 */
// Define a gpio OUTPUT pin, with a name, an initial value, and the highest lowpower mode it should be active in
void* GPIO_define_out(const char* name, int8_t pin, uint8_t initialvalue, LP_MODE offmode);
void* GPIO_define_in(const char* name, int8_t pin,  hal_gpio_pull_t pull, LP_MODE offmode);
void* GPIO_define_irq(const char* name, int8_t pin, hal_gpio_irq_handler_t handler, void * arg, hal_gpio_irq_trig_t trig, hal_gpio_pull_t pull, LP_MODE offmode);

/** 
 * mirror calls for all other hal gpio functions, but that deal with low power operations
 */
void GPIO_release(int8_t pin);

void GPIO_irq_enable(int8_t pin);
void GPIO_irq_disable(int8_t pin);

int GPIO_write(int8_t pin, int val);
int GPIO_read(int8_t pin); 
#ifdef __cplusplus
}
#endif

#endif  /* H_GPIOMGR_H */

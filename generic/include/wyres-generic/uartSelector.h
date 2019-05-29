#ifndef H_UARTSELECTOR_H
#define H_UARTSELECTOR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_selector_init(int8_t gpio0, int8_t gpio1);
// Select UART if using uart switch
// id = -1 -> ignored, 0-3 written to gpios if defined
int8_t uart_select(int8_t id);

#ifdef __cplusplus
}
#endif

#endif  /* H_UARTSELECTOR_H */

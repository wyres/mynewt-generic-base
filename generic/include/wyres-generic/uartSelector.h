#ifndef H_UARTSELECTOR_H
#define H_UARTSELECTOR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_selector_init(int8_t gpio0, int8_t gpio1);
uint8_t uart_select(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif  /* H_UARTSELECTOR_H */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "param.h"

void uart0_send_params(const params_t *params);
void uart0_send_text(const char *s);
uint8_t uart0_poll_com_n_gen(void);


#endif /* UART_H */

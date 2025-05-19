/*
 * CFile1.c
 *
 * Created: 15/5/2025 17:31:47
 *  Author: GESTION_LABORATORIO
 */
#include <asf.h>
#include "uart_custom.h"

void uart_puts(Uart *uart, const char *str) {
	while (*str) {
		while (!uart_is_tx_ready(uart));
		uart_write(uart, *str++);
	}
}
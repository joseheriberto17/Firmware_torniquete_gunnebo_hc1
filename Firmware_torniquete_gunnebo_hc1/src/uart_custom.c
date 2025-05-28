/*
 * CFile1.c
 *
 * Created: 15/5/2025 17:31:47
 *  Author: GESTION_LABORATORIO
 */
#include <asf.h>
#include "uart_custom.h"
#include <string.h>

void uart_puts(Uart *uart, const char *str, size_t length)
{
	for (size_t i = 0; i < length; i++) {
		while (!uart_is_tx_ready(uart))
		;
		uart_write(uart, (uint8_t)str[i]);
	}
}
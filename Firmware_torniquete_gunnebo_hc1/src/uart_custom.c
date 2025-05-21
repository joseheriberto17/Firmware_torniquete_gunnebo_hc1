/*
 * CFile1.c
 *
 * Created: 15/5/2025 17:31:47
 *  Author: GESTION_LABORATORIO
 */
#include <asf.h>
#include "uart_custom.h"
#include <string.h>

void uart_puts(Uart *uart, const char *str, size_t length) {

	for (size_t i = 0; i < length ; i++) {
		while (!uart_is_tx_ready(uart));
		uart_write(uart, str[i]);
	}
	memset((void *)str, 0, length);
}

// limpia el buffer rx y reescribe su contenido 
// evitar el desbodamiento del buffer
// finaliza con un caracter de escape 
void uart_gets(Uart *uart, char *str, size_t length) {
	memset(str, 0, length);
	size_t i = 0;
	int status = uart_is_rx_ready(uart);

	//while (!uart_is_rx_ready(uart));

	while (status && i < length - 1) {
		uart_read(uart, (uint8_t *)&str[i++]);
		status = uart_is_rx_ready(uart);
	}
	str[i] = '\0';  // terminación segura (recomendado, no es obligatorio)
	
	uart_puts(UART1,str,i);
}
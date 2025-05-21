/*
 * uart_custom.h
 *
 * Created: 15/5/2025 17:33:10
 *  Author: GESTION_LABORATORIO
 */ 


#ifndef UART_CUSTOM_H_
#define UART_CUSTOM_H_

void uart_puts(Uart *uart, const char *str, size_t length);
void uart_gets(Uart *uart, char *str, size_t length);



#endif /* UART_CUSTOM_H_ */
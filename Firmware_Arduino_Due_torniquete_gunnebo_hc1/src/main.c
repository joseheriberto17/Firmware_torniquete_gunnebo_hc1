/*
 * Autor: jose heriberto
 * Fecha de creaccion:
 * Objetivo: Firmware para interactuar con la tarjeta de un torniquete gunnebo
 *	-hay 3 sensores de  efecto hall disponible, dos para paso preciso y otro para indicar el paso del torniquete en si.
 *	-2 solenoide con entrada a 24 v.
 *
 * Presente: sensar senal de encoder  y verlo directamente en 2 led en modo polling
 *
 * Futuro: hacer lo mismo pero con manejo de  interrupciones.
 *
 * Notas:
 * Arduino Due Digital Pin Mapping (D0�D13)
 *
 * +--------------+----------------+
 * | Arduino PIN  | SAM3X8E Pin    |
 * +--------------+----------------+
 * | D0           | PA8            |
 * | D1           | PA9            |
 * | D2           | PB25           |
 * | D3           | PC28           |
 * | D4           | PC26/PA29      |
 * | D5           | PC25           |
 * | D6           | PC24           |
 * | D7           | PC23           |
 * | D8           | PC22           |
 * | D9           | PC21           |
 * | D10          | PC29/PA28      |
 * | D11          | PD7            |
 * | D12          | PD8            |
 * | D13 (LED)    | PB27           |
 * +--------------+----------------+
 */

#include <asf.h>
#include "Driver/encoder.h"
#include "string.h"
#include <stdbool.h>

void configure_pins(void);
//void configure_uart(void);
//void uart_puts(Uart *uart, const char *str, size_t length);
void configure_systick(void);
uint32_t millis(void);

volatile uint32_t ms_ticks = 0;
bool status_D2_A = false;
bool status_D3_A = false;
bool status_D2_B = false;
bool status_D3_B = false;

volatile u_int8_t counter_1 = 0;
volatile u_int8_t counter_2 = 0;



// salidas solenoide
#define PIN_D3 PIO_PC28_IDX
#define PIN_D3_MASK PIO_PC28
#define PIN_D3_PORT PIOC

#define PIN_D2 PIO_PB25_IDX
#define PIN_D2_MASK PIO_PB25
#define PIN_D2_PORT PIOB

void configure_systick(void) {
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1); // error
    }
}
void SysTick_Handler(void)
{
	ms_ticks++;
}
uint32_t millis(void)
{
	return ms_ticks;
}

void configure_pins(void)
{
	// Habilitar el reloj para el PIOB
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);

	// salida
	pio_configure(PIN_D2_PORT, PIO_OUTPUT_0, PIN_D2_MASK, PIO_DEFAULT);
	pio_configure(PIN_D3_PORT, PIO_OUTPUT_0, PIN_D3_MASK, PIO_DEFAULT);

	// inicializar pines de salida
	pio_set(PIN_D2_PORT, PIN_D2_MASK);
	pio_set(PIN_D3_PORT, PIN_D3_MASK);
}

//void configure_uart(void)
//{
	//pmc_enable_periph_clk(ID_UART);
//
	//pio_configure(PINS_UART_PIO, PINS_UART_TYPE, PINS_UART_MASK, PINS_UART_ATTR);
//
	//sam_uart_opt_t uart1_settings = {
		//.ul_mck = sysclk_get_cpu_hz(),
		//.ul_baudrate = 19200,
		//.ul_mode = UART_MR_PAR_NO // Modo sin paridad, 8N1
	//};
//
	//uart_init(UART, &uart1_settings);
	//uart_enable_interrupt(UART, UART_IER_RXRDY);
	//NVIC_EnableIRQ(UART_IRQn);
//}
//
//void uart_puts(Uart *uart, const char *str, size_t length)
//{
	//for (size_t i = 0; i < length; i++) {
		//while (!uart_is_tx_ready(uart))
		//;
		//uart_write(uart, (uint8_t)str[i]);
	//}
//}

//void UART_Handler(void)
//{
    //uint32_t sr = UART->UART_SR;
//
    //// Limpia errores primero
    //if (sr & (UART_SR_OVRE | UART_SR_FRAME | UART_SR_PARE)) {
        //volatile uint32_t dump = UART->UART_RHR; // drenar
        //(void)dump;
        //UART->UART_CR = UART_CR_RSTSTA;
        //// No retornes aún: puede haber más datos, continúa a drenar
    //}
//
    //// Drenar todos los bytes disponibles
    //while (UART->UART_SR & UART_SR_RXRDY) {
        //uint8_t ch = (uint8_t)UART->UART_RHR;
//
        //if (ch == '1') {
            //counter_1++;
        //} else if (ch == '2') {
            //counter_2++;
        //} else {
            //// Ignora '\r' y '\n' u otros
        //}
    //}
//}


int main(void)
{
	sysclk_init();
	board_init();

	configure_pins();
	//configure_uart();
	encoder_init();
	configure_systick();
	//char uart_buffer[50];
//
	//sprintf(uart_buffer, "Hola desde UART\n");
	//uart_puts(UART,uart_buffer, strlen(uart_buffer));

	uint32_t timer_while = millis();
	
	

	while (1)
	{
		if ((millis() - timer_while) > 200)
		{
			timer_while = millis();
			// sprintf(uart_buffer, "Posicion: %ld index: %d\n", encoder_get_position(), end_pase_get());
			// uart_puts(UART, uart_buffer);
			
			//counter_1++;
			//
			//if (counter_1 > 150)
			//{
				//status_D2_A = !status_D2_A;
				//counter_1 = 0;
			//}
			//
				//if (status_D2_A)
				//{
					//pio_set(PIN_D2_PORT,PIN_D2_MASK);
					//pio_set(PIN_D3_PORT,PIN_D3_MASK);
				//}
				//else
				//{
					//pio_clear(PIN_D2_PORT,PIN_D2_MASK);
					//pio_clear(PIN_D3_PORT,PIN_D3_MASK);
				//}
				//
		

			//sprintf(uart_buffer, "Contadores: C1: %d C2: %d\n", counter_1,counter_2);
			//uart_puts(UART, uart_buffer,strlen(uart_buffer));


			// if (status_D2)
			// {
			// 	pio_set(PIN_D2_PORT,PIN_D2_MASK);
			// 	uart_puts(UART, "D2 toggled ON\n");
			// }
			// else
			// {
			// 	pio_clear(PIN_D2_PORT,PIN_D2_MASK);
			// 	uart_puts(UART, "D2 toggled OFF\n");
			// }
		
			// if (status_D3)
			// {
			// 	pio_set(PIN_D3_PORT,PIN_D3_MASK);
			// 	uart_puts(UART, "D3 toggled ON\n");
			// }
			// else
			// {
			// 	pio_clear(PIN_D3_PORT,PIN_D3_MASK);
			// 	uart_puts(UART, "D3 toggled OFF\n");
			// }
		}
	}
}

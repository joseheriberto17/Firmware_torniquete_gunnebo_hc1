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
* Arduino Due Digital Pin Mapping (D0–D13)
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

void configure_pins(void);
void configure_uart(void);
void handle_encoder(const uint32_t id, const uint32_t mask);
uint8_t read_AB(void);
void uart_puts(Uart *uart, const char *str);


// salidas led
#define PIN_D3 PIO_PC28_IDX
#define PIN_D3_MASK PIO_PC28
#define PIN_D3_PORT PIOC

#define PIN_D2 PIO_PB25_IDX
#define PIN_D2_MASK PIO_PB25
#define PIN_D2_PORT PIOB

// entrada encoder
#define PIN_D4 PIO_PC26_IDX
#define PIN_D4_MASK PIO_PC26
#define PIN_D4_PORT PIOC

#define PIN_D5 PIO_PC25_IDX
#define PIN_D5_MASK PIO_PC25
#define PIN_D5_PORT PIOC

const int8_t qdec_table[4][4] = {
	// to:  00   01   10   11
	/*from 00*/ {  0, -1, +1,  0 },
	/*from 01*/ { +1,  0,  0, -1 },
	/*from 10*/ { -1,  0,  0, +1 },
	/*from 11*/ {  0, +1, -1,  0 }
};

// variables globales
volatile int32_t position = 0;
volatile uint8_t last_state = 0;
volatile bool new_dir = true;



uint8_t read_AB(void) {
	uint8_t c1 = pio_get(PIN_D5_PORT, PIO_INPUT, PIN_D5_MASK) ? 1 : 0;
	uint8_t c2 = pio_get(PIN_D4_PORT, PIO_INPUT, PIN_D4_MASK) ? 1 : 0;
	return (c2 << 1) | c1;
}

// Handler llamado por fase A o fase B, encoder tipo X4
void handle_encoder(const uint32_t id, const uint32_t mask) {
	// lectura de los encoder
	uint8_t new_state = read_AB();
		
	// validar contador
	int8_t delta = qdec_table[last_state][new_state];
	if (delta != 0) {
		position += delta; // posicion
		new_dir = (delta > 0); // direccion
	}
	
	
	if (new_dir)
	{
		pio_set(PIN_D2_PORT,PIN_D2_MASK);
		pio_clear(PIN_D3_PORT,PIN_D3_MASK);
	}
	else
	{
		pio_clear(PIN_D2_PORT,PIN_D2_MASK);
		pio_set(PIN_D3_PORT,PIN_D3_MASK);
	}

	last_state = new_state;
}

void configure_pins(void)
{
	// Habilitar el reloj para el PIOB
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	
	// salida
	pio_configure(PIN_D2_PORT,PIO_OUTPUT_0,PIN_D2_MASK,PIO_DEFAULT);
	pio_configure(PIN_D3_PORT,PIO_OUTPUT_0,PIN_D3_MASK,PIO_DEFAULT);
	
	// entrada
	pio_configure(PIN_D4_PORT,PIO_INPUT,PIN_D4_MASK,PIO_DEFAULT);
	pio_configure(PIN_D5_PORT,PIO_INPUT,PIN_D5_MASK,PIO_DEFAULT);
	
	// Habilitar la interrupción en el periférico y en el NVIC
	pio_enable_interrupt(PIOC, PIN_D4_MASK|PIN_D5_MASK);
	pio_handler_set(PIOC,ID_PIOC,PIN_D4_MASK|PIN_D5_MASK,PIO_IT_EDGE,handle_encoder);
	NVIC_EnableIRQ(PIOC_IRQn);
}
void configure_uart(void)
{
	pmc_enable_periph_clk (ID_UART);
	
	pio_configure(PINS_UART_PIO,PINS_UART_TYPE,PINS_UART_MASK,PINS_UART_ATTR);	
	
	
	
	sam_uart_opt_t uart_settings = {
		.ul_mck = sysclk_get_cpu_hz(),
		.ul_baudrate = 9600,
		.ul_mode = UART_MR_PAR_NO  // Modo sin paridad, 8N1
	};
	
	
	
	uart_init(UART, &uart_settings);
	uart_enable_tx(UART);
	uart_enable_rx(UART);
}

void uart_puts(Uart *uart, const char *str) {
	while (*str) {
		while (!uart_is_tx_ready(uart));
		uart_write(uart, *str++);
	}
}

int main (void)
{
	sysclk_init();
	board_init();
	configure_pins();
	configure_uart();
	
	
	last_state = read_AB();
	
	uart_puts(UART, "Hola desde UART\n");

	
	while (1) 
	{
	}
}

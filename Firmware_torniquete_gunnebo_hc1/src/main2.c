/*
 * main.c
 * -----------------
 * Controlador de torniquete con lectura de encoder rotativo (A/B/Index).
 * Maneja conteo de pasos, sentido de giro y control de salidas led.
 *
 * Device: torniquete gunnebo HC1.
 *
 * Detalles:
 * 	LED SEN_I: define el sentido horario
 * 	LED SEN_S: define el sentido antihorario
 * 	AMBOS LED: define la alerta de que la devolucion del paso de torniquete supero el valor de tolerancia.
 *
 * Version: ASF 3.52
 * 			Microchip studio (2020) 7.0.2594
 *
 *   Tabla de pin out
 *   +--------------+----------------+
 *   | Modulo IO    | SAM3S1B Pin    |
 *   +--------------+----------------+
 *   | RIGHT        | PA3            |
 *   | LEFT         | PA29           |
 *   | SEN_I        | PB13           |
 *   | SEN_S        | PB11           |
 *   | INDEX        | PB14           |
 *   | LED_SEN_I    | PB1            |
 *   | LED_SEN_S    | PB0            |
 *   | CONF_A       | PA16           |
 *   | PIC_1        | PA8            |
 *   | PIC_2        | PA7            |
 *   +--------------+----------------+
 *
 *
 *
 * Autor: Jose H.
 * Fecha: 13/05/2025
 */
#include <asf.h>
#include <stdio.h>
#include <stdio_serial.h>
#include "uart_custom.h"

void configure_pins(void);
void handle_encoder(const uint32_t id, const uint32_t mask);
// void configure_uart(void);
uint8_t read_AB(void);

// definicion de los parametros de la aplicacion
#define MAX_REVERSE_TOLERANCE 12 // define cuantos contadores puede contar si el torniquete se devuelve
#define COUNTER_ENCODER_PASE 60	 // define el valor minimo que tiene que contar position_encoder para validar un paso.

// Pin de entrada de encoder por SEN_I.
#define SEN_I_PIN PIO_PB13_IDX
#define SEN_I_PIN_MASK PIO_PB13
#define SEN_I_PIN_PORT PIOB

// Pin de entrada de encoder por SEN_S.
#define SEN_S_PIN PIO_PB11_IDX
#define SEN_S_PIN_MASK PIO_PB11
#define SEN_S_PIN_PORT PIOB

// Pin de entrada de encoder Index.
#define INDEX_PIN PIO_PB14_IDX
#define INDEX_PIN_MASK PIO_PB14
#define INDEX_PIN_PORT PIOB

// Pin de entrada de TQ confirm.
#define CONF_A_PIN PIO_PA16_IDX
#define CONF_A_PIN_MASK PIO_PA16
#define CONF_A_PIN_PORT PIOA

// Pin de salida de solenoide B.
#define RIGHT_PIN PIO_PA3_IDX
#define RIGHT_PIN_MASK PIO_PA3
#define RIGHT_PIN_PORT PIOA

// Pin de salida de solenoide A.
#define LEFT_PIN PIO_PA29_IDX
#define LEFT_PIN_MASK PIO_PA29
#define LEFT_PIN_PORT PIOA

// Pin de salida del led SEN_I.
#define LED_SEN_I_PIN PIO_PB1_IDX
#define LED_SEN_I_PIN_MASK PIO_PB1
#define LED_SEN_I_PIN_PORT PIOB

// Pin de salida del led SEN_S.
#define LED_SEN_S_PIN PIO_PB0_IDX
#define LED_SEN_S_PIN_MASK PIO_PB0
#define LED_SEN_S_PIN_PORT PIOB

// Pin de salida del led PICT_1.
#define PIC_1_PIN PIO_PA8_IDX
#define PIC_1_PIN_MASK PIO_PA8
#define PIC_1_PIN_PORT PIOA

// Pin de salida del led PICT_2.
#define PIC_2_PIN PIO_PA7_IDX
#define PIC_2_PIN_MASK PIO_PA7
#define PIC_2_PIN_PORT PIOA

// configuracion del UART1 conexion directa a (RS485).
#define UART_SERIAL_BAUDRATE 19200
#define UART_SERIAL_BAUDRATE_485 115200 /// para pruebas 19200, para produccion 115200
#define UART_SERIAL_CHANNEL_MODE UART_MR_CHMODE_NORMAL
#define UART_SERIAL_MODE UART_MR_PAR_NO

#define PINS_UART1 (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
#define PINS_UART1_FLAGS (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART1_MASK (PIO_PB2 | PIO_PB3)
#define PINS_UART1_PIO PIOB
#define PINS_UART1_ID ID_PIOB
#define PINS_UART1_TYPE PIO_PERIPH_A
#define PINS_UART1_ATTR PIO_DEFAULT

#define ERROR_UNDER	0x01	// Faltan datos, timeout
#define ERROR_OVER	0x02	// Se enviaron mas datos de los esperados antes del caracter 0xFC
#define ERROR_CHK	0x03	// Error de sumatoria
#define ERROR_ACS	0x04	// Se intenta escribir una entrada
#define ERROR_SCN	0x05	// Escenario invalido, < 16 � > 128

// definicion de las direcciones de los bancps de memoria
#define ACCUMULATOR_A               0x10
#define ACCUMULATOR_B               0x14
#define CONFIRMACION_FAIL			0x18
#define ALARM_STATUS				0x1C
#define BATTERY_STATUS				0x1D

#define PASO_MODE_A			0x10
#define PASO_MODE_B			0x11
#define TIMEOUT_PASO		0x12

// Tabla de transición de cuadratura para decodificador X4.
const int8_t qdec_table[4][4] = {
	// to:        00  01  10  11
	/*from 00*/ {0, -1, +1, 0},
	/*from 01*/ {+1, 0, 0, -1},
	/*from 10*/ {-1, 0, 0, +1},
	/*from 11*/ {0, +1, -1, 0}};

// variables globales
// ------------------------------------------------------------------------
// conteo de pulsos del encoder
volatile uint8_t last_state_encoder = 0;	// define la  ultima secuencia de los encoder (A/B).
// conteo de posicion y direccion del encoder.
volatile int32_t position_encoder = 0;		// define el micropaso actual del encoder relativo al ultimo final de carrera del torniquete.
volatile int32_t position_encoder_last = 0; // define el micropaso maximo del encoder relativo la ultimo final de carrera del torniquete, durante el proceso de un paso.
volatile int8_t delta_last = 0;				// direccion del ultima del encoder (valor -1 y +1).
// validacion de pase en progreso
volatile bool end_pase = true;				// indica si position_encoder esta en un final de carrera.
volatile bool invalid_pase = false;			// indica si durante el proceso de un paso el torniquete se devolvio superando el valor de tolerancia (MAX_REVERSE_TOLERANCE).
// confirmacion de un pase
volatile bool confirm_pase = false;			// confirma si permite un paso del torniquete
volatile bool last_confirm_pase = false;	// complemento para 
// gestion de un pase del torniquete
volatile bool pase_A = false;					// control de un pase del torniquete por A
volatile bool pase_B = false;					// control de un pase del torniquete por B
volatile bool flag_pase = false;				// incrementador de paso

// contador de pase de torniquete
volatile int16_t counter_pase = 0;		
volatile int16_t acum_pase_A = 0;		// sentido A
volatile int16_t acum_pase_B = 0;		// sentido B
volatile int16_t acum_pase_fail = 0;		// sin autorizacion
volatile int16_t acum_timeout = 0;		// timeout


// comunicacion RS485
char	bufer_seria_tx[256];
unsigned char	bufer_serial_rx[256];
int		port_slots_write[256];
char	port_slots_read[256];
int		port_slots_default[256];
//
int		data_leng = 256;
int		dev_id = 0x81;
int		escenario_temp = 0xFF;

int		port_address = 0;
int		rx_idx_RS485;
char	funcion	= 0x20;
int		char_timeout_RS485 = 20;
char	error_code = 0x00;
char	escenario_v = 0x00;

// obtiene los estados de los pines asignado para la lectura del encoder, devueve en un nibble la secuencia del estado de encoder.
uint8_t read_AB(void)
{
	uint8_t A = pio_get(SEN_I_PIN_PORT, PIO_INPUT, SEN_I_PIN_MASK) ? 1 : 0;
	uint8_t B = pio_get(SEN_S_PIN_PORT, PIO_INPUT, SEN_S_PIN_MASK) ? 1 : 0;
	return (A << 1) | B;
}

// funcion handler para la interrupcion para la logica del encoder (A/B)
// Detecta sentido, cambios y final de carrera.
void handle_encoder(const uint32_t id, const uint32_t mask)
{
	uint8_t new_state_encoder = read_AB(); // lectura de los encoder
	// determina si esta en final de carrera.
	end_pase = pio_get(INDEX_PIN_PORT, PIO_INPUT, INDEX_PIN_MASK) ? 1 : 0;

	// validar el conteo de encoder. (+1,-1,0)
	int8_t delta = qdec_table[last_state_encoder][new_state_encoder];
	last_state_encoder = new_state_encoder;
	if (delta != 0)
	{
		position_encoder += delta; // deteccion de la posicion del encoder

		// Si estamos en el final de carrera
		if (end_pase)
		{
			// si durante el pase en progreso ocurrio una devolucion
			if (invalid_pase == false)
			{
				if (position_encoder_last > COUNTER_ENCODER_PASE)
				{
					pio_toggle_pin(PIC_1_PIN); // Giro en sentido horario.
					flag_pase = true;
					acum_pase_B++;
				}
				if (position_encoder_last < -COUNTER_ENCODER_PASE)
				{
					pio_toggle_pin(PIC_2_PIN); // Giro en sentido antihorario.
					flag_pase = true;
					acum_pase_A++;
				}
			}

			// Reseteo de variables
			position_encoder = 0; 
			position_encoder_last = 0;
			invalid_pase = false;

			//  Apagamos los LEDs de dirección (SEN_I y SEN_S).
			pio_clear(LED_SEN_I_PIN_PORT, LED_SEN_I_PIN_MASK);
			pio_clear(LED_SEN_S_PIN_PORT, LED_SEN_S_PIN_MASK);
		}
		else
		{
			
			// validacion de un pase en procesos
			// ----------------------------------------------------------------
			// Almacena la mayor distancia recorrida desde el último final de carrera
			// y la ultima dirección (signo) de ese desplazamiento.
			if (abs(position_encoder_last) < abs(position_encoder))
			{
				position_encoder_last = position_encoder;
				delta_last = delta;
			}
			// Si el torniquete se devuelve más allá del límite permitido,
			// se considera como intento de reversa y se activa la bandera.
			if (abs(position_encoder_last - position_encoder) > MAX_REVERSE_TOLERANCE)
			{
				position_encoder = 0;
				invalid_pase = true;
			}

			// Control de salidas:
			// ------------------------------------------------------------------------------
			// - Si hubo reversa: ambos LEDs encendidos (advertencia invalid_pase = true).
			// - Si avanza en sentido horario: LED SEN_I.
			// - Si avanza en sentido antihorario: LED SEN_S.
			if (invalid_pase)
			{
				pio_set(LED_SEN_I_PIN_PORT, LED_SEN_I_PIN_MASK);
				pio_set(LED_SEN_S_PIN_PORT, LED_SEN_S_PIN_MASK);
			}
			else if (delta_last > 0)
			{
				pio_set(LED_SEN_I_PIN_PORT, LED_SEN_I_PIN_MASK);
				pio_clear(LED_SEN_S_PIN_PORT, LED_SEN_S_PIN_MASK);
			}
			else
			{
				pio_clear(LED_SEN_I_PIN_PORT, LED_SEN_I_PIN_MASK);
				pio_set(LED_SEN_S_PIN_PORT, LED_SEN_S_PIN_MASK);
			}
		}
	}
}

// Configura pines como entrada/salida e interrupciones.
void configure_pins(void)
{
	// Activar el reloj del periferico PIOB
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOA);

	// Salidas GPIO
	pio_configure(LED_SEN_I_PIN_PORT, PIO_OUTPUT_0, LED_SEN_I_PIN_MASK, PIO_DEFAULT);
	pio_configure(LED_SEN_S_PIN_PORT, PIO_OUTPUT_0, LED_SEN_S_PIN_MASK, PIO_DEFAULT);
	pio_configure(PIC_1_PIN_PORT, PIO_OUTPUT_0, PIC_1_PIN_MASK, PIO_DEFAULT);
	pio_configure(PIC_2_PIN_PORT, PIO_OUTPUT_0, PIC_2_PIN_MASK, PIO_DEFAULT);
	pio_configure(RIGHT_PIN_PORT, PIO_OUTPUT_0, RIGHT_PIN_MASK, PIO_DEFAULT);
	pio_configure(LEFT_PIN_PORT, PIO_OUTPUT_0, LEFT_PIN_MASK, PIO_DEFAULT);

	// Entrada GPIO
	pio_configure(SEN_I_PIN_PORT, PIO_INPUT, SEN_I_PIN_MASK, PIO_DEFAULT);
	pio_configure(SEN_S_PIN_PORT, PIO_INPUT, SEN_S_PIN_MASK, PIO_DEFAULT);
	pio_configure(INDEX_PIN_PORT, PIO_INPUT, INDEX_PIN_MASK, PIO_DEFAULT);
	pio_configure(CONF_A_PIN_PORT, PIO_INPUT, CONF_A_PIN_MASK, PIO_DEFAULT);

	pio_configure_interrupt(SEN_I_PIN_PORT, SEN_I_PIN_MASK, PIO_IT_EDGE);
	pio_configure_interrupt(SEN_S_PIN_PORT, SEN_S_PIN_MASK, PIO_IT_EDGE);
	pio_handler_set(PIOB, ID_PIOB, SEN_I_PIN_MASK | SEN_S_PIN_MASK, PIO_IT_EDGE, handle_encoder);

	// Habilitar la interrupcion en el periferico y en el NVIC
	pio_enable_interrupt(PIOB, SEN_I_PIN_MASK | SEN_S_PIN_MASK);
	NVIC_EnableIRQ(PIOB_IRQn);
}

void configure_uart(void)
{
	pmc_enable_periph_clk(ID_UART1);

	// set the pins to use the uart peripheral
	pio_configure(PINS_UART1_PIO, PINS_UART1_TYPE, PINS_UART1_MASK, PINS_UART1_ATTR);

	sam_uart_opt_t uart1_settings = {
		.ul_mck = sysclk_get_cpu_hz(),
		.ul_baudrate = 19200,
		.ul_mode = UART_SERIAL_MODE // Modo sin paridad, 8N1
	};

	uart_init(UART1, &uart1_settings); 
	NVIC_EnableIRQ(UART1_IRQn);
	uart_enable(UART1);
}

void	not_ack_RS485()
{
	if (bufer_serial_rx[0] != 0x80)
	{
		bufer_seria_tx[0] = 0xA1;
		bufer_seria_tx[1] = 0x15;
		bufer_seria_tx[2] = error_code;
		bufer_seria_tx[3] = -(0xA1 ^ 0x15 ^ error_code);
		bufer_seria_tx[4] = 0xFC;
		
		Pdc	*serial_485;
		pdc_packet_t serial_485_TX_packet;
		serial_485 = uart_get_pdc_base(UART1);
		pdc_enable_transfer(serial_485, PERIPH_PTCR_TXTEN);
		serial_485_TX_packet.ul_addr = (uint32_t)bufer_seria_tx;
		serial_485_TX_packet.ul_size = 5;
		pdc_tx_init(serial_485, &serial_485_TX_packet, NULL);
	}
}

void UART1_Handler()
{
	unsigned int static sumatoria_v = 0;
	char_timeout_RS485 = 20;
	//	static	int	rx_idx;
	uint8_t received_byte;
	//	unsigned	int	temp_char = 0;
	uint32_t dw_status = uart_get_status(UART1);



	if(dw_status & UART_SR_RXRDY)
	{
		uart_read(UART1, &received_byte);
		//			temp_char = (int)received_byte;
		//	uart_write(UART1, received_byte);

		bufer_serial_rx[rx_idx_RS485 ++] = (int)received_byte;
		
		if (rx_idx_RS485 == 0x01)
		{
			sumatoria_v = (int)received_byte;
			data_leng = 255;
			if (((unsigned int)received_byte != dev_id) && ((unsigned int)received_byte != 0x80))
			{
				rx_idx_RS485 = 0;
				funcion = 0x20;
			}
		}

		
		if ((rx_idx_RS485 != 0x01) && (rx_idx_RS485 < (data_leng + 4)))
		{
			sumatoria_v ^= (int)received_byte;
		}
		
		if (rx_idx_RS485 == 0x02)
		{
			if (((int)received_byte == 0x10) || ((int)received_byte == 0x1A) || ((int)received_byte == 0x1B))
			{
				funcion = (int)received_byte;
			}
			else
			{
				rx_idx_RS485 = 0;
				funcion = 0x20;
			}
		}
		
		if (rx_idx_RS485 == 3)
		{
			port_address = (int)received_byte;
		}

		if (rx_idx_RS485 == 4)
		{
			data_leng = (int)received_byte;
		}
		
		if (rx_idx_RS485 == 5)
		{
			escenario_temp = (int)received_byte;
		}
		
		
		int	i = 1;

		switch (funcion)
		{
			case 0x10:

			if (bufer_serial_rx[0] != 0x80)
			{
				if ((rx_idx_RS485 == 0x06) && ((int)received_byte != 0xFC))
				{
					error_code = ERROR_OVER;
					not_ack_RS485();
					char_timeout_RS485 = 0;
					rx_idx_RS485 = 0;
				}

				

				
				sumatoria_v = (unsigned int)bufer_serial_rx[0];
				
				for (i = 1; i < (4); i++)
				{
					sumatoria_v ^= (unsigned int)bufer_serial_rx[i];
				}
				if ((rx_idx_RS485 == 0x06) && ((-sumatoria_v & 0x000000FF) != (unsigned int)bufer_serial_rx[4] ))
				{
					error_code = ERROR_CHK;
					not_ack_RS485();
				}
				
				if ((rx_idx_RS485 == 6) && ((unsigned char)received_byte == 0xFC))
				{
					
					
					if ((-sumatoria_v & 0x000000FF) == (unsigned int)bufer_serial_rx[4] )
					{
						
						
						sumatoria_v = 0xA1 ^ 0x10 ^ (char)port_address ^ (char)data_leng;
						
						bufer_seria_tx[0] = 0xA1;
						bufer_seria_tx[1] = 0x10;
						bufer_seria_tx[2] = (char)port_address;
						bufer_seria_tx[3] = (char)data_leng;
						
						for (i = 4; i < (data_leng + 4); i++)
						{
							bufer_seria_tx[i] = (char)port_slots_read[port_address];
							sumatoria_v ^= (char)port_slots_read[port_address++];
						}
						
						bufer_seria_tx[i++] = -(char)sumatoria_v;
						bufer_seria_tx[i] = 0xFC;
						
						Pdc	*serial_485;
						pdc_packet_t serial_485_TX_packet;
						serial_485 = uart_get_pdc_base(UART1);
						pdc_enable_transfer(serial_485, PERIPH_PTCR_TXTEN);
						serial_485_TX_packet.ul_addr = (uint32_t)bufer_seria_tx;
						serial_485_TX_packet.ul_size = (data_leng + 6);
						pdc_tx_init(serial_485, &serial_485_TX_packet, NULL);
						
					}
					funcion = 0x20;
					rx_idx_RS485 = 0;
				}
				

			}
			
			break;
			
			
			
			//
			case 0x1A:
			sumatoria_v = (unsigned int)bufer_serial_rx[0];
			for (i = 1; i < (data_leng + 4); i++)
			{
				sumatoria_v ^= (unsigned int)bufer_serial_rx[i];
			}
			
			if ((rx_idx_RS485 == (6 + data_leng)) && ((int)received_byte != 0xFC))
			{
				error_code = ERROR_OVER;
				not_ack_RS485();
			}
			else
			{
				if ((rx_idx_RS485 == (0x06 + data_leng)) && ((-sumatoria_v & 0x000000FF) != (unsigned int)bufer_serial_rx[data_leng + 4] ))

				{
					error_code = ERROR_CHK;
					not_ack_RS485();
				}
			}

			if ((rx_idx_RS485 == (6 + data_leng)) && ((unsigned int)received_byte == 0xFC))
			{

				if ((-sumatoria_v & 0x000000FF) == (unsigned int)bufer_serial_rx[i] )
				{
					if ((port_address + data_leng) < 0x80)
					{
						int	port_address_temp = port_address;
						for (i = 0; i < data_leng; i++)
						{
							port_slots_write[port_address_temp++] = bufer_serial_rx[i + 4];
						}
						funcion = 0x20;
						
						if (bufer_serial_rx[0] != 0x80)
						{
							bufer_seria_tx[0] = 0xA1;
							bufer_seria_tx[1] = 0x06;
							bufer_seria_tx[2] = 0x00;
							bufer_seria_tx[3] = -((char)(0xA1 ^ 0x06 ^ 0x00));
							bufer_seria_tx[4] = 0xFC;
							
							Pdc	*serial_485;
							pdc_packet_t serial_485_TX_packet;
							serial_485 = uart_get_pdc_base(UART1);
							pdc_enable_transfer(serial_485, PERIPH_PTCR_TXTEN);
							serial_485_TX_packet.ul_addr = (uint32_t)bufer_seria_tx;
							serial_485_TX_packet.ul_size = 5;
							pdc_tx_init(serial_485, &serial_485_TX_packet, NULL);
						}
					}
					else
					{
						error_code = ERROR_ACS;
						not_ack_RS485();
					}
				}
			}
			
			break;

			case 0x1B:

			if (bufer_serial_rx[0] != 0x80)
			{
				if ((rx_idx_RS485 == 0x07) && ((int)received_byte != 0xFC))
				{
					error_code = ERROR_OVER;
					not_ack_RS485();
					char_timeout_RS485 = 0;
					rx_idx_RS485 = 0;
				}


				sumatoria_v = (unsigned int)bufer_serial_rx[0];
				
				for (i = 1; i < (5); i++)
				{
					sumatoria_v ^= (unsigned int)bufer_serial_rx[i];
				}
				
				if ((rx_idx_RS485 == 0x07) && ((-sumatoria_v & 0x000000FF) != (unsigned int)bufer_serial_rx[5] ))
				{
					error_code = ERROR_CHK;
					not_ack_RS485();
				}
				
				if ((rx_idx_RS485 == 7) && ((unsigned char)received_byte == 0xFC))
				{
					
					
					if ((-sumatoria_v & 0x000000FF) == (unsigned int)bufer_serial_rx[5] )
					{
						
							bufer_seria_tx[0] = 0xA1;
							bufer_seria_tx[1] = 0x06;
							bufer_seria_tx[2] = 0x00;
							bufer_seria_tx[3] = -((char)(0xA1 ^ 0x06 ^ 0x00));
							bufer_seria_tx[4] = 0xFC;
							
							Pdc	*serial_485;
							pdc_packet_t serial_485_TX_packet;
							serial_485 = uart_get_pdc_base(UART1);
							pdc_enable_transfer(serial_485, PERIPH_PTCR_TXTEN);
							serial_485_TX_packet.ul_addr = (uint32_t)bufer_seria_tx;
							serial_485_TX_packet.ul_size = 5;
							pdc_tx_init(serial_485, &serial_485_TX_packet, NULL);
						//}
						

						
					}
					funcion = 0x20;
					rx_idx_RS485 = 0;
				}
				

			}
			
			break;
			
			
		}

		if (rx_idx_RS485 == (6 + data_leng))
		{
			rx_idx_RS485 = 0;
			funcion = 0x20;
		}
		
	}
}

int main(void)
{
	sysclk_init();				// Inicializa reloj del sistema
	WDT->WDT_MR = WDT_MR_WDDIS; // Desactiva watchdog
	board_init();				// Inicializa la placa base (ASF)
	configure_pins();			// Configura E/S y habilita interrupciones
	// configure_uart();			// Configura E/S y habilita interrupciones

	last_state_encoder = read_AB(); // Guarda estado inicial del encoder

	// uart_puts(UART1, "\x1B[2J\x1B[H"); // Limpia toda pantalla y va al tope

	// char buffer[128]; // buffer del uart


	// 	/* Configura UART. */
	//	#include "asf.h" //uart.h etc. included here
	
	pmc_enable_periph_clk(ID_UART1);
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	#define UART_SERIAL_BAUDRATE       19200
	#define UART_SERIAL_BAUDRATE_485       19200 /// para pruebas 19200, para produccion 115200
	#define UART_SERIAL_CHANNEL_MODE   UART_MR_CHMODE_NORMAL
	#define UART_SERIAL_MODE         UART_MR_PAR_NO

	/* =============== UART1 =============== */ //(UART0 is defined but not UART1)
	#define PINS_UART1          (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
	#define PINS_UART1_FLAGS    (PIO_PERIPH_A | PIO_DEFAULT)
	#define PINS_UART1_MASK     (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
	#define PINS_UART1_PIO      PIOB
	#define PINS_UART1_ID       ID_PIOB
	#define PINS_UART1_TYPE     PIO_PERIPH_A
	#define PINS_UART1_ATTR     PIO_DEFAULT

	// set the pins to use the uart peripheral
	pio_configure(PINS_UART1_PIO, PINS_UART1_TYPE, PINS_UART1_MASK, PINS_UART1_ATTR);

	//enable the uart peripherial clock
	const sam_uart_opt_t uart1_settings =
	{ sysclk_get_cpu_hz(), UART_SERIAL_BAUDRATE_485, UART_SERIAL_MODE };

	uart_init(UART1,&uart1_settings);      //Init UART1 and enable Rx and Tx

	uart_enable_interrupt(UART1,UART_IER_RXRDY);   //Interrupt reading ready
	NVIC_EnableIRQ(UART1_IRQn);

	// 	uart_init(UART1, &serial_485_opt_t);
	uart_enable_tx(UART1);
	uart_enable_rx(UART1);
	uart_enable(UART1);   //Enable UART1
	

	// 	// 	// Configura DMA serial TX
	Pdc	*serial_485;
	//	pdc_packet_t serial_485_TX_packet;
	serial_485 = uart_get_pdc_base(UART1);
	pdc_enable_transfer(serial_485, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	// 	// Habilita interrupciones
	uart_enable_interrupt(UART1, UART_IER_RXRDY);
	NVIC_EnableIRQ(UART1_IRQn);
	Enable_global_interrupt();


	while (1)
	{

		if (port_slots_write[PASO_MODE_A] == 0xFF) {
			pase_A = true;	
			port_slots_write[PASO_MODE_A] = 0x00;
		} else if (port_slots_write[PASO_MODE_B] == 0xFF) {  // no se debe emitar pasos en los dos sentidos
			pase_B = true;	
			port_slots_write[PASO_MODE_B] = 0x00;
		}
		
		// // confirmacion de 10 pasos con un interruptor
		// confirm_pase = pio_get(CONF_A_PIN_PORT, PIO_INPUT, CONF_A_PIN_MASK);
		// if (confirm_pase != last_confirm_pase)
		// {
		// 	if (confirm_pase == 1)
		// 	{
		// 		pase_A = true;
		// 		counter_pase = 10;
		// 	}
		// 	last_confirm_pase = confirm_pase;
		// }

		if (flag_pase)
		{
			flag_pase = false;
			// uart_puts(UART1, "\x1B[2J\x1B[H"); // Limpia toda pantalla y va al tope
			// snprintf(buffer, sizeof(buffer), "Giro sentido A: %d\r\n"
			// 								 "Giro sentido B: %d\r\n"
			// 								 "Giro sin autorizacion: %d\r\n"
			// 								 "Timeout: %d\r\n",
			// 								  acum_pase_A,
			// 								  acum_pase_B,
			// 								  acum_pase_fail,
			// 								  acum_timeout);
			// uart_puts(UART1, buffer); // salida formateada

			for (int i = 0; i < sizeof(acum_pase_A); i++) {
					port_slots_read[ACCUMULATOR_A + i] = acum_pase_A >> (i * 8);
			}
			for (int i = 0; i < sizeof(acum_pase_B); i++) {
					port_slots_read[ACCUMULATOR_B + i] = acum_pase_B >> (i * 8);
			}
			for (int i = 0; i < sizeof(acum_pase_fail); i++) {
					port_slots_read[CONFIRMACION_FAIL + i] = acum_pase_fail >> (i * 8);
			}
			for (int i = 0; i < sizeof(acum_timeout); i++) {
					port_slots_read[TIMEOUT_PASO + i] = acum_timeout >> (i * 8);
			}

			// if (counter_pase > 1)
			// {
			// 	pase = true;
			// 	counter_pase--;
			// }
		}

		// TODO: esta lógica parace solo aplicar para el paso por A, fatal paso por B.
		// control de paso en proceso de torniquete.
		if (pase_A && (abs(position_encoder_last) < 50))
		{
			pio_set(RIGHT_PIN_PORT, RIGHT_PIN_MASK);
			pio_set(LEFT_PIN_PORT, LEFT_PIN_MASK);
		}
		else
		{
			pio_clear(RIGHT_PIN_PORT, RIGHT_PIN_MASK);
			pio_clear(LEFT_PIN_PORT, LEFT_PIN_MASK);
			pase_A = false;
		}

		delay_us(100);

		

	}
}
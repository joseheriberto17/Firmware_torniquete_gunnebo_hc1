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
 * 	 | SW2_1        | PA23           |
 *   | SW2_2        | PA22           |
 *   | SW2_3        | PA19           |
 *   | SW2_4        | PA21           |
 *   +--------------+----------------+
 *
 *
 *
 * Autor: Jose H.
 * Fecha: 13/05/2025
 */

#include <asf.h>
#include <stdio.h>
#include <string.h>
#include "uart_custom.h"
#include "Escenarios.h"
#include "Pictogramas.h"

void configure_pins(void);
void configure_uart(void);
void handle_encoder(const uint32_t id, const uint32_t mask);
uint8_t read_AB(void);
uint32_t millis(void);
void configure_systick(void);
void not_ack_RS485(void);

// definicion de los parametros de la aplicacion
#define MAX_REVERSE_TOLERANCE 12 // cuantos contadores puede contar si el torniquete se devuelve
#define COUNTER_ENCODER_PASE 60	 // valor que tiene que contar position_encoder para validar un paso.
#define COUNTER_ENCODER_PASE_85P 50 // valor del contador de position_encoder donde tiene que desabilitar el paso autorizado.
#define VALUE_TIMER_ALARM_PASE 40000  // Tiempo máximo (ms) fuera de posicion de inicio antes de alarma.
#define VALUE_TIMEOUT_PASE 30 // valor que define por cuanto tiempo el sentido autorizado esta habilitado.
#define COUNTER_ENCODER_PICTO 12 // número de pasos permitidos fuera del posicion de inicio, para reducir el rebote por parte de los pictogramas.

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

// configuracion del UART1 conexion directa a (RS485).
#define UART_SERIAL_BAUDRATE_485 19200
#define UART_SERIAL_CHANNEL_MODE UART_MR_CHMODE_NORMAL
#define UART_SERIAL_MODE UART_MR_PAR_NO

#define PINS_UART1 (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
#define PINS_UART1_FLAGS (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART1_MASK (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
#define PINS_UART1_PIO PIOB
#define PINS_UART1_ID ID_PIOB
#define PINS_UART1_TYPE PIO_PERIPH_A
#define PINS_UART1_ATTR PIO_DEFAULT

#define ERROR_UNDER 0x01 // Faltan datos, timeout
#define ERROR_OVER 0x02	 // Se enviaron mas datos de los esperados antes del caracter 0xFC
#define ERROR_CHK 0x03	 // Error de sumatoria
#define ERROR_ACS 0x04	 // Se intenta escribir una entrada
#define ERROR_SCN 0x05	 // Escenario invalido, < 16 � > 128

// definicion de las direcciones de los bancps de memoria
#define ACCUMULATOR_A 0x10
#define ACCUMULATOR_B 0x14
#define CONFIRMACION_FAIL 0x18
#define ALARM_STATUS 0x1C
// TODO: si durante un tiempo no se configuro el valor de CONF_ESCENARIO CONF_TIMEOUT
// Activa la arlam y trabajacon vcon valores por defecto.
#define CONF_STATUS 0x1D
#define TIMEOUT_PASO 0x1E

#define PASO_MODE_A 0x10
#define PASO_MODE_B 0x11
// hay 4 escenarios  1234
#define CONF_ESCENARIO 0x12
// valor entre (1 - ?)
#define CONF_TIMEOUT 0x13
// Tabla de transición de cuadratura para decodificador X4.
const int8_t qdec_table[4][4] = {
	// to:        00  01  10  11
	/*from 00*/ {0, -1, +1, 0},
	/*from 01*/ {+1, 0, 0, -1},
	/*from 10*/ {-1, 0, 0, +1},
	/*from 11*/ {0, +1, -1, 0}};

// variables globales
// ------------------------------------------------------------------------
// conteo de posicion y direccion del encoder.
volatile int32_t position_encoder = 0;		// define el micropaso actual del encoder relativo al ultimo posicion de inicio del torniquete.
volatile int32_t position_encoder_last = 0; // define el micropaso maximo del encoder relativo la ultimo posicion de inicio del torniquete, durante el proceso de un paso.
volatile bool position_encoder_moved = 0;
// validacion de pase en progreso
volatile bool end_pase = true;		// indica si position_encoder esta en un posicion de inicio.
volatile bool reversa_pase = false; // indica si durante el proceso de un paso el torniquete se devolvio superando el valor de tolerancia (MAX_REVERSE_TOLERANCE).
// gestion de un pase del torniquete
volatile bool pase_A = false;
volatile bool pase_B = false;
volatile bool flag_pase_A = false;
volatile bool flag_pase_B = false;

// contador de pase de torniquete
volatile int16_t counter_pase = 0;
volatile int16_t acum_pase_A = 0;	 // sentido A
volatile int16_t acum_pase_B = 0;	 // sentido B
volatile int16_t acum_pase_fail = 0; // sin autorizacion
volatile int16_t acum_timeout = 0;	 // timeout

// comunicacion RS485
volatile char bufer_seria_tx[256];
volatile unsigned char bufer_serial_rx[256];
volatile int port_slots_write[256];
volatile char port_slots_read[256];
volatile int port_slots_default[256];

volatile int data_leng = 256;
volatile int dev_id = 0x81;

volatile int port_address = 0;
volatile int rx_idx_RS485;
volatile char funcion = 0x20;
volatile int char_timeout_RS485 = 20;
volatile char error_code = 0x00;

// manejo de tiempos
volatile uint32_t ms_ticks = 0;
volatile bool timeout_pase = false;

// TODO: definir la logica de alarma de torniquete.
volatile bool alarm_pase = false;

volatile uint8_t escenario_app = 0;
volatile uint8_t dif_escenario_app = 0;

// timeout de configuracion de torniquete.
volatile uint16_t conf_timeout_pase = VALUE_TIMEOUT_PASE;

void configure_systick(void)
{
	// SystemCoreClock debe estar correctamente definido como 64000000
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		// Error al configurar SysTick
		while (1)
			;
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

// obtiene los estados de los pines asignado para la lectura del encoder, devueve en un nibble la secuencia del estado de encoder.
uint8_t read_AB(void)
{
	uint8_t encoder_A = pio_get(SEN_I_PIN_PORT, PIO_INPUT, SEN_I_PIN_MASK) ? 1 : 0;
	uint8_t encoder_B = pio_get(SEN_S_PIN_PORT, PIO_INPUT, SEN_S_PIN_MASK) ? 1 : 0;
	return (encoder_A << 1) | encoder_B;
}

// funcion handler para la interrupcion para la logica del encoder (A/B)
// Detecta sentido, cambios y posicion de inicio.
void handle_encoder(const uint32_t id, const uint32_t mask)
{
	uint8_t escenario = 0;

	// conteo de pulsos del encoder
	static uint8_t last_state_encoder = 0; // define la  ultima secuencia de los encoder (A/B).
	uint8_t new_state_encoder = read_AB(); // define la secuencia actual de los encoder (A/B).

	// determina si esta una posicion de inicio.
	end_pase = pio_get(INDEX_PIN_PORT, PIO_INPUT, INDEX_PIN_MASK) ? 1 : 0;

	// validar el conteo de encoder. (+1,-1,0)
	int8_t new_delta = qdec_table[last_state_encoder][new_state_encoder];
	last_state_encoder = new_state_encoder;
	static int8_t delta_last = 0; // direccion del ultima del encoder (valor -1 y +1).

	if (new_delta != 0)
	{
		position_encoder += new_delta; // posicion absoluta del encoder.

		if (end_pase)
		{
			// si durante el pase en progreso NO ocurrio una devolucion.
			if (reversa_pase == false)
			{
				if (position_encoder_last < -COUNTER_ENCODER_PASE)
				{
					acum_pase_A++;

					// confirmacion de pase no autorizado
					if (escenario_app != 0)
					{
						escenario = escenario_app;
					}
					else
					{
						escenario = Read_SW_2_Esc() + 1;
					}

					if ((escenario == 1 || escenario == 2) && !flag_pase_A)
					{
						acum_pase_fail++;
					}

					flag_pase_A = false;
				}
				if (position_encoder_last > COUNTER_ENCODER_PASE)
				{
					acum_pase_B++;

					// confirmacion de pase no autorizado
					if (escenario_app != 0)
					{
						escenario = escenario_app;
					}
					else
					{
						escenario = Read_SW_2_Esc() + 1;
					}

					if ((escenario == 1 || escenario == 3) && !flag_pase_B)
					{
						acum_pase_fail++;
					}

					flag_pase_B = false;
				}
			}

			// Reseteo de variables
			position_encoder = 0;
			position_encoder_last = 0;
			reversa_pase = false;
			alarm_pase = false;

			//  Apagamos los LEDs de dirección (SEN_I y SEN_S).
			pio_clear(LED_SEN_I_PIN_PORT, LED_SEN_I_PIN_MASK);
			pio_clear(LED_SEN_S_PIN_PORT, LED_SEN_S_PIN_MASK);
		}
		else
		{

			// validacion de un pase en procesos
			// ----------------------------------------------------------------
			// Almacena la mayor distancia recorrida desde el último posicion de inicio
			// y la ultima dirección (signo) de ese desplazamiento.
			if (abs(position_encoder_last) < abs(position_encoder))
			{
				position_encoder_last = position_encoder;
				delta_last = new_delta;
				position_encoder_moved = true;
			}
			// Si el torniquete se devuelve más allá del límite permitido,
			// se considera como intento de reversa y se activa la bandera.
			if (abs(position_encoder_last - position_encoder) > MAX_REVERSE_TOLERANCE)
			{
				position_encoder = 0;
				reversa_pase = true;
			}

			// Control de salidas:
			// ------------------------------------------------------------------------------
			// - Si hubo reversa: ambos LEDs encendidos (advertencia reversa_pase = true).
			// - Si avanza en sentido horario: LED SEN_I.
			// - Si avanza en sentido antihorario: LED SEN_S.
			if (reversa_pase)
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
	pio_configure(PINS_UART1_PIO, PINS_UART1_TYPE, PINS_UART1_MASK, PINS_UART1_ATTR);

	sam_uart_opt_t uart1_settings = {
		.ul_mck = sysclk_get_cpu_hz(),
		.ul_baudrate = 19200,
		.ul_mode = UART_SERIAL_MODE // Modo sin paridad, 8N1
	};

	uart_init(UART1, &uart1_settings);
	uart_enable_interrupt(UART1, UART_IER_RXRDY);
	NVIC_EnableIRQ(UART1_IRQn);
}

void not_ack_RS485(void)
{
	if (bufer_serial_rx[0] != 0x80)
	{
		bufer_seria_tx[0] = 0xA1;
		bufer_seria_tx[1] = 0x15;
		bufer_seria_tx[2] = error_code;
		bufer_seria_tx[3] = -(0xA1 ^ 0x15 ^ error_code);
		bufer_seria_tx[4] = 0xFC;

		uart_puts(UART1, (char *)bufer_seria_tx, 5);

		// Pdc	*serial_485;
		// pdc_packet_t serial_485_TX_packet;
		// serial_485 = uart_get_pdc_base(UART1);
		// pdc_enable_transfer(serial_485, PERIPH_PTCR_TXTEN);
		// serial_485_TX_packet.ul_addr = (uint32_t)bufer_seria_tx;
		// serial_485_TX_packet.ul_size = 5;
		// pdc_tx_init(serial_485, &serial_485_TX_packet, NULL);
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

	if (dw_status & UART_SR_RXRDY)
	{
		uart_read(UART1, &received_byte);
		//			temp_char = (int)received_byte;
		//	uart_write(UART1, received_byte);

		bufer_serial_rx[rx_idx_RS485++] = (int)received_byte;

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

		int i = 1;

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
					break;
				}

				sumatoria_v = (unsigned int)bufer_serial_rx[0];

				for (i = 1; i < (4); i++)
				{
					sumatoria_v ^= (unsigned int)bufer_serial_rx[i];
				}
				if ((rx_idx_RS485 == 0x06) && ((-sumatoria_v & 0x000000FF) != (unsigned int)bufer_serial_rx[4]))
				{
					error_code = ERROR_CHK;
					not_ack_RS485();
					rx_idx_RS485 = 0;
					break;
				}

				if ((rx_idx_RS485 == 6) && ((unsigned char)received_byte == 0xFC))
				{

					if ((-sumatoria_v & 0x000000FF) == (unsigned int)bufer_serial_rx[4])
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

							if ((port_address - 1) == ACCUMULATOR_A)
							{
								acum_pase_A = 0;
							}
							if ((port_address - 1) == ACCUMULATOR_B)
							{
								acum_pase_B = 0;
							}
							if ((port_address - 1) == CONFIRMACION_FAIL)
							{
								acum_pase_fail = 0;
							}
							if ((port_address - 1) == TIMEOUT_PASO)
							{
								timeout_pase = false;
							}
						}

						bufer_seria_tx[i++] = -(char)sumatoria_v;
						bufer_seria_tx[i] = 0xFC;

						uart_puts(UART1, (char *)bufer_seria_tx, (data_leng + 6));

						// Pdc	*serial_485;
						// pdc_packet_t serial_485_TX_packet;
						// serial_485 = uart_get_pdc_base(UART1);
						// pdc_enable_transfer(serial_485, PERIPH_PTCR_TXTEN);
						// serial_485_TX_packet.ul_addr = (uint32_t)bufer_seria_tx;
						// serial_485_TX_packet.ul_size = (data_leng + 6);
						// pdc_tx_init(serial_485, &serial_485_TX_packet, NULL);
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
				rx_idx_RS485 = 0;
				break;
			}
			else
			{
				if ((rx_idx_RS485 == (0x06 + data_leng)) && ((-sumatoria_v & 0x000000FF) != (unsigned int)bufer_serial_rx[data_leng + 4]))

				{
					error_code = ERROR_CHK;
					not_ack_RS485();
					rx_idx_RS485 = 0;
					break;
				}
			}

			if ((rx_idx_RS485 == (6 + data_leng)) && ((unsigned int)received_byte == 0xFC))
			{

				if ((-sumatoria_v & 0x000000FF) == (unsigned int)bufer_serial_rx[i])
				{
					if ((port_address + data_leng) < 0x80)
					{
						int port_address_temp = port_address;

						if ((port_address_temp == PASO_MODE_A) && (bufer_serial_rx[4] != 0x00) && !end_pase)
						{
							error_code = ERR_BUSY;
							not_ack_RS485();
							rx_idx_RS485 = 0;
							break;
						}
						if ((port_address_temp == PASO_MODE_B) && (bufer_serial_rx[4] != 0x00) && !end_pase)
						{
							error_code = ERR_BUSY;
							not_ack_RS485();
							rx_idx_RS485 = 0;
							break;
						}
						// configuracion de escenarios no permitidos
						if ((port_address_temp == CONF_ESCENARIO) && ((bufer_serial_rx[4] == 0x00) || (bufer_serial_rx[4] >= 5)))
						{
							error_code = ERR_BAD_DATA;
							not_ack_RS485();
							rx_idx_RS485 = 0;
							break;
						}
						// configuracion de timeout no permitidos
						if ((port_address_temp == CONF_TIMEOUT) && (bufer_serial_rx[4] < 5))
						{
							error_code = ERR_BAD_DATA;
							not_ack_RS485();
							rx_idx_RS485 = 0;
							break;
						}

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

							uart_puts(UART1, (char *)bufer_seria_tx, 5);

							// Pdc	*serial_485;
							// pdc_packet_t serial_485_TX_packet;
							// serial_485 = uart_get_pdc_base(UART1);
							// pdc_enable_transfer(serial_485, PERIPH_PTCR_TXTEN);
							// serial_485_TX_packet.ul_addr = (uint32_t)bufer_seria_tx;
							// serial_485_TX_packet.ul_size = 5;
							// pdc_tx_init(serial_485, &serial_485_TX_packet, NULL);
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

				if ((rx_idx_RS485 == 0x07) && ((-sumatoria_v & 0x000000FF) != (unsigned int)bufer_serial_rx[5]))
				{
					error_code = ERROR_CHK;
					not_ack_RS485();
				}

				if ((rx_idx_RS485 == 7) && ((unsigned char)received_byte == 0xFC))
				{

					if ((-sumatoria_v & 0x000000FF) == (unsigned int)bufer_serial_rx[5])
					{

						bufer_seria_tx[0] = 0xA1;
						bufer_seria_tx[1] = 0x06;
						bufer_seria_tx[2] = 0x00;
						bufer_seria_tx[3] = -((char)(0xA1 ^ 0x06 ^ 0x00));
						bufer_seria_tx[4] = 0xFC;

						uart_puts(UART1, (char *)bufer_seria_tx, 5);

						// Pdc	*serial_485;
						// pdc_packet_t serial_485_TX_packet;
						// serial_485 = uart_get_pdc_base(UART1);
						// pdc_enable_transfer(serial_485, PERIPH_PTCR_TXTEN);
						// serial_485_TX_packet.ul_addr = (uint32_t)bufer_seria_tx;
						// serial_485_TX_packet.ul_size = 5;
						// pdc_tx_init(serial_485, &serial_485_TX_packet, NULL);
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
	WDT->WDT_MR = WDT_MR_WDDIS; // Desactiva watchdog
	sysclk_init();				// Inicializa reloj del sistema
	board_init();				// Inicializa la placa base (ASF)

	esc_init();
	picto_init();
	configure_pins();			// Configura E/S y habilita interrupciones
	configure_uart();			// Configura E/S y habilita interrupciones
	configure_systick();


	// control del pase
	bool control_pase = false;
	bool dif_control_pase = false;

	// manejo de tiempos
	uint32_t last_time = 0;
	bool flag_timeout_pase = false;
	uint32_t timer_while = millis();
	uint32_t timer_while_last = 0;
	uint32_t timer_while_delta = 0;

	// temporizador de configuracion de escenario
	uint32_t value_init_conf = millis();

	uint32_t accumulated_inactive_time = 0;


	// inicializar los solenoide
	esc_action_A(LEFT_PIN_PORT, LEFT_PIN_MASK, escenario_app, 0);
	esc_action_B(RIGHT_PIN_PORT, RIGHT_PIN_MASK, escenario_app, 0);

	bool flag_conf = false;

	while (1)
	{
		// el bucle de while se ejecuta cada 10 ms
		if ((millis() - timer_while) > 10)
		{

			timer_while = millis();

			// --------------------------------------------------------------------------
			// TEMPORIZACION DE STATUS_ALARM
			if (!end_pase)
			{
				timer_while_delta = timer_while - timer_while_last;
				timer_while_last = timer_while;	
	
				if (!position_encoder_moved) {
					// Solo acumula tiempo si no hay movimiento
					accumulated_inactive_time += timer_while_delta;
				} else {
					// Si hubo movimiento, pausa acumulación
					position_encoder_moved = false;
				}
			
				if (accumulated_inactive_time >= VALUE_TIMER_ALARM_PASE) {
					alarm_pase =true;
					accumulated_inactive_time = 0;  
				}
				if (abs(position_encoder) > COUNTER_ENCODER_PICTO)
				{
					picto_action(A,X);
					picto_action(B,X);
				}
			}
			else
			{
				accumulated_inactive_time = 0;
				position_encoder_moved = false;
				alarm_pase = false;

				// reestablecer los pictos como estaban los escenario cuando vuelve a un posicion de inicio.
				picto_action(A,ESC_target(0,escenario_app)^pase_A);
				picto_action(B,ESC_target(1,escenario_app)^pase_B);
			}
			
			

			// Lógica de control de configuración.
			// ---------------------------------------------------------------------------------------------------------
			if (!flag_conf && ((millis() - value_init_conf) > 5000))
			{
				// si no se recibio configuracion de torniquete en 5 segundos, se activa la alarma.
				if ((escenario_app == 0)) // 0 es el valor de SIN CONFIGURACIÓN de torniquete desde la app (usar valores mayores a 0 para los escenarios)
				{
					flag_conf = true;					 // esta variable se usa para indicar que NO se recibio configuracion de torniquete. no se vuelve a activar la alarma.
					port_slots_read[CONF_STATUS] = 0xFF; // Alarma de configuracion.
				}
			}

			// cada vez que cambie escenario_app solo por medio de banco de memoria actualiza los solenoides.
			if (escenario_app != dif_escenario_app)
			{
				dif_escenario_app = escenario_app;

				esc_action_A(LEFT_PIN_PORT, LEFT_PIN_MASK, escenario_app, 0);
				esc_action_B(RIGHT_PIN_PORT, RIGHT_PIN_MASK, escenario_app, 0);
			}

			// Lógica de control del pase del torniquete desde APP.
			// ---------------------------------------------------------------------------------
			if (port_slots_write[PASO_MODE_A] == 0xFF)
			{
				pase_A = true;
				flag_pase_A = true;
				port_slots_write[PASO_MODE_A] = 0x00;
			}
			else if (port_slots_write[PASO_MODE_B] == 0xFF)
			{ // no se debe emitar pasos en los dos sentidos
				pase_B = true;
				flag_pase_B = true;
				port_slots_write[PASO_MODE_B] = 0x00;
			}
			// --------------------------------------------------------------------------------

			// Lógica de configuración desde APP.
			// ---------------------------------------------------------------------------------
			//
			// TODO: confirmacion de escenario de torniquete.
			if (port_slots_write[CONF_ESCENARIO] != 0x00) // usar valores de escenario mayores a 0
			{
				escenario_app = port_slots_write[CONF_ESCENARIO]; // es esta varible se almacena el escenario de torniquete.
				port_slots_write[CONF_ESCENARIO] = 0x00;		  // normalizar el banco de memoria.
				port_slots_read[CONF_STATUS] = 0x00;			  // Alarma de configuracion, nomalizar.
			}

			// TODO: configuración de timeout de pase.
			if (port_slots_write[CONF_TIMEOUT] > 5) // valor mínimo de 5 segundos
			{
				conf_timeout_pase = port_slots_write[CONF_TIMEOUT]; // es esta varible se almacena el timeout de pase.
				port_slots_write[CONF_TIMEOUT] = 0x00;				// normalizar el banco de memoria.
			}
			// ----------------------------------------------------------------------------------

			// Lógica de lectura de datos del torniquete para APP.
			// ---------------------------------------------------------------------------------
			for (size_t i = 0; i < sizeof(acum_pase_A); i++)
			{
				port_slots_read[ACCUMULATOR_A + i] = acum_pase_A >> (i * 8);
			}
			for (size_t i = 0; i < sizeof(acum_pase_B); i++)
			{
				port_slots_read[ACCUMULATOR_B + i] = acum_pase_B >> (i * 8);
			}
			for (size_t i = 0; i < sizeof(acum_pase_fail); i++)
			{
				port_slots_read[CONFIRMACION_FAIL + i] = acum_pase_fail >> (i * 8);
			}
			if (timeout_pase)
			{
				port_slots_read[TIMEOUT_PASO] = 0xFF;
			}
			else
			{
				port_slots_read[TIMEOUT_PASO] = 0x00;
			}

			// TODO: alarma para indicar que torniquete está en posición no permitida.
			// desarollar la logica de alarma de torniquete.
			if (alarm_pase)
			{
				port_slots_read[ALARM_STATUS] = 0xFF;
			}
			else
			{
				port_slots_read[ALARM_STATUS] = 0x00;
			}
			// ---------------------------------------------------------------------------------
			// CONTROL PASE DEL TORNIQUETE.
			// ---------------------------------------------------------------------------------
			// actualizacion del temporizador cuando se activa control_pase
			if (dif_control_pase)
			{
				if (millis() - last_time > conf_timeout_pase * 1000)
				{
					// activar la alarma y pones en modo falso la condicion de control_pase.
					flag_timeout_pase = true;
					timeout_pase = true;
					flag_pase_B = false;
					flag_pase_A = false;
				}
			}
			else
			{
				flag_timeout_pase = false;
			}

			// hay 2 condiciones que regulan el pase por el toniquete cuando el sentido de giro pase_A o pase_B  es true.
			// 	-si el torniquete hace recorrido mas de 85% de la distancia correspodiente al sentido habilitado.
			// 	-si se cumple el timeout.
			if (pase_A)
			{
				control_pase = (position_encoder_last > -COUNTER_ENCODER_PASE_85P) && !flag_timeout_pase;
			}
			if (pase_B)
			{
				control_pase = (position_encoder_last < COUNTER_ENCODER_PASE_85P) && !flag_timeout_pase;
			}

			if (control_pase != dif_control_pase)
			{
				dif_control_pase = control_pase;
				if (control_pase)
				{

					// habilita el intervalo de tiempo posterior y paso del torniquete  una vez.
					if (pase_A)
					{
						esc_action_A(LEFT_PIN_PORT, LEFT_PIN_MASK, escenario_app, 1);
					}
					if (pase_B)
					{
						esc_action_B(RIGHT_PIN_PORT, RIGHT_PIN_MASK, escenario_app, 1);
					}
					last_time = millis();
				}
				else
				{
					// reiniciar el intervalo de tiempo posterior y el paso de torniquete una vez.
					if (pase_A)
					{
						esc_action_A(LEFT_PIN_PORT, LEFT_PIN_MASK, escenario_app, 0);
					}
					if (pase_B)
					{
						esc_action_B(RIGHT_PIN_PORT, RIGHT_PIN_MASK, escenario_app, 0);
					}

					last_time = 0;
					pase_A = false;
					pase_B = false;
				}
			}
		}
	}
}

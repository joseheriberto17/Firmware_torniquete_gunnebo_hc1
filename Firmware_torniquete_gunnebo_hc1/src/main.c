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
#include "uart_custom.h"

void configure_pins(void);
void handle_encoder(const uint32_t id, const uint32_t mask);
void configure_uart(void);
uint8_t read_AB(void);

// definicion de los parametros de la aplicacion
#define MAX_REVERSE_TOLERANCE 12 // define cuantos contadores puede contar si el torniquete se devuelve
#define COUNTER_ENCODER_PASE 60 // define el valor minimo que tiene que contar position_encoder para validar un paso.

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
#define UART_SERIAL_BAUDRATE       19200
#define UART_SERIAL_BAUDRATE_485   115200 /// para pruebas 19200, para produccion 115200
#define UART_SERIAL_CHANNEL_MODE   UART_MR_CHMODE_NORMAL
#define UART_SERIAL_MODE           UART_MR_PAR_NO

#define PINS_UART1          (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
#define PINS_UART1_FLAGS    (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART1_MASK     (PIO_PB2 | PIO_PB3)
#define PINS_UART1_PIO      PIOB
#define PINS_UART1_ID       ID_PIOB
#define PINS_UART1_TYPE     PIO_PERIPH_A
#define PINS_UART1_ATTR     PIO_DEFAULT

// Tabla de transición de cuadratura para decodificador X4.
const int8_t qdec_table[4][4] = {
	// to:        00  01  10  11
	/*from 00*/ {0, -1, +1, 0},
	/*from 01*/ {+1, 0, 0, -1},
	/*from 10*/ {-1, 0, 0, +1},
	/*from 11*/ {0, +1, -1, 0}};

	// variables globales
	// ------------------------------------------------------------------------
	// conteo de micropasos del encoder
	volatile int32_t position_encoder      = 0;	    // define el micropaso actual del encoder relativo al ultimo final de carrera del torniquete.
	volatile int32_t position_encoder_last = 0;     // define el micropaso maximo del encoder relativo la ultimo final de carrera del torniquete, durante el proceso de un paso.
	volatile bool    dir_encoder_new       = true;  // direccion del encoder actual (valor entre 0 y 1).
	volatile int8_t  delta_last            = 0;     // direccion del ultima del encoder (valor -1 y +1).
	volatile uint8_t last_state_encoder    = 0;	    // define la  ultima secuencia de los encoder (A/B).
	volatile bool    end_pase              = true;  // indica si position_encoder esta en un final de carrera.
	volatile bool    change_dir_detected   = false; // indica si durante el proceso de un paso el torniquete se devolvio superando el valor de tolerancia (MAX_REVERSE_TOLERANCE).
	volatile bool    confirm_pase          = false;	// valida un paso del torniquete
	volatile bool    last_confirm_pase     = false;	// valida un paso del torniquete
	volatile bool    pase                  = false;	// control de un pase del torniquete
	volatile bool    acum_pase             = 0 ;	// incrementador de paso
	volatile int8_t  counter_pase		   = 0; //
	

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
		if (delta != 0)
		{
			position_encoder += delta;
			dir_encoder_new = (delta > 0);

			// Si estamos en el final de carrera
			if (end_pase)
			{
				// validar paso completo
				if (change_dir_detected == false)
				{
					position_encoder = 0; // Reiniciamos la posición
					if (position_encoder_last > COUNTER_ENCODER_PASE)
					{
						pio_toggle_pin(PIC_1_PIN); // Giro en sentido horario.
					}
					if (position_encoder_last < -COUNTER_ENCODER_PASE)
					{
						pio_toggle_pin(PIC_2_PIN); // Giro en sentido antihorario.
						acum_pase = true ;
						
					}
				}

				// Reseteo de variables
				position_encoder_last = 0;
				change_dir_detected = false;

				//  Apagamos los LEDs de dirección (SEN_I y SEN_S).
				pio_clear(LED_SEN_I_PIN_PORT, LED_SEN_I_PIN_MASK);
				pio_clear(LED_SEN_S_PIN_PORT, LED_SEN_S_PIN_MASK);
			}
			else
			{
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
					change_dir_detected = true;
				}

				// Control de salidas:
				// - Si hubo reversa: ambos LEDs encendidos (advertencia change_dir_detected = true).
				// - Si avanza en sentido horario: LED SEN_I.
				// - Si avanza en sentido antihorario: LED SEN_S.
				if (change_dir_detected)
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
		last_state_encoder = new_state_encoder;
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
		pio_configure(LEFT_PIN_PORT,  PIO_OUTPUT_0, LEFT_PIN_MASK,  PIO_DEFAULT);

		// Entrada GPIO
		pio_configure(SEN_I_PIN_PORT, PIO_INPUT, SEN_I_PIN_MASK, PIO_DEFAULT);
		pio_configure(SEN_S_PIN_PORT, PIO_INPUT, SEN_S_PIN_MASK, PIO_DEFAULT);
		pio_configure(INDEX_PIN_PORT,   PIO_INPUT, INDEX_PIN_MASK,   PIO_DEFAULT);
		pio_configure(CONF_A_PIN_PORT,  PIO_INPUT, CONF_A_PIN_MASK,  PIO_DEFAULT);

		pio_configure_interrupt(SEN_I_PIN_PORT, SEN_I_PIN_MASK, PIO_IT_EDGE);
		pio_configure_interrupt(SEN_S_PIN_PORT, SEN_S_PIN_MASK, PIO_IT_EDGE);
		pio_handler_set(PIOB, ID_PIOB, SEN_I_PIN_MASK | SEN_S_PIN_MASK, PIO_IT_EDGE, handle_encoder);

		// Habilitar la interrupcion en el periferico y en el NVIC
		pio_enable_interrupt(PIOB, SEN_I_PIN_MASK | SEN_S_PIN_MASK);
		NVIC_EnableIRQ(PIOB_IRQn);
	}

	void configure_uart(void)
	{
		pmc_enable_periph_clk (ID_UART1);
		
		// set the pins to use the uart peripheral
		pio_configure(PINS_UART1_PIO, PINS_UART1_TYPE, PINS_UART1_MASK, PINS_UART1_ATTR);

		sam_uart_opt_t uart1_settings = {
			.ul_mck = sysclk_get_cpu_hz(),
			.ul_baudrate = 19200,
			.ul_mode = UART_SERIAL_MODE  // Modo sin paridad, 8N1
		};
		
		uart_init(UART1,&uart1_settings);
		NVIC_EnableIRQ(UART1_IRQn);
		uart_enable(UART1);
		
	}
	
	int main(void)
	{
		sysclk_init();				// Inicializa reloj del sistema
		WDT->WDT_MR = WDT_MR_WDDIS; // Desactiva watchdog
		board_init();				// Inicializa la placa base (ASF)
		configure_pins();			// Configura E/S y habilita interrupciones
		configure_uart();			// Configura E/S y habilita interrupciones

		last_state_encoder = read_AB(); // Guarda estado inicial del encoder
		
		uart_puts(UART1, "\x1B[2J\x1B[H");  // Limpia toda pantalla y va al tope
		
		char buffer[64];


		while (1)
		{
			confirm_pase = pio_get(CONF_A_PIN_PORT,PIO_INPUT,CONF_A_PIN_MASK);
			if (confirm_pase != last_confirm_pase)
			{
				if (confirm_pase == 1)
				{
					pase = true; // flando de subida
					
				}
				
				last_confirm_pase = confirm_pase;
			}
			
			if (acum_pase)
			{
				acum_pase = 0;
				uart_puts(UART1, "\x1B[2J\x1B[H");  // Limpia toda pantalla y va al tope
				snprintf(buffer, sizeof(buffer),"Giro sendido A: %u\r\n", counter_pase);
				uart_puts(UART1, buffer);  // salida formateada
				
				if (counter_pase < 9)
				{
					pase = true;
					counter_pase++;
				}
				else
				{
					counter_pase = 0;
				}				
			}			
			
			// salida de paso de torniquete
			if(pase && (abs(position_encoder_last) < 50) )
			{
				pio_set(RIGHT_PIN_PORT,RIGHT_PIN_MASK);
				pio_set(LEFT_PIN_PORT,LEFT_PIN_MASK);
			}
			else
			{
				pio_clear(RIGHT_PIN_PORT,RIGHT_PIN_MASK);
				pio_clear(LEFT_PIN_PORT,LEFT_PIN_MASK);
				pase = false;
			}
		}
	}
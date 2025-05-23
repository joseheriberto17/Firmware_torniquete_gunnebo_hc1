/*
 * Escenarios.c
 *
 * Created: 19/5/2025 13:47:12
 *  Author: GESTION_LABORATORIO
 */ 
#include <asf.h>
#include "Escenarios.h"


// Pin de Entrada SW2_1
#define SW2_1_PIN PIO_PA23_IDX
#define SW2_1_PIN_MASK PIO_PA23
#define SW2_1_PIN_PORT PIOA

// Pin de Entrada SW2_2
#define SW2_2_PIN PIO_PA22_IDX
#define SW2_2_PIN_MASK PIO_PA22
#define SW2_2_PIN_PORT PIOA

// Pin de Entrada SW2_3
#define SW2_3_PIN PIO_PA19_IDX
#define SW2_3_PIN_MASK PIO_PA19
#define SW2_3_PIN_PORT PIOA

// Pin de Entrada SW2_4
#define SW2_4_PIN PIO_PA21_IDX
#define SW2_4_PIN_MASK PIO_PA21
#define SW2_4_PIN_PORT PIOA



 void configuracion_escenario(void)
{
	// lectura de SW2_1
	uint8_t A = pio_get(SW2_1_PIN_PORT, PIO_INPUT, SW2_1_PIN_MASK) ? 1 : 0;
	uint8_t B = pio_get(SW2_2_PIN_PORT, PIO_INPUT, SW2_2_PIN_MASK) ? 1 : 0;
	uint8_t C = pio_get(SW2_3_PIN_PORT, PIO_INPUT, SW2_3_PIN_MASK) ? 1 : 0;
	uint8_t D = pio_get(SW2_4_PIN_PORT, PIO_INPUT, SW2_4_PIN_MASK) ? 1 : 0;

	// return (A << 3) | (B << 2) | (C << 1) | D;
	if (A)
	{
		pio_set()
	}
	else
	{
		/* code */
	}
	
	typedef struct {
		char Name[10];
		uint8_t Addr;
		uint8_t Value;      
		bool Update; 
	} registers;
	

}


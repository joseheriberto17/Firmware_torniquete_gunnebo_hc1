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

// el switche sw_2 define la posicion inicial que esta el escenario,
// el banco de memoria config_esc define el escenario que quiero que trabaje.

// valor de sw_2:
// libre = ON;
// blooqueado = OFF;
const bool esc_table[4][2] = {
	// A		 | B
	/* libre 	 | libre     */	{1,1},
	/* libre 	 | bloqueado */	{1,0},
	/* bloqueado | libre	 */	{0,1},
	/* bloqueado | bloqueado */	{0,0}
};

// CONMUTACION DE LOS SOLENOIDES.
// -------------------------------------------------------------------------------
// habilitar y deshabilitar el solenoide deacuerdo al escenario definido en el banco de memoria
// y el switche sw_2 para el  solenoide A.
void esc_action_A(Pio *p_pio, const uint32_t ul_mask,uint8_t esc_app, bool level){

	bool ESC_A = pio_get(SW2_1_PIN_PORT, PIO_INPUT, SW2_1_PIN_MASK) ? 0 : 1;
	bool ESC_app_A = esc_table[esc_app][0];
	bool Poralidad_SOL = ESC_A ^  ESC_app_A; // define la poralidad del solenoide que va a trabajar.
	

	if (Poralidad_SOL ^ level)
	{
		pio_set(p_pio, ul_mask);
	}
	else
	{
		pio_clear(p_pio, ul_mask);
	}
}

// habilitar y deshabilitar el solenoide deacuerdo al escenario definido en el banco de memoria
// y el switche sw_2 para el  solenoide B.
void esc_action_B(Pio *p_pio, const uint32_t ul_mask,uint8_t esc_app, bool level){

	bool ESC_B = pio_get(SW2_2_PIN_PORT, PIO_INPUT, SW2_2_PIN_MASK) ? 0 : 1;
	bool ESC_app_B = esc_table[esc_app][1];
	bool Poralidad_SOL = ESC_B ^  ESC_app_B; // define la poralidad del solenoide que va a trabajar.

	if (Poralidad_SOL ^ level)
	{
		pio_set(p_pio, ul_mask);
	}
	else
	{
		pio_clear(p_pio, ul_mask);
	}
}
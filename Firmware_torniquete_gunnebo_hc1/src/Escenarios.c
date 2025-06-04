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


// tabla de verdad absoluta, define los estados logicos alto y bajos.
const bool esc_table[4][2] = {
	// A		 | B
	/* bloqueado | bloqueado */	{0,0},
	/* bloqueado | libre	 */	{0,1},
	/* libre 	 | bloqueado */	{1,0},
	/* libre 	 | libre     */	{1,1}
};

uint8_t Read_SW_2_Esc(void)
{
	bool esc_init_1 = pio_get(SW2_1_PIN_PORT, PIO_INPUT, SW2_1_PIN_MASK) ? 0 : 1;
	bool esc_init_2 = pio_get(SW2_2_PIN_PORT, PIO_INPUT, SW2_2_PIN_MASK) ? 0 : 1;
	return (esc_init_1 << 1) | esc_init_2;
}

// CONMUTACION DE LOS ESCENARIOS.
// ---------------------------------------------------------------------------------------------------------
// la iteracion entre los escenarios se define entre banco de memoria (esc_app) y sw_2 (pines 1 y 2) si no se define por esc_app.
// se tiene que conocer el escenario donde se mantiene los solenoides relajados y definirlo en sw_2 (pines 3 y 4).
void esc_action_A(Pio *p_pio, const uint32_t ul_mask,uint8_t esc_app, bool level){
	bool ESC_target = 0;
	bool Esc_init_A = 0;
	bool Poralidad_SOL = 0;

	if (esc_app != 0)
	{
		// iteracion de la tabla entre los escenario definidos por el banco de memoria.
		ESC_target = esc_table[esc_app-1][0];
	} else {
		// iteracion de la tabla entre los escenarios definidos por el switche sw_2 (pines 1 y 2).
		ESC_target = esc_table[Read_SW_2_Esc()][0];
	}

	Esc_init_A = pio_get(SW2_3_PIN_PORT, PIO_INPUT, SW2_3_PIN_MASK) ? 0 : 1;
	Poralidad_SOL = Esc_init_A ^  ESC_target; // define la poralidad del solenoide que va a trabajar.

	if (Poralidad_SOL ^ level)
	{
		pio_set(p_pio, ul_mask);
	} else {
		pio_clear(p_pio, ul_mask);
	}
}


void esc_action_B(Pio *p_pio, const uint32_t ul_mask,uint8_t esc_app, bool level){
	bool ESC_target = 0;
	bool Esc_init_B = 0;
	bool Poralidad_SOL = 0;

	if (esc_app != 0)
	{
		// iteracion de la tabla entre los escenario definidos por el banco de memoria.
		ESC_target = esc_table[esc_app-1][1];
	} else {
		// iteracion de la tabla entre los escenarios definidos por el switche sw_2 (pines 1 y 2).
		ESC_target = esc_table[Read_SW_2_Esc()][1];
	}

	Esc_init_B = pio_get(SW2_4_PIN_PORT, PIO_INPUT, SW2_4_PIN_MASK) ? 0 : 1;
	Poralidad_SOL = Esc_init_B ^  ESC_target; // define la poralidad del solenoide que va a trabajar.

	if (Poralidad_SOL ^ level)
	{
		pio_set(p_pio, ul_mask);
	} else {
		pio_clear(p_pio, ul_mask);
	}
}

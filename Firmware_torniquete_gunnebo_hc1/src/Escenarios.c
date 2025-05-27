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



const uint32_t timeout_pase_table[4] = {
										10000, /* millisegundos*/
	 									20000, /* millisegundos*/
	 									25000, /* millisegundos*/
	 									30000};/* millisegundos*/

// devuelve el valor de timeout definido por los pines sw_2 segun la tabla
uint32_t esc_read_time(void)
{
	uint8_t TIM_A = pio_get(SW2_3_PIN_PORT, PIO_INPUT, SW2_3_PIN_MASK) ? 1 : 0;
	uint8_t TIM_B = pio_get(SW2_4_PIN_PORT, PIO_INPUT, SW2_4_PIN_MASK) ? 1 : 0;
	uint8_t TIM_status_SW_2 = (TIM_A << 1) | TIM_B;

	return timeout_pase_table[TIM_status_SW_2];
}


// PORALIDAD DE LOS SOLENOIDES
// -------------------------------------------------------------------------------
// habilitar y deshabilitar el solenoide deacuerdo a la poralidad definido en sw_2
// para el  solenoide A.
void SOL_action_A(Pio *p_pio, const uint32_t ul_mask,bool set_SOL)
{
	bool get_SOL = pio_get(SW2_2_PIN_PORT, PIO_INPUT, SW2_2_PIN_MASK) ? 0 : 1;

	if (get_SOL ^ set_SOL )
	{
		pio_set(p_pio, ul_mask);
	}
	else
	{
		pio_clear(p_pio, ul_mask);
	}
}
// habilitar y deshabilitar el solenoide deacuerdo a la poralidad definido en sw_2
// para el  solenoide B.
void SOL_action_B(Pio *p_pio, const uint32_t ul_mask,bool set_SOL)
{
	bool get_SOL = pio_get(SW2_1_PIN_PORT, PIO_INPUT, SW2_1_PIN_MASK) ? 0 : 1;

	if (get_SOL ^ set_SOL )
	{
		pio_set(p_pio, ul_mask);
	}
	else
	{
		pio_clear(p_pio, ul_mask);
	}
}

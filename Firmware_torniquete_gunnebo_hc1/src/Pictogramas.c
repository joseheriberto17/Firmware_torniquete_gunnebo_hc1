/*
 * Pictogramas.c
 *
 * Created: 17/06/2025 3:12:33 p. m.
 *  Author: USUARIO
 */
#include <asf.h>
#include "Pictogramas.h"

// Pin de salida del led PICT_1.
#define PIC_1_PIN PIO_PA8_IDX
#define PIC_1_PIN_MASK PIO_PA8
#define PIC_1_PIN_PORT PIOA

// Pin de salida del led PICT_2.
#define PIC_2_PIN PIO_PA7_IDX
#define PIC_2_PIN_MASK PIO_PA7
#define PIC_2_PIN_PORT PIOA

// Pin de salida del led PICT_1.
#define PIC_3_PIN PIO_PA6_IDX
#define PIC_3_PIN_MASK PIO_PA6
#define PIC_3_PIN_PORT PIOA

// Pin de salida del led PICT_2.
#define PIC_4_PIN PIO_PA5_IDX
#define PIC_4_PIN_MASK PIO_PA5
#define PIC_4_PIN_PORT PIOA

void picto_init(void)
{
    // Entradas SW2_1 y SW2_2
    pio_configure(PIC_1_PIN_PORT, PIO_OUTPUT_0, PIC_1_PIN_MASK, PIO_DEFAULT);
    pio_configure(PIC_2_PIN_PORT, PIO_OUTPUT_0, PIC_2_PIN_MASK, PIO_DEFAULT);

    // Entradas SW2_3 y SW2_4
    pio_configure(PIC_3_PIN_PORT, PIO_OUTPUT_0, PIC_3_PIN_MASK, PIO_DEFAULT);
    pio_configure(PIC_4_PIN_PORT, PIO_OUTPUT_0, PIC_4_PIN_MASK, PIO_DEFAULT);
}

void picto_action(Lado lado_pase,bool level)
{
    if (level)
    {
        picto_state(lado_pase, V);
    }
    else
    {
        picto_state(lado_pase, X);
    }

}

// determina el estado de los picto en cada lado.
// lado: A, B
// state: X, V
void picto_state(Lado lado, State state)
{
    if (lado == A)
    {
        if (state == V)
        {
            // en B = (V)
            pio_set(PIC_1_PIN_PORT, PIC_1_PIN_MASK);
            pio_clear(PIC_2_PIN_PORT, PIC_2_PIN_MASK);
        }
        if (state == X)
        {
            // en B = (X)
            pio_clear(PIC_1_PIN_PORT, PIC_1_PIN_MASK);
            pio_set(PIC_2_PIN_PORT, PIC_2_PIN_MASK);
        }
    }

    if (lado == B)
    {
        if (state == V)
        {
            // en A = (V)
            pio_set(PIC_3_PIN_PORT, PIC_3_PIN_MASK);
            pio_clear(PIC_4_PIN_PORT, PIC_4_PIN_MASK);
        }
        if (state == X)
        {
            // en A = (X)
            pio_clear(PIC_3_PIN_PORT, PIC_3_PIN_MASK);
            pio_set(PIC_4_PIN_PORT, PIC_4_PIN_MASK);
        }
    }
}
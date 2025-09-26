#include "encoder.h"

/* ── variables internas ──────────────────────────────────── */
volatile int32_t position = 0;
volatile uint8_t last_state = 0;
volatile bool end_pase = false;
volatile bool new_dir = true;

/* Tabla de transición x4 – solo visible en este módulo */
const int8_t qdec_table[4][4] = {
    /*   to: 00  01  10  11      */
    /*00*/ {0, -1, +1, 0},
    /*01*/ {+1, 0, 0, -1},
    /*10*/ {-1, 0, 0, +1},
    /*11*/ {0, +1, -1, 0}};

/* Prototipo interno */
uint8_t read_AB(void);
void encoder_isr(uint32_t id, uint32_t mask);

/* ── implementación pública ──────────────────────────────── */
void encoder_init(void)
{
    /* Entradas encoder */
    pio_configure(PIN_ENC_A_PORT, PIO_INPUT, PIN_ENC_A_MASK, PIO_DEFAULT);
    pio_configure(PIN_ENC_B_PORT, PIO_INPUT, PIN_ENC_B_MASK, PIO_DEFAULT);
    pio_configure(PIN_ENC_C_PORT, PIO_INPUT, PIN_ENC_C_MASK, PIO_DEFAULT);

    /* Configurar interrupción en ambos pines */
    pio_enable_interrupt(PIN_ENC_A_PORT, PIN_ENC_A_MASK | PIN_ENC_B_MASK);
    pio_handler_set(PIN_ENC_A_PORT, ID_PIOC, PIN_ENC_A_MASK | PIN_ENC_B_MASK, PIO_IT_EDGE, encoder_isr);

    NVIC_EnableIRQ(PIOC_IRQn);

    last_state = read_AB();
}

int32_t encoder_get_position(void) { return position; }
bool encoder_get_direction(void) { return new_dir; }
bool end_pase_get(void) { return end_pase; }

/* ── rutina de ISR privada ───────────────────────────────── */
void encoder_isr(uint32_t id, uint32_t mask)
{
    (void)id; /* sin usar */
    (void)mask;

    uint8_t new_state = read_AB();
    int8_t delta = qdec_table[last_state][new_state];

    if (delta != 0)
    {
        position += delta;
        new_dir = (delta > 0);
    }
    last_state = new_state;

    end_pase = pio_get(PIN_ENC_C_PORT, PIO_INPUT, PIN_ENC_C_MASK) ? 1 : 0;
}

/* Lee los dos canales y los empaqueta en 2 bits */
uint8_t read_AB(void)
{
    uint8_t a = pio_get(PIN_ENC_A_PORT, PIO_INPUT, PIN_ENC_A_MASK) ? 1 : 0;
    uint8_t b = pio_get(PIN_ENC_B_PORT, PIO_INPUT, PIN_ENC_B_MASK) ? 1 : 0;
    return (b << 1) | a;
}
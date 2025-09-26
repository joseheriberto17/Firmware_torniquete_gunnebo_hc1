#ifndef ENCODER_H_
#define ENCODER_H_

#include <asf.h>

/* Pines ── fase A PC25 (D4), fase B PC24 (D5) y fase C PC23 (D6) ------------- */
#define PIN_ENC_A_IDX   PIO_PC23_IDX       /* D7  */
#define PIN_ENC_A_MASK  PIO_PC23
#define PIN_ENC_A_PORT  PIOC

#define PIN_ENC_B_IDX   PIO_PC24_IDX       /* D6  */
#define PIN_ENC_B_MASK  PIO_PC24
#define PIN_ENC_B_PORT  PIOC

#define PIN_ENC_C_IDX   PIO_PC25_IDX       /* D5  */
#define PIN_ENC_C_MASK  PIO_PC25
#define PIN_ENC_C_PORT  PIOC

/* API ------------------------------------------------------- */
void  encoder_init(void);
int32_t encoder_get_position(void);
bool   encoder_get_direction(void);
bool end_pase_get(void);

#endif /* ENCODER_H_ */
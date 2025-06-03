/*
 * Escenarios.h
 *
 * Created: 19/5/2025 13:47:52
 *  Author: GESTION_LABORATORIO
 */ 

#ifndef ESCENARIOS_H_
#define ESCENARIOS_H_

uint32_t esc_read_time(void);
void esc_action_A(Pio *p_pio, const uint32_t ul_mask,uint8_t esc_app, bool level);
void esc_action_B(Pio *p_pio, const uint32_t ul_mask,uint8_t esc_app, bool level);
#endif /* ESCENARIOS_H_ */
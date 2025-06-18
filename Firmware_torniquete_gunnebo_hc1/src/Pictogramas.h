/*
 * Pictogramas.h
 *
 * Created: 17/06/2025 3:13:20 p. m.
 *  Author: USUARIO
 */ 


#ifndef PICTOGRAMAS_H_
#define PICTOGRAMAS_H_

typedef enum
{
    A,
    B
} Lado;

typedef enum
{
    X,
    V
} State;

void picto_init(void);
void picto_action(Lado lado_pase,bool level);
void picto_state(Lado lado, State state);

#endif /* PICTOGRAMAS_H_ */
#include <asf.h>

//pines de entrada
#define PHASE_A_PIN PIO_PB11_IDX
#define PHASE_A_PIN_MASK PIO_PB11
#define PHASE_A_PIN_PORT PIOB

#define PHASE_B_PIN PIO_PB13_IDX
#define PHASE_B_PIN_MASK PIO_PB13
#define PHASE_B_PIN_PORT PIOB

#define INDEX_PIN PIO_PB14_IDX
#define INDEX_PIN_MASK PIO_PB14
#define INDEX_PIN_PORT PIOB

//pines de salida
#define SEN_I_PIN PIO_PB0_IDX
#define SEN_I_PIN_MASK PIO_PB0
#define SEN_I_PIN_PORT PIOB

#define SEN_S_PIN PIO_PB1_IDX
#define SEN_S_PIN_MASK PIO_PB1
#define SEN_S_PIN_PORT PIOB

#define PIC_1_PIN PIO_PA8_IDX
#define PIC_1_PIN_MASK PIO_PA8
#define PIC_1_PIN_PORT PIOA

#define PIC_2_PIN PIO_PA8_IDX
#define PIC_2_PIN_MASK PIO_PA8
#define PIC_2_PIN_PORT PIOA

// parametros de la aplicacion
#define MAX_REVERSE_TOLERANCE 12 // define cuantos contadores puede contar si el torniquete se devuelve
#define DIR_CLOCKWISE true // sentido de giro

const int8_t qdec_table[4][4] = {
	// to:  00   01   10   11
	/*from 00*/ {  0, -1, +1,  0 },
	/*from 01*/ { +1,  0,  0, -1 },
	/*from 10*/ { -1,  0,  0, +1 },
	/*from 11*/ {  0, +1, -1,  0 }
};



// variables globales
volatile int32_t position_encoder = 0;
volatile int32_t position_encoder_last = 0;
volatile int32_t position_encoder_reverse = 0;

volatile uint8_t last_state_encoder = 0;
volatile bool dir_encoder_new = true;
volatile int8_t delta_last = 0;
volatile bool end_pase = true;
volatile bool clockwise = true;
volatile bool change_dir_detected = false;



// prototipos de funciones
void configure_pins(void);
void handle_encoder(const uint32_t id, const uint32_t mask);
uint8_t read_AB(void);

uint8_t read_AB(void) {
	uint8_t A = pio_get(PHASE_A_PIN_PORT, PIO_INPUT, PHASE_A_PIN_MASK) ? 1 : 0;
	uint8_t B = pio_get(PHASE_B_PIN_PORT, PIO_INPUT, PHASE_B_PIN_MASK) ? 1 : 0;
	return (A << 1) | B;
}

// Handler llamado por fase A o fase B, encoder tipo X4
void handle_encoder(const uint32_t id, const uint32_t mask) {
	// lectura de los encoder
	uint8_t new_state_encoder = read_AB();
	end_pase = pio_get(INDEX_PIN_PORT, PIO_INPUT, INDEX_PIN_MASK) ? 1 : 0;
	
	
	int8_t delta = qdec_table[last_state_encoder][new_state_encoder];// validar el conteo de encoder
	if (delta != 0)
	{
		position_encoder += delta;
		dir_encoder_new = (delta > 0);
		
		if (end_pase) // Estas en final de carrera
		{
			//validar paso completo
			
			if ((change_dir_detected == false) && (abs(position_encoder_last) > 60) )
			{
				pio_toggle_pin(PIC_1_PIN);
				position_encoder = 0; // reinicia el contador de posicion
			}
			
			position_encoder_last= 0; // reinicias
			change_dir_detected = false; // ignora los cambio de direccion
			delta_last = delta; // almacenas el ultimo valor
			
		}
		else
		{
			
			if (abs(position_encoder_last) < abs(position_encoder))
			{
				position_encoder_last = position_encoder;
			}
			
			if (abs(position_encoder_last - position_encoder) > MAX_REVERSE_TOLERANCE)
			{
				position_encoder = 0;
				change_dir_detected = true;
			}
			
			
			
			
			
			
			
			
			
			
			
			
			//if (clockwise == !(delta > 0)) // mientra gire en en el sentido correcto
			//{
			//// si vas en el sentido de giro definido , cuenta posicion y direccion
			//dir_encoder_new = (delta > 0);
			//}
			//
			//if (clockwise)
			//{
			//
			//if (position_encoder_last > position_encoder)
			//{
			//position_encoder_last = position_encoder;
			//}
			//
			//}
			//else
			//{
			//if (position_encoder_last < position_encoder)
			//{
			//position_encoder_last = position_encoder;
			//}
			//}
			//
			//if (abs(position_encoder_last - position_encoder) > MAX_REVERSE_TOLERANCE)
			//{
			//position_encoder = 0;
			//change_dir_detected = true;
			//}
			//
			
			
			//if (clockwise == !(delta > 0)) // mientra gire en en el sentido correcto
			//{
			//// si vas en el sentido de giro definido , cuenta posicion y direccion
			//position_encoder += delta;
			//new_dir_encoder = (delta > 0);
			//position_encoder_reverse = 0;
			//
			//}
			//else
			//{
			//position_encoder_reverse++;
			//if (position_encoder_reverse > MAX_REVERSE_TOLERANCE)
			//{
			//position_encoder = 0;   // anular valor de posicion
			//change_dir_detected = true; // activar el detecto devolucion invalida
			//}
			//}
		}
	}
	
	
	if (change_dir_detected)
	{
		pio_set(SEN_I_PIN_PORT, SEN_I_PIN_MASK);
		pio_set(SEN_S_PIN_PORT, SEN_S_PIN_MASK);
	}
	else if (dir_encoder_new)
	{
		pio_set(SEN_I_PIN_PORT, SEN_I_PIN_MASK);
		pio_clear(SEN_S_PIN_PORT, SEN_S_PIN_MASK);
	}
	else
	{
		pio_clear(SEN_I_PIN_PORT, SEN_I_PIN_MASK);
		pio_set(SEN_S_PIN_PORT, SEN_S_PIN_MASK);
	}
	
	last_state_encoder = new_state_encoder;
}





void configure_pins(void) {
	// Activar el reloj del periférico PIOB
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOA);

	// Salidas GPIO
	pio_configure(SEN_I_PIN_PORT, PIO_OUTPUT_0, SEN_I_PIN_MASK , PIO_DEFAULT);
	pio_configure(SEN_S_PIN_PORT, PIO_OUTPUT_0, SEN_S_PIN_MASK , PIO_DEFAULT);
	pio_configure(PIC_1_PIN_PORT, PIO_OUTPUT_0, PIC_1_PIN_MASK , PIO_DEFAULT);

	
	// Entrada GPIO
	pio_configure(PHASE_A_PIN_PORT,PIO_INPUT,PHASE_A_PIN_MASK,PIO_DEFAULT);
	pio_configure(PHASE_B_PIN_PORT,PIO_INPUT,PHASE_B_PIN_MASK,PIO_DEFAULT);
	pio_configure(INDEX_PIN_PORT,PIO_INPUT,INDEX_PIN_MASK,PIO_DEFAULT);
	pio_configure_interrupt(PHASE_A_PIN_PORT,PHASE_A_PIN_MASK, PIO_IT_EDGE);
	pio_configure_interrupt(PHASE_B_PIN_PORT,PHASE_B_PIN_MASK, PIO_IT_EDGE);
	pio_handler_set(PIOB, ID_PIOB, PHASE_A_PIN_MASK|PHASE_B_PIN_MASK, PIO_IT_EDGE, handle_encoder);
	
	// Habilitar la interrupción en el periférico y en el NVIC
	pio_enable_interrupt(PIOB, PHASE_A_PIN_MASK|PHASE_B_PIN_MASK);
	NVIC_EnableIRQ(PIOB_IRQn);
}

int main(void) {
	sysclk_init();
	
	// deshabilitar el watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	board_init();
	configure_pins();
	
	last_state_encoder = read_AB();
	
	while (1)
	{
		
		
	}
}
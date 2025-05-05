/*
* Autor: jose heriberto
* Fecha de creaccion:
* Objetivo: Firmware para interactuar con la tarjeta de un torniquete gunnebo
*	-hay 3 sensores de  efecto hall disponible, dos para paso preciso y otro para indicar el paso del torniquete en si.
*	-2 solenoide con entrada a 24 v.
*
* Presente: sensar senal de encoder  y verlo directamente en 2 led en modo polling
*
* Futuro: hacer lo mismo pero con manejo de  interrupciones.
*
* Notas:
* Arduino Due Digital Pin Mapping (D0–D13)
*
* +--------------+----------------+
* | Arduino PIN  | SAM3X8E Pin    |
* +--------------+----------------+
* | D0           | PA8            |
* | D1           | PA9            |
* | D2           | PB25           |
* | D3           | PC28           |
* | D4           | PC26/PA29      |
* | D5           | PC25           |
* | D6           | PC24           |
* | D7           | PC23           |
* | D8           | PC22           |
* | D9           | PC21           |
* | D10          | PC29/PA28      |
* | D11          | PD7            |
* | D12          | PD8            |
* | D13 (LED)    | PB27           |
* +--------------+----------------+	
*/

// salidas led
#define PIN_D3 PIO_PC28_IDX
#define PIN_D3_MASK PIO_PC28
#define PIN_D3_PORT PIOC

#define PIN_D2 PIO_PB25_IDX
#define PIN_D2_MASK PIO_PB25
#define PIN_D2_PORT PIOB

// entrada encoder
#define PIN_D4 PIO_PC26_IDX
#define PIN_D4_MASK PIO_PC26
#define PIN_D4_PORT PIOC

#define PIN_D5 PIO_PC25_IDX
#define PIN_D5_MASK PIO_PC25
#define PIN_D5_PORT PIOC

#include <asf.h>

void configure_pins(void);

void configure_pins(void)
{
	// Habilitar el reloj para el PIOB
	pmc_enable_periph_clk(ID_PIOB);	
	pmc_enable_periph_clk(ID_PIOC);	
	
	// salida
	pio_configure(PIN_D2_PORT,PIO_OUTPUT_0,PIN_D2_MASK,PIO_DEFAULT);
	pio_configure(PIN_D3_PORT,PIO_OUTPUT_0,PIN_D3_MASK,PIO_DEFAULT);
	
	// entrada
	pio_configure(PIN_D4_PORT,PIO_INPUT,PIN_D4_MASK,PIO_DEFAULT);
	pio_configure(PIN_D5_PORT,PIO_INPUT,PIN_D5_MASK,PIO_DEFAULT);
	
}


int main (void)
{
	sysclk_init();
	board_init();
	configure_pins();
	
	uint8_t encoder_1=0;
	uint8_t encoder_2=0;
	while (1) 
	{
		encoder_1 = pio_get(PIN_D5_PORT,PIO_INPUT,PIN_D5_MASK);
		encoder_2 = pio_get(PIN_D4_PORT,PIO_INPUT,PIN_D4_MASK);

		if (encoder_1)
		{
			pio_set(PIN_D3_PORT,PIN_D3_MASK);
		} 
		else
		{
			pio_clear(PIN_D3_PORT,PIN_D3_MASK);		
		}
		
		if (encoder_2)
		{
			pio_set(PIN_D2_PORT,PIN_D2_MASK);
		}
		else
		{
			pio_clear(PIN_D2_PORT,PIN_D2_MASK);
		}
		delay_ms(10);
	}
}

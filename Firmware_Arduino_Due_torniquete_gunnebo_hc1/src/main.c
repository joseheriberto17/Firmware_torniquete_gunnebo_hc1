/*
* Autor: jose heriberto
* Fecha de creaccion:
* Objetivo: Firmware para interactuar con la tarjeta de un torniquete gunnebo
*	-hay 3 sensores de  efecto hall disponible, dos para paso preciso y otro para indicar el paso del torniquete en si.
*	-2 solenoide con entrada a 24 v.
*
* Presente: inicializando el proyecto, empezando haciendo un blink con driver PIO.
*
* Futuro: hacer polling con dos pines de salida led junto con las senales de los 2 encoder.
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

#include <asf.h>

void configure_pins(void);

void configure_pins(void)
{
	// Habilitar el reloj para el PIOB
	pmc_enable_periph_clk(ID_PIOB);	
	pio_configure(PIOB,PIO_OUTPUT_1,PIO_PB27,PIO_DEFAULT);
}


int main (void)
{
	sysclk_init();
	board_init();
	configure_pins();
	while (1) 
	{
		pio_toggle_pin(PIO_PB27_IDX);
		delay_ms(500);
	}
}

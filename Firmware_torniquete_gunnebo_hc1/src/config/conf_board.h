/**
 * \file
 *
 * \brief User board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H


	// Cristal principal (12 MHz)
	#define BOARD_FREQ_MAINCK_XTAL       12000000UL
	#define BOARD_FREQ_MAINCK_BYPASS     12000000UL
	#define BOARD_OSC_STARTUP_US         500000

	// Slow Clock (SLCK): típicamente 32.768 kHz si se usa externo
	#define BOARD_FREQ_SLCK_XTAL         32768UL
	#define BOARD_FREQ_SLCK_BYPASS       32768UL
#endif // CONF_BOARD_H

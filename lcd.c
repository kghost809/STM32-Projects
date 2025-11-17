
#include "lcd.h"
#include "common.h"
#include <stdint.h>

// TODO: Move button pin initialization to its own file or function.

/*	LCD Initialization
 * Currently uses PB6 and PB7 for R/S and EN respectively.
 * 
 * Uses PC4-PC7 for LCD D4-D7, and PC0-PC3 for the buttons.
 */
void LCD_ports_init(void) {
	RCC->AHB1ENR |=  0x06;          /* enable GPIOB/C clock */

	/* PORTB 6 for LCD R/S */
	/* PORTB 7 for LCD EN */
	GPIOB->MODER &= ~0x0000F000;    /* clear pin mode */
	GPIOB->MODER |=  0x00005000;    /* set pin output mode */
	GPIOB->BSRR = EN << 16;       /* turn off EN */

	// PC4-PC7 for LCD D4-D7, respectively.
	GPIOC->MODER &= ~0x0000FF00;    /* clear pin mode */
	GPIOC->MODER |=  0x00005500;    /* set pin output mode */
}

void LCD_nibble_write(char data, bool control) {
	/* populate data bits */
	GPIOC->BSRR = 0x00F00000;       /* clear data bits */
	GPIOC->BSRR = data & 0xF0;      /* set data bits */

	/* set R/S bit */
	if (control)
		GPIOB->BSRR = RS;
	else
		GPIOB->BSRR = RS << 16;

	/* pulse E */
	GPIOB->BSRR = EN;
	delayMs(1);
	GPIOB->BSRR = EN << 16;
}

void LCD_command(unsigned char command) {
	LCD_nibble_write(command & 0xF0, 0);    /* upper nibble first */
	LCD_nibble_write(command << 4, 0);      /* then lower nibble */

	if (command < 4)
			delayMs(2);         /* command 1 and 2 needs up to 1.64ms */
	else
			delayMs(1);         /* all others 40 us */
}

void LCD_data(char data) {
	LCD_nibble_write(data & 0xF0, true);      /* upper nibble first */
	LCD_nibble_write(data << 4, true);        /* then lower nibble */

	delayMs(1);
}

void LCD_set_custom_char(int idx, uint8_t* c) {
	// Set initial CGRAM address
	LCD_command(ADDR(0b01000000 | (idx << 3)));
	for (int i = 0; i < 8; i++) {
		LCD_data(c[i]);
	}
}

void LCD_init(void) {
	LCD_ports_init();

	delayMs(20);                /* LCD controller reset sequence */
	LCD_nibble_write(0x30, false);
	delayMs(5);
	LCD_nibble_write(0x30, false);
	delayMs(1);
	LCD_nibble_write(0x30, false);
	delayMs(1);

	LCD_nibble_write(0x20, false);  /* use 4-bit data mode */
	delayMs(1);
	LCD_command(0x28);          /* set 4-bit data, 2-line, 5x7 font */
	LCD_command(0x06);          /* move cursor right */
	LCD_command(0x01);          /* clear screen, move cursor to home */
	LCD_command(0x0F);          /* turn on display, cursor blinking */
}

void LCD_clear() {
	LCD_command(CLR);
}

inline uint8_t lcd_index_to_ptr(uint8_t idx) {
	// Convert memory address pointer to an index into `LCDdata`
	return (idx & 0x0F) | ((idx > 0x0F) ? 0xC0 : 0x80);
}

void LCD_write_string(char* str, LCDLine line) {
	switch (line) {
		case Line0: {
			LCD_command(ADDR(0x80));
		} break;
		case Line1: {
			LCD_command(ADDR(0xC0));
		} break;
	}
	for (int i = 0; i < 16 && str[i]; i++) {
		LCD_data(str[i]);
	}
}
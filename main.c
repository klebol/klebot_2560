/*
 * main.c
 *
 *  Created on: 6 paü 2022
 *      Author: Michal Klebokowski
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "L293D/l293d.h"
#include "nRF24/nRF24.h"
#include "nRF24/SPI/SPI.h"
#include "OLED/I2C/TWI.h"
#include "OLED/lcd.h"
#include "UART/uart.h"
#include "klebot.h"

#define LED_BUILTIN (1<<PB7);
uint8_t address0[3] = {1,2,3};
uint8_t address1[3] = {3,2,1};
uint8_t length;
uint8_t ReceivedData[32];

int main (void)
{
	DDRB |= LED_BUILTIN;
	PORTD |= (1<<PD3); 	// INT3 pullup
	EIMSK |= (1<<INT3);	//int3 interrupt

	Klebot_Init();
	I2C_init();
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();


	SPI_Init();
	nRF24_Init();
	nRF24_SetRXAddress(0, address0);
	nRF24_SetTXAddress(address1);
	nRF24_RX_Mode();
	sei();

	lcd_puts("ELO");


	for(uint8_t i = 0; i < 4; i++)
	{
		L293D_SetMotorSpeed(i,250);
	}

	while(1)
	{
		L293D_Event();
		nRF24_Event();
		Klebot_PerformInstruction(ReceivedData);
	}
}


ISR(INT3_vect)
{
	//PORTB |=LED_BUILTIN;
	nRF24_IRQ_Handler();
	//PORTB &=~LED_BUILTIN;

}

ISR(TIMER0_COMPA_vect)
{
	if(SoftTimer1)
	{
		SoftTimer1--;
	}
	if(SoftTimer2)
	{
		SoftTimer2--;
	}
}

void nRF24_EventRxCallback(void)
{
	nRF24_ReadRXPaylaod(ReceivedData, &length);
}


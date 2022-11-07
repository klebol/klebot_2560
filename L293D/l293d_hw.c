/*
 * l293d_hw.c
 *
 *  Created on: 9 sie 2022
 *      Author: Michal
 */

#include <avr/io.h>
#include "l293d_hw.h"

void HW_Timer1InitPWM(uint8_t InitialFill)
{
//START OF HARDWARE INITIALIZATION (SPECYFIC FOR EVERY MCU)
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1); 	//Clear OC1A and OC1B on copmare match
	TCCR1A |= (1<<WGM10);
	TCCR1B |= (1<<WGM12);					//Fast PWM 8-bit mode
	TCCR1B |= (1<<CS11) | (1<<CS10);		//Preskaler: F_cpu / 64 = F_timer
//END OF HARDWARE INITIALIZATION
	RL_PWM_REGISTER = InitialFill;
	RR_PWM_REGISTER = InitialFill;
	RL_PWM_DIR_REG |= RL_PWM_PIN;
	RR_PWM_DIR_REG |= RR_PWM_PIN;
}

void HW_Timer4InitPWM(uint8_t InitialFill)
{
//START OF HARDWARE INITIALIZATION (SPECYFIC FOR EVERY MCU)
	TCCR4A |= (1<<COM4A1) | (1<<COM4B1); 	//Clear OC1A and OC1B on copmare match
	TCCR4A |= (1<<WGM40);
	TCCR4B |= (1<<WGM42);					//Fast PWM 8-bit mode
	TCCR4B |= (1<<CS41) | (1<<CS40);		//Preskaler: F_cpu / 64 = F_timer
//END OF HARDWARE INITIALIZATION
	FL_PWM_REGISTER = InitialFill;
	FR_PWM_REGISTER = InitialFill;
	FL_PWM_DIR_REG |= RL_PWM_PIN;
	FR_PWM_DIR_REG |= RR_PWM_PIN;
}


void HW_MotorDirPinsInit(void)
{
	FL_ENABLE1_DIR_REG |= FL_ENABLE1_PIN;
	FL_ENABLE2_DIR_REG |= FL_ENABLE2_PIN;
	FR_ENABLE1_DIR_REG |= FR_ENABLE1_PIN;
	FR_ENABLE2_DIR_REG |= FR_ENABLE2_PIN;
	RL_ENABLE1_DIR_REG |= RL_ENABLE1_PIN;
	RL_ENABLE2_DIR_REG |= RL_ENABLE2_PIN;
	RR_ENABLE1_DIR_REG |= RR_ENABLE1_PIN;
	RR_ENABLE2_DIR_REG |= RR_ENABLE2_PIN;

}

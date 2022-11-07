/*
 * klebot.c
 *
 *  Created on: 21 wrz 2022
 *      Author: miqix
 */

#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "klebot.h"
#include "nRF24/nRF24.h"
#include "nRF24/SPI/SPI.h"
#include "UART/uart.h"
#include "OLED/lcd.h"
#include "OLED/I2C/TWI.h"
#include "L293D/l293d.h"
#include "klebot_movement.h"



void HW_Timer0Init(void)
{
	TCCR0A |= (1<<WGM01);				//Timer0 CTC Mode
	TCCR0B |= (1<<CS00) | (1<<CS02);	//Preskaler Fcpu/1024
	OCR0A = 156;						//Compare value, in this case, one count takes ~10ms (100Hz)
	TIMSK0 |= (1<<OCIE0A); 				//enable interrupt from OCR0A compare match
}


void Klebot_Init(void)
{
	HW_Timer0Init();
	L293D_Init();
	Movement_Init();
}

void Klebot_PerformInstruction(uint8_t *InstructionsData)
{
	static uint8_t MovementDurationExtend = OFF;
	uint8_t MotorNumber;

	uint8_t *InstructionIdentifier;					//copy address of instruction identifier to use it in switch and clear it after instructions execution
	InstructionIdentifier = InstructionsData;		//thanks to this every new instruction will perform only once when this function is called from main loop

	InstructionsData++;								//Icrease pointer to second byte in the data frame to use it for executing instructions specified by instruction identifier

	uint8_t i;
	switch (*InstructionIdentifier)					//first byte in Klebot's frame is instruction identifier
	{

	case INSTRUCTION_CLEARED:

		if(SoftTimer1 == 0 && MovementDurationExtend == ON)	//extending duration of motors movement
		{
			Movement_SetDrivePattern(StopMove);
			MovementDurationExtend = OFF;
		}
		return;

	case STEERING_MANUAL:
															//Next 4 bytes in frame after identifier are each motor direction data
		for(i = 0; i < 4; i++)								//Set direction for all motors (look L239D enums, 0 = Front_L, 1 = Front_R ... )
		{
			L293D_SetMotorDirection(i,*InstructionsData);
			InstructionsData++;
		}
		break;

	case PWM_SET_MANUAL:
															//Second byte in this frame is motor number, third byte its pwm value which has to be set for that motor
		MotorNumber = *InstructionsData;					//Save motor number
		InstructionsData++;									//Increase pointer to third byte ine the frame (PWM value)
		L293D_SetMotorSpeed(MotorNumber,*InstructionsData);	//pass these arguments to pwm setting functions
		break;

	case STEERING_PATTERN:

		Movement_SetDrivePattern(*InstructionsData);
		break;
	}

	MovementDurationExtend = ON;
	SoftTimer1 = MOTOR_INSTRUCTION_DURATION;
	*InstructionIdentifier = INSTRUCTION_CLEARED;	//after all, clear first byte of frame to prevent repetition of the same instruction in main loop
													//(look at switch , case INSTRUCTION_CLEARED: return)
}




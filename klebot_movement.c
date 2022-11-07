/*
 * klebot_movement.c
 *
 *  Created on: 9 paü 2022
 *      Author: miqix
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include "L293D/l293d.h"
#include "klebot_movement.h"


// ** Robot Movement Patterns **
// This array contains movement patterns for robot, every pattern has instruction for every motor
const MotorDirection_t MovementPatterns [9][4] PROGMEM = {
		{Stop,Stop,Stop,Stop},						//Stop Move
		{Forward,Forward,Forward,Forward},			//Drive Forward Straight
		{Backward,Backward,Backward,Backward},		//Drive Backward Straight
		{Stop,Forward,Stop,Forward},				//Drive Forward Left
		{Forward,Stop,Forward,Stop},				//Drive Forward Right
		{Stop,Backward,Stop,Backward},				//Drive Backward Left
		{Backward,Stop,Backward,Stop},				//Drive Backward Right
		{Backward,Forward,Backward,Forward},		//Rotate Left
		{Forward,Backward,Forward,Backward}			//Rotate Right
};

void Movement_SetDrivePattern (Klebot_MovementPattern_t PatternID)
{
	uint8_t i;
	for(i = 0; i < 4; i++)
	{
		if(PatternID != StopMove) 											//no need for edit pwm for pattern in which no motor is moving
			{
			L293D_SetMotorSpeed(i,PatternsPwmValues[PatternID - 1]);
			}
		L293D_SetMotorDirection(i, pgm_read_byte(&MovementPatterns[PatternID][i]) );	//passing data read from flash
	}
}

void Movement_EditPatternPWM (Klebot_MovementPattern_t PatternID ,uint8_t PWMvalue)
{
	PatternsPwmValues[PatternID - 1] = PWMvalue;
}

void Movement_Init(void)
{
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		PatternsPwmValues[i] = 155;  //nie dziala
	}
}





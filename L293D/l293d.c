/*
 * l293d.c
 *
 *  Created on: 9 sie 2022
 *      Author: Michal Klebokowski
 */

#include <avr/io.h>
#include "l293d.h"
#include "l293d_hw.h"
#include "D:\Embedded\Eclipse_workspace\28_Klebot_2560\UART\uart.h"
#include "D:\Embedded\Eclipse_workspace\28_Klebot_2560\OLED\lcd.h"


static uint8_t l293d_movement_change_flag;

MotorData_t MotorsDataArrayCurrent[4];

void L293D_Init(void)
{
	HW_Timer1InitPWM(190);
	HW_Timer4InitPWM(190);
	HW_MotorDirPinsInit();
}

void L293D_WriteMotorForward(MotorNumb_t MotorNumber)
{
	switch(MotorNumber)
	{
	case Front_L:
		FL_FORWARD;
		break;

	case Front_R:
		FR_FORWARD;
		break;

	case Rear_L:
		RL_FORWARD;
		break;

	case Rear_R:
		RR_FORWARD;

		break;
	}
}

void L293D_WriteMotorBackward(MotorNumb_t MotorNumber)
{
	switch(MotorNumber)
	{
	case Front_L:
		FL_BACKWARD;
		break;

	case Front_R:
		FR_BACKWARD;
		break;

	case Rear_L:
		RL_BACKWARD;
		break;

	case Rear_R:
		RR_BACKWARD;

		break;
	}
}

void L293D_WriteMotorStop(MotorNumb_t MotorNumber)
{
	switch(MotorNumber)
	{
	case Front_L:
		FL_STOP;
		break;

	case Front_R:
		FR_STOP;
		break;

	case Rear_L:
		RL_STOP;
;
		break;

	case Rear_R:
		RR_STOP;

		break;
	}
}

void L293D_WritePWM(MotorNumb_t MotorNumber)
{
	switch(MotorNumber)
	{
	case Front_L:
		FL_PWM_REGISTER = MotorsDataArrayCurrent[MotorNumber].PwmValue;
		break;

	case Front_R:
		FR_PWM_REGISTER = MotorsDataArrayCurrent[MotorNumber].PwmValue;
		break;

	case Rear_L:
		RL_PWM_REGISTER = MotorsDataArrayCurrent[MotorNumber].PwmValue;
		break;

	case Rear_R:
		RR_PWM_REGISTER = MotorsDataArrayCurrent[MotorNumber].PwmValue;
		break;
	}
}

void L293D_WriteDataToMotor(MotorNumb_t MotorNumber)
{
	uint8_t Direction;
	Direction = MotorsDataArrayCurrent[MotorNumber].Direction;

	L293D_WritePWM(MotorNumber);

	switch(Direction)
	{
	case Stop:
		L293D_WriteMotorStop(MotorNumber);
		break;

	case Forward:
		L293D_WriteMotorForward(MotorNumber);
		break;

	case Backward:
		L293D_WriteMotorBackward(MotorNumber);
		break;
	}

}

void L293D_SetMotorSpeed(MotorNumb_t MotorNumber, uint8_t pwm)
{
	if(MotorsDataArrayCurrent[MotorNumber].PwmValue == pwm) return;	//if new value are the same as current, return
	MotorsDataArrayCurrent[MotorNumber].PwmValue = pwm;				//Edit PWM of chosen motor
	l293d_movement_change_flag |= (1<<MotorNumber);					//Set bit in flag corresponding to motor number


}

void L293D_SetMotorDirection(MotorNumb_t MotorNumber, MotorDirection_t direction)
{
	if(MotorsDataArrayCurrent[MotorNumber].Direction == direction) return;
	MotorsDataArrayCurrent[MotorNumber].Direction = direction;
	l293d_movement_change_flag |= (1<<MotorNumber);
	for (uint8_t i = 0; i < 4; i++)
					{
						lcd_gotoxy(3,i);
						lcd_puti(MotorsDataArrayCurrent[i].Direction);

					}
}

void L293D_Event(void)
{
	if(l293d_movement_change_flag == 0) return;		//return if there was no changes in motors data


	if(l293d_movement_change_flag & (1<<Front_L))	//if flag was set for Front_L motor (0x01)
	{
		L293D_WriteDataToMotor(Front_L);
	}

	if(l293d_movement_change_flag & (1<<Front_R))  	//if flag was set for Front_L motor (0x02)
	{
		L293D_WriteDataToMotor(Front_R);
	}

	if(l293d_movement_change_flag & (1<<Rear_L))	//if flag was set for Front_L motor (0x04)
	{
		L293D_WriteDataToMotor(Rear_L);
	}

	if(l293d_movement_change_flag & (1<<Rear_R))	//if flag was set for Front_L motor (0x08)
	{
		L293D_WriteDataToMotor(Rear_R);
	}

	l293d_movement_change_flag = 0;

}

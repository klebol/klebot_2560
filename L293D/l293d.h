/*
 * l293d.h
 *
 *  Created on: 9 sie 2022
 *      Author: Michal Klebokowski
 */

#ifndef L293D_L293D_H_
#define L293D_L293D_H_
#include <avr/pgmspace.h>

//
//	Enums
//
typedef enum {
	Front_L,
	Front_R,
	Rear_L,
	Rear_R
}MotorNumb_t;

typedef enum {
	Stop,
	Forward,
	Backward,
	Right,
	Left,
}MotorDirection_t;

//
//Struct for keeping motor data
//
typedef struct{
	uint8_t Direction;
	uint8_t PwmValue;
}MotorData_t;

//
//	Functions
//

void L293D_Init(void);								//Initialization (PWM and direction pins)

void L293D_WriteMotorForward(MotorNumb_t MotorNumber);
void L293D_WriteMotorBackward(MotorNumb_t MotorNumber);
void L293D_WriteMotorStop(MotorNumb_t MotorNumber);
void L293D_WritePWM(MotorNumb_t MotorNumber);
void L293D_WriteDataToMotor(MotorNumb_t MotorNumber);
void L293D_SetMotorSpeed(MotorNumb_t MotorNumber, uint8_t pwm);
void L293D_SetMotorDirection(MotorNumb_t MotorNumber, MotorDirection_t direction);
void L293D_Event(void);



#endif /* L293D_L293D_H_ */

/*
 * klebot_movement.h
 *
 *  Created on: 9 paü 2022
 *      Author: miqix
 */

#ifndef KLEBOT_MOVEMENT_H_
#define KLEBOT_MOVEMENT_H_



// ** Movement Patterns names Enum **
// Designed for use with MovementPatterns array
typedef enum {
	StopMove,
	DriveForwardStraight,
	DriveBackwardStraight,
	DriveForwardLeft,
	DriveForwardRight,
	DriveBackwardLeft,
	DriveBackwardRight,
	RotateLeft,
	RotateRight
}Klebot_MovementPattern_t;

// ** Robot Movement Patterns **
// (more info in .c file)
extern const MotorDirection_t MovementPatterns [9][4] PROGMEM;

//** Movement Patterns PWM Values array**
//This array stores PWM values for every movement pattern
uint8_t PatternsPwmValues [8];


void Movement_SetDrivePattern (Klebot_MovementPattern_t PatternID);
void Movement_EditPatternPWM (Klebot_MovementPattern_t PatternID ,uint8_t PWMvalue);
void Movement_Init(void);






void Movement_DrivePattern (Klebot_MovementPattern_t PatternID);

#endif /* KLEBOT_MOVEMENT_H_ */

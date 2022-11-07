/*
 * klebot.h
 *
 *  Created on: 21 wrz 2022
 *      Author: miqix
 */

#ifndef KLEBOT_H_
#define KLEBOT_H_

//
// Configuration
//

//Duration of motors being in action after receiving movement instruction,
//after that time motors will be stopped           [ms]
#define MOTOR_INSTRUCTION_DURATION 10

//
// Transmision Instruction codes
//

#define INSTRUCTION_CLEARED 0x00
#define STEERING_MANUAL 	0xA1
#define PWM_SET_MANUAL 		0XA2
#define STEERING_PATTERN	0xA3

//
//
//

#define ON 1
#define OFF 0

volatile uint8_t SoftTimer1;
volatile uint8_t SoftTimer2;

void Klebot_Init(void);
void HW_Timer0Init(void);
void Klebot_PerformInstruction(uint8_t *InstructionsData);

#endif /* KLEBOT_H_ */

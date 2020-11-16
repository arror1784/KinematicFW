/*
 * common.h
 *
 *  Created on: 2019. 1. 29.
 *      Author: JeonSeungHwan
 */

#ifndef STPMT_INC_COMMON_H_
#define STPMT_INC_COMMON_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

//#define LOGGING

#define FW_NUMBER 0.0.0

#define FALSE						(uint8_t)0
#define TRUE  						(uint8_t)1

#define MOTER_DRIVER 				A4988

#define MOTOR_PRESCALER					1024-1
#define MOTOR_MAX_SPEED					150
#define MOTOR_MIN_SPEED					20
#define MOTOR_ACCEL 					0
#define MOTOR_DECEL						0
#define MOTOR_MOTOR_STEP				200
#define MOTOR_MICRO_STEP				16
#define MOTOR_SCREW_PITCH				2000 // unit : micro
#define MOTOR_SCREW_PITCH_MIL			2

#define AUTO_REBOOT						1

typedef uint8_t                        bool;

typedef enum{

	STATE_ACCEL = 0,
	STATE_DECEL = 1,
	STATE_STAND = 2,
	STATE_STOP = 3,
	STATE_SOFTSTOP = 4,
	STATE_HARDSTOP = 5,
	STATE_INFINITE = 6,
	STATE_FINISH_AUTO_HOME = 7

}MotorState_t;

typedef enum{

	DIR_FORWORD = 1,
	DIR_BACKWORD = 0,
	DIR_UNKNOWN = 2

}MotorDirection_t;

typedef enum{
	BTN_END = 1,
	BTN_FRT_SHORT,
	BTN_FRT_LONG,
	BTN_FRT_LONG_LONG
}BtnChannel_t;

#endif /* STPMT_INC_COMMON_H_ */

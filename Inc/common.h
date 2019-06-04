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


#define FW_NUMBER 0.0.0

#define FALSE						(uint8_t)0
#define TRUE  						(uint8_t)1

#define MOTER_DRIVER 				A4988

#define MAX_SPEED 5000
#define MIN_SPEED 800

#define FRT_TOUCH_SENSOR_Pin GPIO_PIN_13
#define FRT_TOUCH_SENSOR_GPIO_Port GPIOB

#define COOLING_FAN_Pin GPIO_PIN_14
#define COOLING_FAN_GPIO_Port GPIOB

//#define UV_LCD_Pin GPIO_PIN_15
//#define UV_LCD_GPIO_Port GPIOB

#define FRT_NOE_PX_SIGNAL_Pin GPIO_PIN_0
#define FRT_NOE_PX_SIGNAL_GPIO_Port GPIOA

//#define MOTOR_1_DIR_GPIOX			GPIOB
//#define MOTOR_1_DIR_PINX			GPIO_PIN_15

//#define MOTOR_1_ENABLE_GPIOX		GPIOC
//#define MOTOR_1_ENABLE_PINX			GPIO_PIN_9

#define MOTOR_1_PULSE_TIM
#define MOTOR_1_PULSE_TIM_CHANNEL

#define MOTOR_1_ENDSTOP_IRQN		EXTI0_IRQn
#define MOTOR_1_ENDSTOP_PIN			GPIO_PIN_7

#define MOTOR_DRIVER_MIN_STEP_DELAY		1
#define MOTOR_DRIVER_MiCRO_STEP			(uint32_t)16
#define MOTOR_STEP						(uint32_t)200
#define MOTOR_SCREW_PITCH				2
#define MOTOR_SCREW_LENGHT				500 // unit : mm

#define MOTOR_PRESCALER					1024-1
#define MOTOR_MAX_SPEED					0
#define MOTOR_MIN_SPEED					3200
#define MOTOR_ACCEL 					0
#define MOTOR_DECEL						0
#define MOTOR_MOTOR_STEP				200
#define MOTOR_MICRO_STEP				16

typedef uint8_t						bool;

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

typedef struct{

	uint8_t deviceNumber;
	TIM_HandleTypeDef* timHandle;
	TIM_TypeDef* instance;
	HAL_TIM_ActiveChannel timChaanel;
	IRQn_Type IRQn;
	GPIO_TypeDef* port;
	uint16_t pin;
	volatile bool isActivate;

	volatile uint32_t timPrescaler;

}PWMmotor_t;

typedef struct{

	volatile MotorDirection_t direction;
	volatile MotorState_t state;

	volatile int32_t curPosition; //unit : milli.....step.....??..rotation??
	volatile uint32_t nStep;
	volatile uint32_t targetStep;

	volatile uint32_t accel;
	volatile uint32_t decel;

	volatile uint32_t curSpeed;
	volatile uint32_t maxSpeed;
	volatile uint32_t minSpeed;

	volatile uint32_t endAccel;
	volatile uint32_t startDecel;
	volatile uint32_t accu;

	//volatile uint8_t microStep;
	//volatile uint8_t motorStep;
	//volatile uint8_t screwPitch

}STMotorParam;

typedef struct{

	volatile PWMmotor_t motorHandler;
	volatile STMotorParam motorParam;

}STMotorHandle_t;


#endif /* STPMT_INC_COMMON_H_ */

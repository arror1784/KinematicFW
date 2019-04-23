/*
 * A4988.h
 *
 *  Created on: 2019. 1. 28.
 *      Author: JeonSeungHwan
 */

#ifndef STPMT_INC_STEPMOTORDRIVER_H_
#define STPMT_INC_STEPMOTORDRIVER_H_

#include "common.h"

//#include "PWMStepMotor.h"

//#define STEP_MOTOR_DEVICE_INFO

//#ifdef STEP_MOTOR_DEIVCE_INFO

typedef struct{

	void (*SetDirectionGPIO)(STMotorHandle_t *, MotorDirection_t);
	void (*SetEnableGPIO)(STMotorHandle_t *, bool);
	void (*SetSleepGPIO)(STMotorHandle_t *, bool);
	void (*SetResetGPIO)(STMotorHandle_t *,bool);
	void (*SetMicroStep)(STMotorHandle_t *,uint16_t);
	void (*SetEnableIRQ)(STMotorHandle_t *,bool);

	void (*FinishCallBack)(STMotorHandle_t *);

}STMotorDeviceControl_t;

/// Maximum speed in step/s for device 0 (30 step/s < Maximum speed <= 10 000 step/s )

void SetDirectionGPIO(STMotorHandle_t* STMotorHandle, MotorDirection_t direction);
void SetEnableGPIO(STMotorHandle_t* STMotorHandle, bool flag);
void SetSleepGPIO(STMotorHandle_t* STMotorHandle, bool flag);
void SetResetGPIO(STMotorHandle_t* STMotorHandle,bool flag);
void SetMicroStep(STMotorHandle_t* STMotorHandle,uint16_t microStep);
void SetEnableIRQ(STMotorHandle_t *STMotorHandle,bool flag);

void FinishCallBack(STMotorHandle_t *STMotorHandle);

void PWMPulseInterruptHandle(TIM_HandleTypeDef *htim);
void EXTInterruptHandle(void);

//#endif

#endif /* STPMT_INC_STEPMOTORDRIVER_H_ */

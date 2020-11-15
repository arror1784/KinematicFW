/*
 * A4988.h
 *
 *  Created on: 2019. 1. 28.
 *      Author: JeonSeungHwan
 */

#ifndef STPMT_INC_STEPMOTORDRIVER_H_
#define STPMT_INC_STEPMOTORDRIVER_H_

#include "common.h"
#include "PWMStepMotor.h"

//#include "PWMStepMotor.h"

// Maximum speed in step/s for device 0 (30 step/s < Maximum speed <= 10 000 step/s )

void SetDirectionGPIO(STMotorHandle_t* STMotorHandle, MotorDirection_t direction);
void SetEnableGPIO(STMotorHandle_t* STMotorHandle, bool flag);
void SetSleepGPIO(STMotorHandle_t* STMotorHandle, bool flag);
void SetResetGPIO(STMotorHandle_t* STMotorHandle,bool flag);
void SetMicroStep(STMotorHandle_t* STMotorHandle,uint16_t microStep);
void SetEnableIRQ(STMotorHandle_t *STMotorHandle,bool flag);

void FinishCallBack(STMotorHandle_t *STMotorHandle);

void PWMPulseInterruptHandle(TIM_HandleTypeDef *htim);
void EXTInterruptHandle(BtnChannel_t btn);

extern STMotorHandle_t STMotorDevices[1];

//#endif

#endif /* STPMT_INC_STEPMOTORDRIVER_H_ */

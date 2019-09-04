/*
 * PWMStepMotor.h
 *
 *  Created on: 2019. 1. 25.
 *      Author: JeonSeungHwan
 */

#ifndef STPMT_INC_PWMSTEPMOTOR_H_
#define STPMT_INC_PWMSTEPMOTOR_H_

#include <stdint.h>
#include "common.h"
#include "StepMotorDriver.h"
#include "stm32f4xx_hal.h"

#ifndef MOTER_DRIVER
#error MAKE StepMotorDriver.h/.c file or define MOTER_DRIVER
#endif

extern STMotorDeviceControl_t STMotorDeviceControl;
extern uint8_t endStopSignal;

bool STMotorInitControl(void);
bool STMotorInitHandler(STMotorHandle_t* STMotorHandle,TIM_HandleTypeDef* Handle,HAL_TIM_ActiveChannel Chaanel,IRQn_Type IRQn);
bool STMotorInitParam(STMotorHandle_t* STMotorHandle, uint32_t accel,uint32_t decel,uint32_t maxSpeed,uint32_t minSpeed);

uint32_t STMotorGetPosition(STMotorHandle_t* STMotorHandle);
uint32_t STMotorGetCurSpeed(STMotorHandle_t* STMotorHandle);
uint8_t STMotorGetMicroStep(STMotorHandle_t* STMotorHandle);
uint8_t STMotorGetMotorStep(STMotorHandle_t* STMotorHandle);
uint32_t STMotorGetEnable(STMotorHandle_t* STMotorHandle);
uint16_t STMotorGetDeviceNum(STMotorHandle_t* STMotorHandle);

uint16_t STMotorSetDeviceNum(STMotorHandle_t* STMotorHandle,uint16_t deviceNum);

uint32_t STMotorSetMicroStep(STMotorHandle_t* STMotorHandle, uint32_t microStep);
uint32_t STMotorSetMotorStep(STMotorHandle_t* STMotorHandle, uint32_t motorStep);
uint32_t STMotorSetMaxSpeed(STMotorHandle_t* STMotorHandle, uint32_t maxSpeed);
uint32_t STMotorSetMinSpeed(STMotorHandle_t* STMotorHandle, uint32_t minSpeed);
uint32_t STMotorSetMinSpeed(STMotorHandle_t* STMotorHandle, uint32_t minSpeed);
uint32_t STMotorSetAccelSpeed(STMotorHandle_t* STMotorHandle, uint32_t accel);
uint32_t STMotorSetDecelSpeed(STMotorHandle_t* STMotorHandle, uint32_t decel);


uint32_t STMotorClearCurPosition(STMotorHandle_t* STMotorHandle);

uint32_t STMotorCalcAccelSpeed(STMotorHandle_t* STMotorHandle,uint32_t nStep);

uint32_t STMotorMoveStart(STMotorHandle_t* STMotorHandle);

uint32_t STMotorGoStep(STMotorHandle_t* STMotorHandle,int32_t step);
//uint32_t STMotorGoMilli(STMotorHandle_t* STMotorHandle,double milli);
uint32_t STMotorGoMicro(STMotorHandle_t* STMotorHandle,int32_t micro);
uint32_t STMotorGoRotation(STMotorHandle_t* STMotorHandle,double rotation);
uint32_t STMotorHardStop(STMotorHandle_t* STMotorHandle);
uint32_t STMotorSoftStop(STMotorHandle_t* STMotorHandle);

uint32_t STMotorGoSpeed(STMotorHandle_t* STMotorHandle,int32_t speed,uint16_t timeOut);

uint32_t STMotorAutoHome(STMotorHandle_t* STMotorHandle,int32_t speed);

uint32_t STMotorWaitingActivate(STMotorHandle_t* STMotorHandle,uint32_t timeOut);
uint8_t STMotorIsActivate(STMotorHandle_t* STMotorHandle);

uint32_t STMotorGoHome(STMotorHandle_t* STMotorHandle);
uint32_t STMotorSetHome(STMotorHandle_t* STMotorHandle);

//uint32_t STMotorSetMark(STMotorHandle_t* STMotorHandle,uint16_t mark);
//uint32_t STMotorGoMark(STMotorHandle_t* STMotorHandle,uint16_t mark);

uint32_t STMotorSetFreq(STMotorHandle_t* STMotorHandle,uint32_t freq);
uint32_t STMotorStopFreq(STMotorHandle_t* STMotorHandle);

double STMotorCalcStepToRotation(uint32_t nStep);
int32_t STMotorCalcRotationToStep(double rotation);

int32_t STMotorCalcStepToMicro(uint32_t nStep);
int32_t STMotorCalcMicroToStep(int32_t micro);

int32_t STMotorCalcRotationToMicro(double rotation);
double STMotorCalcMicroToRotation(int32_t micro);



uint32_t STMotorPWMPulseInterruptHandle(STMotorHandle_t* STMotorHandle);
uint32_t STMotorEXTInterruptHandle(STMotorHandle_t* STMotorHandle);

#endif /* STPMT_INC_PWMSTEPMOTOR_H_ */

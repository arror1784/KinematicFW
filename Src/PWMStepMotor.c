/*
 * PWMStepMotor.c
 *
 *  Created on: 2019. 1. 25.
 *      Author: JeonSeungHwan
 */


#include "PWMStepMotor.h"
#include "StepMotorDriver.h"
#include "usart.h"
#include "common.h"
#include "stdlib.h"

uint8_t deviceCount = 0;
uint8_t buff[100];

//extern HAL_UART_StateTypeDef huart2;

bool STMotorInitControl(void){
	//return A4988_ControlInit();
	return TRUE;
}

bool STMotorInitHandler(STMotorHandle_t* STMotorHandle, TIM_HandleTypeDef* Handle, HAL_TIM_ActiveChannel channel,IRQn_Type IRQn){

	STMotorHandle->motorHandler.timHandle = Handle;
	STMotorHandle->motorHandler.instance = Handle->Instance;
	STMotorHandle->motorHandler.timChaanel = channel;
	STMotorHandle->motorHandler.deviceNumber = deviceCount++;
	STMotorHandle->motorHandler.isActivate = FALSE;
	STMotorHandle->motorHandler.timPrescaler = MOTOR_PRESCALER;
	STMotorHandle->motorHandler.IRQn = IRQn;

	HAL_NVIC_DisableIRQ(STMotorHandle->motorHandler.IRQn);

	STMotorDeviceControl.SetEnableGPIO(STMotorHandle,FALSE);
	return TRUE;
}

bool STMotorInitParam(STMotorHandle_t* STMotorHandle, uint32_t accel,uint32_t decel,uint32_t maxSpeed,uint32_t minSpeed){

	STMotorHandle->motorParam.accel = (accel) ? accel : MOTOR_ACCEL;
	STMotorHandle->motorParam.decel = (decel) ? decel : MOTOR_DECEL;
	STMotorHandle->motorParam.maxSpeed = (maxSpeed) ? maxSpeed : MOTOR_MAX_SPEED;
	STMotorHandle->motorParam.minSpeed = (minSpeed) ? minSpeed : MOTOR_MIN_SPEED;
	//STMotorHandle->motorParam.microStep = (microStep) ? microStep : MOTOR_MICRO_STEP;
	//STMotorHandle->motorParam.motorStep = (motorStep) ? motorStep : MOTOR_MOTOR_STEP;

	STMotorHandle->motorParam.curPosition = 0;
	STMotorHandle->motorParam.direction = DIR_UNKNOWN;
	STMotorHandle->motorParam.state = STATE_STOP;

	return TRUE;
}

uint32_t STMotorGetPosition(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.curPosition;
}
uint32_t STMotorGetCurSpeed(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.curSpeed;
}
uint32_t STMotorGetMaxSpeed(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.maxSpeed;
}
uint32_t STMotorGetMinSpeed(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.minSpeed;
}
uint8_t STMotorGetMicroStep(STMotorHandle_t* STMotorHandle){
	//return STMotorHandle->motorParam.microStep;
	return MOTOR_DRIVER_MiCRO_STEP;
}
uint8_t STMotorGetMotorStep(STMotorHandle_t* STMotorHandle){
	//return STMotorHandle->motorParam.motorStep;
	return MOTOR_STEP;
}
uint8_t STMotorGetScrewPitch(STMotorDeviceControl_t* STMotorHandle){
	//return STMotorHandle->motorParam.screwPitch;
	return MOTOR_SCREW_PITCH;
}
uint16_t STMotorGetDeviceNum(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorHandler.deviceNumber;
}
uint32_t STMotorSetMaxSpeed(STMotorHandle_t* STMotorHandle, uint32_t maxSpeed){
	return STMotorHandle->motorParam.maxSpeed = maxSpeed;
}
uint32_t STMotorSetMinSpeed(STMotorHandle_t* STMotorHandle, uint32_t minSpeed){
	return STMotorHandle->motorParam.minSpeed = minSpeed;
}
uint32_t STMotorSetcurSpeed(STMotorHandle_t* STMotorHandle, uint32_t curSpeed){
	return STMotorHandle->motorParam.curSpeed = curSpeed;
}
uint16_t STMotorSetDeviceNum(STMotorHandle_t* STMotorHandle,uint16_t deviceNum){
	return STMotorHandle->motorHandler.deviceNumber = deviceNum;
}
uint32_t STMotorSetAccelSpeed(STMotorHandle_t* STMotorHandle, uint32_t accel){
	return STMotorHandle->motorParam.accel = accel;
}
uint32_t STMotorSetDecelSpeed(STMotorHandle_t* STMotorHandle, uint32_t decel){
	return STMotorHandle->motorParam.decel = decel;
}

uint32_t STMotorCalcAccelSpeed(STMotorHandle_t* STMotorHandle,uint32_t nStep){

	uint32_t accSteps;
	uint32_t decSteps;

	uint32_t minSpeed = STMotorHandle->motorParam.minSpeed;
	uint32_t maxSpeed = STMotorHandle->motorParam.maxSpeed;

	uint32_t acc = STMotorHandle->motorParam.accel;
	uint32_t dec = STMotorHandle->motorParam.decel;

	accSteps = (maxSpeed - minSpeed) * (maxSpeed + minSpeed);
	decSteps = accSteps;
	accSteps /= acc;
	accSteps /= 2;

	decSteps /= dec;
	decSteps /= 2;

	if((accSteps + decSteps) > nStep){

		decSteps = (dec * nStep) / (acc + dec);

		if(decSteps > 1){
			accSteps = decSteps - 1;
			if(accSteps == 0){
				accSteps = 1;
			}
		}else{
			accSteps = 0;
		}
		STMotorHandle->motorParam.endAccel = accSteps;
		STMotorHandle->motorParam.startDecel = decSteps;
	}else{
		STMotorHandle->motorParam.endAccel = accSteps;
		STMotorHandle->motorParam.startDecel = nStep - decSteps - 1;
	}
	return 0;
}

uint32_t STMotorSetFreq(STMotorHandle_t* STMotorHandle,uint32_t freq){
	uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
	uint32_t period = (sysFreq/ (STMotorHandle->motorHandler.timPrescaler * (uint32_t)freq)) - 1;

	__HAL_TIM_SET_AUTORELOAD(STMotorHandle->motorHandler.timHandle, period);
	__HAL_TIM_SET_COMPARE(STMotorHandle->motorHandler.timHandle, STMotorHandle->motorHandler.timChaanel, period >> 1);
	HAL_TIM_PWM_Start_IT(STMotorHandle->motorHandler.timHandle, STMotorHandle->motorHandler.timChaanel);

	return 0;
}

uint32_t STMotorStopFreq(STMotorHandle_t* STMotorHandle){
	STMotorDeviceControl.SetEnableGPIO(STMotorHandle,FALSE);
	HAL_TIM_PWM_Stop_IT(STMotorHandle->motorHandler.timHandle,STMotorHandle->motorHandler.timChaanel);

	STMotorDeviceControl.FinishCallBack(STMotorHandle);
	return 0;
}

uint32_t STMotorMoveStart(STMotorHandle_t* STMotorHandle){

	STMotorHandle->motorParam.accu = 0;
	STMotorHandle->motorParam.nStep = 0;

	STMotorDeviceControl.SetEnableGPIO(STMotorHandle,TRUE);

	if (STMotorHandle->motorParam.endAccel != 0)
	{
		STMotorHandle->motorParam.state = STATE_ACCEL;
	}
	else
	{
		STMotorHandle->motorParam.state = STATE_DECEL;
	}

	STMotorSetFreq(STMotorHandle,STMotorHandle->motorParam.minSpeed);

	return 0;
}

uint32_t STMotorGoStep(STMotorHandle_t* STMotorHandle,int32_t step){

	if(STMotorHandle->motorHandler.isActivate == FALSE){
		STMotorHandle->motorHandler.isActivate = TRUE;
		if(step == 0){
			STMotorHandle->motorHandler.isActivate = FALSE;
			return 1;
		}else if(step < 0) {
			STMotorHandle->motorParam.targetStep = -step;
			STMotorHandle->motorParam.direction = DIR_BACKWORD;
			STMotorDeviceControl.SetDirectionGPIO(STMotorHandle,DIR_BACKWORD);
		}else{
			STMotorHandle->motorParam.targetStep = step;
			STMotorHandle->motorParam.direction = DIR_FORWORD;
			STMotorDeviceControl.SetDirectionGPIO(STMotorHandle,DIR_FORWORD);
		}
		STMotorCalcAccelSpeed(STMotorHandle,STMotorHandle->motorParam.targetStep);

		STMotorMoveStart(STMotorHandle);

		return 0;
	}else{
		return 1;
	}
}

uint32_t STMotorGoSpeed(STMotorHandle_t* STMotorHandle,int16_t speed,uint16_t timeOut){
	if(STMotorHandle->motorHandler.isActivate == FALSE){
		STMotorHandle->motorHandler.isActivate = TRUE;
		if(speed == 0){
			STMotorHandle->motorHandler.isActivate = FALSE;
			return 0;
		}else if(speed < 0) {
			STMotorDeviceControl.SetDirectionGPIO(STMotorHandle,DIR_BACKWORD);

		}else{
			STMotorDeviceControl.SetDirectionGPIO(STMotorHandle,DIR_FORWORD);
		}
		STMotorHandle->motorParam.accu = 0;
		STMotorHandle->motorParam.endAccel = 0;
		STMotorHandle->motorParam.startDecel = 0;
		STMotorHandle->motorParam.targetStep = 0;
		STMotorHandle->motorParam.nStep = 0;

		STMotorHandle->motorParam.state = STATE_INFINITE;

		STMotorDeviceControl.SetEnableGPIO(STMotorHandle,TRUE);
		STMotorSetFreq(STMotorHandle,abs(speed));
		return 0;
	}else{
		return -1;
	}
}

uint32_t STMotorAutoHome(STMotorHandle_t* STMotorHandle,uint16_t speed){

	if(speed == 0){
		STMotorGoSpeed(STMotorHandle,-MOTOR_MIN_SPEED,0);
	}else{
		STMotorGoSpeed(STMotorHandle,speed,0);
	}
	HAL_NVIC_EnableIRQ(STMotorHandle->motorHandler.IRQn);
	return 0;
}

uint32_t STMotorGoMilli(STMotorHandle_t* STMotorHandle,double milli){
	return STMotorGoStep(STMotorHandle, STMotorCalcMilliToStep(milli));
}
uint32_t STMotorGoRotation(STMotorHandle_t* STMotorHandle,double rotation){
	return STMotorGoStep(STMotorHandle, STMotorCalcRotationToStep(rotation));
}

uint32_t STMotorHardStop(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.state = STATE_HARDSTOP;
}

uint32_t STMotorSoftStop(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.state = STATE_SOFTSTOP;
}

uint32_t STMotorWaitingActivate(STMotorHandle_t* STMotorHandle,uint32_t timeOut){
	uint32_t startMilli = HAL_GetTick();
	while(STMotorHandle->motorHandler.isActivate == TRUE){
		if(timeOut == 0){
			continue;
		}else if(HAL_GetTick() > startMilli + timeOut){
			STMotorHandle->motorHandler.isActivate = FALSE;
			STMotorHardStop(STMotorHandle);
			break;
		}
	}
	return 0;
}
uint8_t STMotorIsActivate(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorHandler.isActivate;
}

uint32_t STMotorGoHome(STMotorHandle_t* STMotorHandle){

	int32_t step = -STMotorHandle->motorParam.curPosition;
	if(STMotorGoStep(STMotorHandle,step) == 1){
		STMotorDeviceControl.FinishCallBack(STMotorHandle);
	}
	return 0;
}
uint32_t STMotorSetHome(STMotorHandle_t* STMotorHandle){

	STMotorHandle->motorParam.curPosition = 0;
	return 0;
}

double STMotorCalcStepToRotation(uint32_t nStep){
	return nStep/MOTOR_DRIVER_MiCRO_STEP * MOTOR_STEP;
}
int32_t STMotorCalcRotationToStep(double rotation){
	return (double)(MOTOR_DRIVER_MiCRO_STEP * MOTOR_STEP) * rotation;
}

double STMotorCalcRotationToMilli(double rotation){
	return rotation * MOTOR_SCREW_PITCH;
}
double STMotorCalcMilliToRotation(double milli){
	return milli / (double)MOTOR_SCREW_PITCH;
}

double STMotorCalcStepToMilli(uint32_t nStep){
	return nStep/(MOTOR_DRIVER_MiCRO_STEP * MOTOR_STEP) * MOTOR_SCREW_PITCH;
}
int32_t STMotorCalcMilliToStep(double milli){
	return (milli / ((double)MOTOR_SCREW_PITCH)) * (MOTOR_DRIVER_MiCRO_STEP * MOTOR_STEP);
}




uint32_t STMotorPWMPulseInterruptHandle(STMotorHandle_t* STMotorHandle){

	uint32_t relStep = ++STMotorHandle->motorParam.nStep;
	uint32_t targetStep = STMotorHandle->motorParam.targetStep;
	uint32_t speed = STMotorHandle->motorParam.curSpeed;
	uint32_t accel = STMotorHandle->motorParam.accel << 16;
	uint32_t decel = STMotorHandle->motorParam.decel << 16;

	STMotorHandle->motorParam.curPosition += STMotorHandle->motorParam.direction ? 1 : -1;

	bool speedUpdated = FALSE;

	switch(STMotorHandle->motorParam.state){

		case STATE_INFINITE:
//			if(STMotorHandle->motorParam.state == STATE_HARDSTOP){
//				STMotorHandle->motorParam.state = STATE_STOP;
//			}else if(STMotorHandle->motorParam.state == STATE_SOFTSTOP){
//				STMotorHandle->motorParam.state = STATE_DECEL;
//			}
			break;
		case STATE_ACCEL:
			if(relStep >= targetStep){
				STMotorHandle->motorParam.state = STATE_STOP;
			}else if(relStep >= STMotorHandle->motorParam.endAccel){
				STMotorHandle->motorParam.state = STATE_STAND;
			}else{
				if(speed <= 0 )
					speed = 1;

				STMotorHandle->motorParam.accu += accel / speed;
				while (STMotorHandle->motorParam.accu >= (0X10000L))
				{
					STMotorHandle->motorParam.accu -= (0X10000L);
					speed += 1;
					speedUpdated = TRUE;
				}
				if(speedUpdated == TRUE){
					speedUpdated = FALSE;
					if(speed > STMotorHandle->motorParam.maxSpeed)
						speed = STMotorHandle->motorParam.maxSpeed;
					STMotorHandle->motorParam.curSpeed = speed;
					STMotorSetFreq(STMotorHandle,speed);
				}
			}
			break;
		case STATE_STAND:
			if( relStep >= STMotorHandle->motorParam.startDecel){
				STMotorHandle->motorParam.accu = 0;
				STMotorHandle->motorParam.state = STATE_DECEL;
			}else if(relStep <= STMotorHandle->motorParam.endAccel){
				STMotorHandle->motorParam.accu = 0;
				STMotorHandle->motorParam.state = STATE_ACCEL;
			}
			break;
		case STATE_SOFTSTOP:
		case STATE_DECEL:
			if(relStep >= targetStep-1 || speed <= STMotorHandle->motorParam.minSpeed){
				STMotorHandle->motorParam.state = STATE_STOP;
			}else if(relStep <= STMotorHandle->motorParam.startDecel){
				STMotorHandle->motorParam.state = STATE_STAND;
			}else if(STMotorHandle->motorParam.state == STATE_HARDSTOP){
				STMotorHandle->motorParam.state = STATE_STOP;

			}else{

				bool speedUpdated = FALSE;
		        if (speed == 0) speed =1;

				STMotorHandle->motorParam.accu += decel / speed;
				while (STMotorHandle->motorParam.accu >= (0X10000L))
				{
					STMotorHandle->motorParam.accu -= (0X10000L);
					if (speed > 1){
						speed -=1;
					}
					speedUpdated = TRUE;
				}
				if(speedUpdated == TRUE){
					speedUpdated = FALSE;
					if(speed < STMotorHandle->motorParam.minSpeed)
						speed = STMotorHandle->motorParam.minSpeed;
					STMotorHandle->motorParam.curSpeed = speed;
					STMotorSetFreq(STMotorHandle,speed);
				}
			}
			break;
		case STATE_HARDSTOP:
		case STATE_STOP:
			//STMotorHandle->motorParam.curPosition += (STMotorHandle->motorParam.targetStep * (STMotorHandle->motorParam.direction) ? 1 : -1);
			STMotorHandle->motorParam.nStep = 0;
			STMotorHandle->motorParam.targetStep = 0;
			STMotorHandle->motorHandler.isActivate = FALSE;
			STMotorHandle->motorParam.state = STATE_STOP;
			STMotorStopFreq(STMotorHandle);
			break;
		default:
			break;
	}

	return 0;
}

uint32_t STMotorEXTInterruptHandle(STMotorHandle_t* STMotorHandle){
	HAL_NVIC_DisableIRQ(STMotorHandle->motorHandler.IRQn);
	STMotorHardStop(STMotorHandle);
	STMotorSetHome(STMotorHandle);
	return 0;
}











